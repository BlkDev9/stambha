// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
// Copyright(c) 2019-2020  Realtek Corporation
// Copyright(c) 2020  Realtek Corporation
//! An RTL8852CE kept powered after its bring-up (M3 spec S2-S3): [`start`]
//! runs the bring-up of [`super::bringup`] through `rtw89_phy_dm_init`
//! (`core.c:6697`), then what `rtw89_leave_ips` (`ps.c:257-278`) runs from
//! idle, [`leave_ips`]: the rest of `rtw89_core_start` (`core.c:6658-6725`,
//! [`core_tail`]), `rtw89_set_channel` (`core.c:600-618`) to 2.4 GHz channel
//! 1 with `__rtw89_set_channel`'s TX power between the RF part and the
//! help's exit (`core.c:568-598`, [`channel::set_channel`]), and
//! `rtw89_mac_vif_init` (`mac.c:5045`, [`vif`]); then the WD pages and
//! payload slots of the data and management queues, as
//! `rtw89_pci_alloc_tx_wd_ring` (`pci.c:3539`) allocates them at probe
//! ([`TxPools`]). The result, [`Live`], owns the register window, the rings,
//! the pools, the H2C sequence, the command session's receiver and ledger,
//! the bring-up's report, the inputs of every channel switch, and the TX
//! power tables with their regulatory index. [`Live::tune`] switches
//! channel as `__rtw89_set_channel` does, TX power included;
//! [`Live::stop`] is `rtw89_core_stop` (`core.c:6727-6767`): the coex half,
//! `rtw89_pci_ops_deinit` (`pci.c:3053`), then `rtw89_mac_pwr_off`
//! (`mac.c:1603`).
//!
//! A failure of any step after the power-on powers the chip off as the
//! bring-up's failures do and hands the window and the rings back
//! ([`StartError`]). A failed channel switch leaves the TX scheduler stopped
//! and the BB held in reset, so [`Live::tune`] powers the chip off on any
//! failure after its first access, and every later call is refused with
//! [`POWERED_OFF`].
//!
//! This module grants no transmission, scan, association or key yet: it
//! submits nothing on the data or management queues, polls nothing outside
//! its command session, and runs no IQK, DPK, TSSI, watchdog or interrupts.
//! It has no `Drop`: a live chip that is dropped keeps running, its rings
//! and pools quarantined, as rtw88's `Hardware` keeps running for the boot.
use super::bb::BbGain;
use super::bringup::{self, BringUpError, Calibrated, FirmwareFile, Host, Running};
use super::chan::Chan;
use super::channel::{self, ChannelInputs, TxPower};
use super::command::{Parked, Session};
use super::core_tail;
use super::efuse::RxGain;
use super::h2c::H2c;
use super::io::{AxError, AxIo};
use super::pci::{Rings, TxPools};
use super::receive::{Mode, Tag};
use super::regd::Regds;
use super::scan::ScanError;
use super::txpwr_tables::TxPowerTables;
use super::vif;
use crate::bus::dma::{mapping::DmaDomain, memory::PageSource};
use alloc::sync::Arc;

/// The channel `rtw89_leave_ips` sets from idle: an unassociated channel
/// context is the default chandef, 2.4 GHz channel 1 (M2c spec S1.2).
pub const BOOT_CHANNEL: Tag = Tag {
    band: super::chan::HwBand::G2,
    channel: 1,
};

/// What every call on a [`Live`] chip returns once a failed channel switch
/// powered it off.
pub const POWERED_OFF: AxError =
    AxError::Phy("the chip was powered off after a failed channel switch");

/// An RTL8852CE that [`start`] left running (M3 spec S3.1).
pub struct Live<I: AxIo> {
    io: I,
    rings: Rings,
    pools: TxPools,
    h2c: H2c,
    /// The command session's receiver and ledger between two sessions;
    /// `None` only while [`Live::session`] runs one.
    parked: Option<Parked>,
    running: Running,
    host: Host,
    gain: BbGain,
    rx_gain: RxGain,
    offset_base: i8,
    tables: TxPowerTables,
    regds: (Regds, [u8; 2]),
    /// The channel the last switch set.
    chan: Chan,
    /// A failed switch powered the chip off.
    off: bool,
}

/// A failed [`start`]: the step and its error, the register window, and the
/// rings once they were installed. The chip has been powered off; the rings
/// must be kept, as nothing proves the DMA stopped.
pub struct StartError<I> {
    pub error: BringUpError,
    pub io: I,
    pub rings: Option<Rings>,
}

/// `rtw89_leave_ips` (`ps.c:257-278`) after the bring-up's
/// `rtw89_phy_dm_init`, on `session` (M3 spec S2 steps 1-3):
/// 1. `"core start"`: the rest of `rtw89_core_start` ([`core_tail::run`]);
/// 2. `"set channel"` and `"tx power"`: `rtw89_set_channel` to `chan`,
///    `set_txpwr` between its RF part and its help's exit
///    ([`channel::set_channel`]);
/// 3. `"vif init"`: `rtw89_mac_vif_init` for `mac` ([`vif::init`]).
///
/// The first failure is returned and nothing after it runs; the caller
/// powers the chip off.
///
/// # Safety
/// The contract of `Ch12::submit` and `Rxq::poll` for `io` and the
/// session's rings: `io` is the exclusive register window of the same
/// RTL8852C requester that `pci::pre_init` programmed with them, `pre_init`
/// succeeded, and its contract still holds.
pub unsafe fn leave_ips<I: AxIo + ?Sized>(
    io: &mut I,
    session: &mut Session<'_>,
    mac: [u8; 6],
    chan: &Chan,
    inputs: &ChannelInputs<'_>,
    power: TxPower<'_>,
) -> Result<(), ScanError> {
    let at = |step: &'static str| move |error: AxError| ScanError { step, error };
    // SAFETY: the caller's contract is `core_tail::run`'s, word for word.
    unsafe { core_tail::run(io, session) }.map_err(at("core start"))?;
    channel::set_channel(io, chan, inputs, power).map_err(|e| at(e.step)(e.error))?;
    // SAFETY: as above, `vif::init`'s.
    unsafe { vif::init(io, session, mac) }.map_err(at("vif init"))
}

/// Bring an RTL8852CE of cut `cut` up on `firmware` for `host` and keep it
/// running (M3 spec S2-S3): the bring-up through `rtw89_phy_dm_init`
/// ([`super::bringup::bring_up`]'s steps 1-14, with the file's TX power
/// tables and the regulatory index of the efuse country, else
/// `network_country`, else the world's), then [`leave_ips`] on 2.4 GHz
/// channel 1 with one command session from the sequence state the RF pages
/// left (seq 3), then `"tx rings"`: the TX pools ([`TxPools::install`]), new
/// for this start, as pages in flight from an earlier run can never return.
/// The live chip's receiver then keeps every frame a station receives
/// ([`Mode::Station`]), tagged with the channel set; the H2C sequence goes
/// on at seq 17.
///
/// On a failure the chip is powered off as [`super::bringup::bring_up`]
/// powers it off for that step (a failure from `"core start"` on as a
/// calibration failure is, [`bringup::shut_down`]), and the window and the
/// rings come back in the error.
///
/// # Safety
/// [`super::bringup::bring_up`]'s contract, for as long as the live chip is
/// used: `io` is the exclusive register window of exactly the RTL8852C PCI
/// requester that `domain` isolates; `source` yields real coherent WB DMA
/// RAM for that requester, with NoSnoop and ATS disabled; one worker owns
/// the device and its queues, and no other engine, IRQ or reset worker
/// touches them.
#[expect(
    clippy::result_large_err,
    reason = "A failure hands the window and the rings back by value: the rings are never freed, as nothing proves the DMA stopped."
)]
pub unsafe fn start<I: AxIo>(
    mut io: I,
    source: &dyn PageSource,
    domain: Arc<dyn DmaDomain>,
    cut: u8,
    firmware: FirmwareFile<'_>,
    host: Host,
    network_country: Option<[u8; 2]>,
) -> Result<Live<I>, StartError<I>> {
    let pools_domain = domain.clone();
    // SAFETY: the caller's contract is `power_up`'s, word for word.
    let up = unsafe {
        bringup::power_up(
            &mut io,
            source,
            domain,
            cut,
            firmware,
            host,
            network_country,
        )
    };
    let (calibrated, mut rings) = match up {
        Ok(up) => up,
        Err((error, rings)) => return Err(StartError { error, io, rings }),
    };
    let Calibrated {
        running,
        mut h2c,
        gain,
        rx_gain,
        offset_base,
        tables,
        regds,
    } = calibrated;
    // Every failure from here on follows the calibration, so it keeps RF
    // 0x18 as the bring-up's do.
    let failed = |step: &'static str, error: AxError| BringUpError {
        rf18: running.rf18,
        ..BringUpError::new(step, error)
    };
    let inputs = ChannelInputs {
        gain: &gain,
        rx_gain: &rx_gain,
        offset_base,
        cut,
    };
    let power = TxPower {
        tables: &tables,
        regds: regds.0,
    };
    let left = match Chan::new20(BOOT_CHANNEL.band, BOOT_CHANNEL.channel) {
        Ok(chan) => {
            let mut session = Session::new(&mut rings, &mut h2c, BOOT_CHANNEL);
            // SAFETY: the caller's contract is `leave_ips`'s: `power_up`
            // programmed these rings through this window with a
            // `pci::pre_init` that succeeded, and its OFLD_CFG wait and radio
            // steps submitted and polled on them since.
            let left =
                unsafe { leave_ips(&mut io, &mut session, running.mac, &chan, &inputs, power) };
            left.map(|()| (chan, session.park()))
        }
        Err(error) => Err(ScanError {
            step: "set channel",
            error,
        }),
    };
    let left = left.map_err(|e| failed(e.step, e.error)).and_then(|left| {
        // SAFETY: the caller's `Rings::install` contract, which is
        // `TxPools::install`'s, for the rings' own requester and domain.
        let pools = unsafe { TxPools::install(source, pools_domain) };
        pools
            .map(|pools| (left, pools))
            .map_err(|e| failed("tx rings", e))
    });
    let ((chan, mut parked), pools) = match left {
        Ok(left) => left,
        Err(error) => {
            bringup::shut_down(&mut io, &rings, host);
            return Err(StartError {
                error,
                io,
                rings: Some(rings),
            });
        }
    };
    parked.rx.set_mode(Mode::Station);
    parked.rx.set_tag(tag_of(&chan));
    Ok(Live {
        io,
        rings,
        pools,
        h2c,
        parked: Some(parked),
        running,
        host,
        gain,
        rx_gain,
        offset_base,
        tables,
        regds,
        chan,
        off: false,
    })
}

/// The receiver's tag for frames on `chan`.
fn tag_of(chan: &Chan) -> Tag {
    Tag {
        band: chan.band,
        channel: chan.channel,
    }
}

impl<I: AxIo> Live<I> {
    /// What the bring-up reported (M2b2's report, no scan).
    pub fn running(&self) -> &Running {
        &self.running
    }

    /// The efuse MAC address.
    pub fn mac(&self) -> [u8; 6] {
        self.running.mac
    }

    /// The TX power's regulatory index and the alpha2 it was found by
    /// (`00` for the world's).
    pub fn regds(&self) -> (Regds, [u8; 2]) {
        self.regds
    }

    /// The channel the last switch set: [`start`]'s 2.4 GHz channel 1, then
    /// each [`Live::tune`]'s. A firmware scan takes the radio off it until a
    /// switch sets one again.
    pub fn chan(&self) -> Chan {
        self.chan
    }

    /// `__rtw89_set_channel` to `chan` ([`channel::set_channel`]): the TX
    /// scheduler stopped, the MAC, BB and RF parts, `set_txpwr` from the
    /// file's tables at [`Live::regds`], then the help's exit; then the
    /// receiver tags frames with `chan`. A channel [`Chan::new20`] would not
    /// build is refused before any I/O and changes nothing. Any later
    /// failure leaves the TX scheduler stopped, so the chip is powered off
    /// ([`bringup::shut_down`]) and every later call fails with
    /// [`POWERED_OFF`], as does this one on a chip already off.
    ///
    /// # Safety
    /// [`start`]'s contract, still holding for this chip.
    pub unsafe fn tune(&mut self, chan: &Chan) -> Result<(), AxError> {
        if self.off {
            return Err(POWERED_OFF);
        }
        channel::check_chan(chan)?;
        let inputs = ChannelInputs {
            gain: &self.gain,
            rx_gain: &self.rx_gain,
            offset_base: self.offset_base,
            cut: self.running.cut,
        };
        let power = TxPower {
            tables: &self.tables,
            regds: self.regds.0,
        };
        if let Err(failed) = channel::set_channel(&mut self.io, chan, &inputs, power) {
            bringup::shut_down(&mut self.io, &self.rings, self.host);
            self.off = true;
            return Err(failed.error);
        }
        self.chan = *chan;
        if let Some(parked) = self.parked.as_mut() {
            parked.rx.set_tag(tag_of(chan));
        }
        Ok(())
    }

    /// Run `f` on the command session, resumed from where the last one was
    /// parked, then park it again. Refused with [`POWERED_OFF`] on a chip a
    /// failed switch powered off.
    pub(super) fn session<R>(
        &mut self,
        f: impl FnOnce(&mut I, &mut Session<'_>) -> R,
    ) -> Result<R, AxError> {
        if self.off {
            return Err(POWERED_OFF);
        }
        // Taken only here, and `f` cannot reach `self`: always `Some`.
        let parked = self
            .parked
            .take()
            .ok_or(AxError::Mac("command session already in use"))?;
        let mut session = Session::resume(&mut self.rings, &mut self.h2c, parked);
        let result = f(&mut self.io, &mut session);
        self.parked = Some(session.park());
        Ok(result)
    }

    /// `rtw89_core_stop` (M3 spec S3.2, [`bringup::shut_down`]): the coex
    /// half, `pci::deinit`, then the power-off, every result ignored as
    /// Linux ignores them; nothing on a chip a failed switch already powered
    /// off. The window, the rings and the pools come back and must be kept:
    /// nothing proves the DMA stopped.
    ///
    /// # Safety
    /// [`start`]'s contract, still holding for this chip.
    pub unsafe fn stop(self) -> (I, Rings, TxPools) {
        let Self {
            mut io,
            rings,
            pools,
            host,
            off,
            ..
        } = self;
        if !off {
            bringup::shut_down(&mut io, &rings, host);
        }
        (io, rings, pools)
    }
}

#[cfg(test)]
impl<I: AxIo> Live<I> {
    pub(crate) fn io(&self) -> &I {
        &self.io
    }

    pub(crate) fn io_mut(&mut self) -> &mut I {
        &mut self.io
    }

    pub(crate) fn h2c(&self) -> H2c {
        self.h2c
    }

    pub(crate) fn receiver(&self) -> &super::receive::Receiver {
        &self.parked.as_ref().unwrap().rx
    }
}

#[cfg(test)]
mod tests;
