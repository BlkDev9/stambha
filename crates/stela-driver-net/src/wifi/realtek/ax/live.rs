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
//! A firmware scan over any list is an operation on the live chip (M3 spec
//! S1.3, S8), each step from [`scan`]: [`Live::begin_scan`] is
//! `rtw89_hw_scan_start` and `rtw89_hw_scan_offload(true)`
//! (`fw.c:10367-10437`, `:10531-10571`); [`Live::drain`] reads RXQ, where
//! END_SCAN ends the scan (`rtw89_mac_c2h_scanofld_rsp`, `mac.c:5353-5367`)
//! and the drain that brings it also drains what follows, as M2c's scan
//! does after END_SCAN; [`Live::expire_scan`]
//! stops a scan past its budget as `rtw89_hw_scan_abort` does
//! (`fw.c:10494-10511`); [`Live::end_scan`] is `rtw89_hw_scan_complete`
//! (`fw.c:10473-10492`): `rtw89_chanctx_proceed`'s `rtw89_set_channel`
//! (`chan.c:3269-3302`) back to the operating channel through
//! [`Live::tune`], TX power and receiver tag included, then
//! `rtw89_hw_scan_complete_cb`. The operating channel is the one the last
//! switch set, as Linux returns to its channel context's chandef, or to the
//! default chandef, 2.4 GHz channel 1, when none is registered
//! (`chan.c:614-616`).
//!
//! While the scan's RX filter is on, from `rtw89_hw_scan_start` to the
//! restore's filter write, the receiver keeps only beacons and probe
//! responses ([`Mode::Scan`], M2c's), and every other frame is counted, not
//! handed on. The scan's filter clears `B_AX_A_A1_MATCH`, `B_AX_A_BC` and
//! `B_AX_A_BCN_CHK_EN` (`fw.c:10424-10428`), so the chip then passes other
//! BSSes' frames too; Linux hands them all to mac80211, whose scan takes
//! only beacons and probe responses, and an unassociated station, the only
//! one that scans here (spec S1.15), acts on no other frame from a BSS it
//! has not joined. A station receiver would hand each to the station's
//! scanner, which refuses it, counted as dropped from the scan. An
//! associated scan would need the station receiver, as the AP's frames go on
//! during it in Linux.
//!
//! A scan whose offload fails is aborted as `rtw89_ops_hw_scan` aborts it
//! (`mac80211.c:1315-1319`): the stop, whose failure Linux only warns of,
//! then the restore. A failure in the scan start's own steps, which Linux's
//! cannot fail, or in the restore, powers the chip off, as a failed channel
//! switch does: the chip is left with the scan's RX filter, EDCCA, TSF sync
//! or DIG, or a stopped TX scheduler.
//!
//! This module grants no transmission, association or key yet: it submits
//! nothing on the data or management queues, polls nothing outside its
//! command session, and runs no IQK, DPK, TSSI, watchdog, interrupts, probe
//! request or idle power save. It has no `Drop`: a live chip that is dropped
//! keeps running, its rings and pools quarantined, as rtw88's `Hardware`
//! keeps running for the boot.
use super::bb::BbGain;
use super::bringup::{self, BringUpError, Calibrated, FirmwareFile, Host, Running};
use super::c2h::{C2h, SCAN_END};
use super::chan::Chan;
use super::channel::{self, ChannelInputs, TxPower};
use super::command::{Parked, Session};
use super::core_tail;
use super::efuse::RxGain;
use super::h2c::{H2c, ScanChannel};
use super::io::{AxError, AxIo};
use super::pci::{Rings, TxPools};
use super::receive::{Event, MAX_PENDING, Mode, Received, Tag};
use super::regd::Regds;
use super::scan::{self, EdccaBak, ScanEnd, ScanError};
use super::txpwr_tables::TxPowerTables;
use super::vif;
use crate::bus::dma::{mapping::DmaDomain, memory::PageSource};
use alloc::collections::VecDeque;
use alloc::sync::Arc;

/// The channel `rtw89_leave_ips` sets from idle: an unassociated channel
/// context is the default chandef, 2.4 GHz channel 1 (M2c spec S1.2).
pub const BOOT_CHANNEL: Tag = Tag {
    band: super::chan::HwBand::G2,
    channel: 1,
};

/// What every call on a [`Live`] chip returns once a failed channel switch,
/// scan start or scan restore powered it off.
pub const POWERED_OFF: AxError =
    AxError::Phy("the chip was powered off after a failed channel switch or scan");

/// What a scan, a channel switch or [`scan::run_once`] gets while the last
/// scan has not been ended with [`Live::end_scan`].
pub const SCAN_NOT_ENDED: AxError = AxError::Scan("the last scan has not been ended");

/// The most frames a [`Live`] holds for the next [`Live::drain`], as the
/// receiver's own queue is bounded; past it they are dropped and counted
/// ([`Live::held_dropped`]).
const MAX_HELD: usize = MAX_PENDING;

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
    /// The firmware scan begun, until [`Live::end_scan`] restores after it.
    scan: Option<LiveScan>,
    /// Frames drained outside [`Live::drain`] and [`Live::expire_scan`].
    held: Held,
    /// A failure powered the chip off.
    off: bool,
}

/// A firmware scan on a [`Live`] chip.
struct LiveScan {
    /// The EDCCA levels the scan start raised.
    edcca: EdccaBak,
    /// SCANOFLD start's DONE_ACK plus the list's budget.
    deadline_us: u64,
    /// The first END_SCAN's status.
    status: Option<u8>,
    /// How the scan ended, once what followed END_SCAN, or the stop, was
    /// drained.
    end: Option<ScanEnd>,
}

impl LiveScan {
    /// Record the first END_SCAN of a scan that has not ended.
    fn track(&mut self, event: &Event<'_>) {
        if let Event::C2h(C2h::ScanOfld(report)) = event
            && report.reason == SCAN_END
            && self.status.is_none()
            && self.end.is_none()
        {
            self.status = Some(report.status);
        }
    }
}

/// Frames a scan start or restore drained, oldest first, for the next
/// [`Live::drain`] to hand out. C2Hs other than the scan's own are not
/// held: the ledger counts them.
#[derive(Default)]
struct Held {
    frames: VecDeque<Received>,
    dropped: u16,
}

impl Held {
    fn keep(&mut self, event: Event<'_>) {
        if let Event::Frame(frame) = event {
            if self.frames.len() < MAX_HELD {
                self.frames.push_back(frame);
            } else {
                self.dropped = self.dropped.saturating_add(1);
            }
        }
    }
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
        scan: None,
        held: Held::default(),
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
    /// each [`Live::tune`]'s. A firmware scan takes the radio off it until
    /// [`Live::end_scan`] switches back to it.
    pub fn chan(&self) -> Chan {
        self.chan
    }

    /// Whether a firmware scan was begun and not yet ended with
    /// [`Live::end_scan`].
    pub fn scanning(&self) -> bool {
        self.scan.is_some()
    }

    /// How the firmware scan ended: `None` while it runs, and when none was
    /// begun since the last [`Live::end_scan`].
    pub fn scan_state(&self) -> Option<ScanEnd> {
        self.scan.as_ref().and_then(|scan| scan.end)
    }

    /// The frames a scan start or restore drained past the 64 it
    /// holds for the next [`Live::drain`], which were dropped (saturating).
    pub fn held_dropped(&self) -> u16 {
        self.held.dropped
    }

    /// Refused with [`POWERED_OFF`] on a chip a failure powered off, and
    /// with [`SCAN_NOT_ENDED`] while a scan was begun and not ended.
    pub(super) fn idle(&self) -> Result<(), AxError> {
        if self.off {
            return Err(POWERED_OFF);
        }
        if self.scan.is_some() {
            return Err(SCAN_NOT_ENDED);
        }
        Ok(())
    }

    /// `__rtw89_set_channel` to `chan` ([`channel::set_channel`]): the TX
    /// scheduler stopped, the MAC, BB and RF parts, `set_txpwr` from the
    /// file's tables at [`Live::regds`], then the help's exit; then the
    /// receiver tags frames with `chan`. Refused before any I/O, changing
    /// nothing, on a chip already off ([`POWERED_OFF`]), while a scan has not
    /// been ended ([`SCAN_NOT_ENDED`]: the firmware moves the radio), and for
    /// a channel [`Chan::new20`] would not build. Any later failure leaves
    /// the TX scheduler stopped, so the chip is powered off
    /// ([`bringup::shut_down`]) and every later call fails with
    /// [`POWERED_OFF`].
    ///
    /// # Safety
    /// [`start`]'s contract, still holding for this chip.
    pub unsafe fn tune(&mut self, chan: &Chan) -> Result<(), AxError> {
        self.idle()?;
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
            self.power_off();
            return Err(failed.error);
        }
        self.chan = *chan;
        if let Some(parked) = self.parked.as_mut() {
            parked.rx.set_tag(tag_of(chan));
        }
        Ok(())
    }

    /// Begin a firmware scan of `list` (M3 spec S8.1): `rtw89_hw_scan_start`'s
    /// chip steps ([`scan::hw_scan_start`]), then the scan's receiver
    /// ([`Mode::Scan`], see the module documentation), into which the frames
    /// still waiting for their PPDU status, received before the scan, are
    /// flushed for the next [`Live::drain`]; then `rtw89_hw_scan_offload(true)`
    /// ([`scan::offload`]: ADD_SCANOFLD_CH with `list`, SCANOFLD start, each
    /// with its 100 ms DONE_ACK wait). The scan's budget,
    /// [`scan::scan_budget_us`] of the list, runs from SCANOFLD start's
    /// DONE_ACK ([`Live::expire_scan`]). Frames drained meanwhile are held
    /// for the next [`Live::drain`], END_SCAN recorded.
    ///
    /// Refused before any I/O, changing nothing: on a chip off
    /// ([`POWERED_OFF`]), while the last scan has not been ended
    /// ([`SCAN_NOT_ENDED`]), for a list [`scan::scan_list`] would refuse, and
    /// on a firmware without SCAN_OFFLOAD. A failure in the scan start's own
    /// steps powers the chip off. A failed offload is aborted as Linux aborts
    /// it: [`scan::stop`], its failure ignored, then the restore of
    /// [`Live::end_scan`]; its error is returned, and the chip stays on
    /// unless the restore failed.
    ///
    /// # Safety
    /// [`start`]'s contract, still holding for this chip.
    pub unsafe fn begin_scan(&mut self, list: &[ScanChannel]) -> Result<(), AxError> {
        self.idle()?;
        scan::check_list(list)?;
        scan::check_firmware(self.running.firmware)?;
        let mac = self.running.mac;
        let budget_us = scan::scan_budget_us(list.len());
        let mut held = core::mem::take(&mut self.held);
        let started = self.session(|io, session| {
            // SAFETY: the caller's contract is `start`'s, which
            // `hw_scan_start`'s is, for the live chip's own window and rings.
            let edcca = unsafe { scan::hw_scan_start(io, session, mac, &mut |e| held.keep(e)) }?;
            session.rx.set_mode(Mode::Scan);
            session.rx.flush(&mut |e| held.keep(e));
            let mut scan = LiveScan {
                edcca,
                deadline_us: u64::MAX,
                status: None,
                end: None,
            };
            // SAFETY: as above, `offload`'s.
            let offloaded = unsafe {
                scan::offload(io, session, list, &mut |e| {
                    scan.track(&e);
                    held.keep(e)
                })
            }
            .and_then(|()| io.now_us());
            Ok((scan, offloaded))
        });
        self.held = held;
        let (mut scan, offloaded) = match started {
            Ok(Ok(started)) => started,
            Ok(Err(error)) => {
                self.power_off();
                return Err(error);
            }
            Err(off) => return Err(off),
        };
        match offloaded {
            Ok(now_us) => {
                scan.deadline_us = now_us.saturating_add(budget_us);
                self.scan = Some(scan);
                Ok(())
            }
            Err(error) => {
                self.scan = Some(scan);
                // SAFETY: the caller's contract is `abort`'s.
                unsafe { self.abort() };
                Err(error)
            }
        }
    }

    /// One RXQ poll for `sink` (the RXQ step of the radio's poll): first the
    /// frames a scan start or restore held, oldest first, then every RXQ
    /// entry through the receiver and the session's ledger, as
    /// `rtw89_pci_poll_rxq_dma` hands them to `rtw89_core_rx`. A running
    /// scan's first END_SCAN ends it: the same call then drains what
    /// followed END_SCAN and flushes the receiver into `sink`
    /// ([`scan::tail`]), so every frame the scan heard is handed on before
    /// [`Live::scan_state`] says how it ended. A malformed buffer ends the
    /// drain with its error after the buffers before it were handed on, and
    /// a later call goes on. Refused with [`POWERED_OFF`] on a chip off.
    ///
    /// # Safety
    /// [`start`]'s contract, still holding for this chip.
    pub unsafe fn drain(&mut self, sink: &mut dyn FnMut(Event<'_>)) -> Result<(), AxError> {
        if self.off {
            return Err(POWERED_OFF);
        }
        self.release(sink);
        let mut scan = self.scan.take();
        let drained = self.session(|io, session| {
            // SAFETY: the caller's contract is `start`'s, which `drain`'s
            // is, for the live chip's own window and rings.
            unsafe {
                session.drain(io, &mut |e| {
                    if let Some(scan) = scan.as_mut() {
                        scan.track(&e);
                    }
                    sink(e)
                })
            }?;
            if let Some(scan) = scan.as_mut()
                && scan.end.is_none()
                && let Some(status) = scan.status
            {
                // SAFETY: as above, `tail`'s.
                unsafe { scan::tail(io, session, sink) }?;
                scan.end = Some(scan::ended(status));
            }
            Ok(())
        });
        self.scan = scan;
        drained.and_then(|drained| drained)
    }

    /// The scan budget (the radio poll's last step, M3 spec S8.1): a scan
    /// without END_SCAN at its deadline is stopped as `rtw89_hw_scan_abort`
    /// does ([`scan::stop`]: SCANOFLD stop, then up to `STOP_WAIT_US` for
    /// END_SCAN), what followed drained into `sink` ([`scan::tail`]), after
    /// the frames held, and it ends [`ScanEnd::Stopped`]. Anything else
    /// changes nothing and does no I/O but the clock read: no scan, one that
    /// ended or brought END_SCAN, one within its budget. Refused with
    /// [`POWERED_OFF`] on a chip off.
    ///
    /// # Safety
    /// [`start`]'s contract, still holding for this chip.
    pub unsafe fn expire_scan(&mut self, sink: &mut dyn FnMut(Event<'_>)) -> Result<(), AxError> {
        if self.off {
            return Err(POWERED_OFF);
        }
        let deadline_us = match &self.scan {
            Some(scan) if scan.status.is_none() && scan.end.is_none() => scan.deadline_us,
            _ => return Ok(()),
        };
        if self.io.now_us()? < deadline_us {
            return Ok(());
        }
        self.release(sink);
        let mut scan = self.scan.take();
        let stopped = self.session(|io, session| {
            // SAFETY: the caller's contract is `start`'s, which `stop`'s is,
            // for the live chip's own window and rings.
            let ended = unsafe {
                scan::stop(io, session, &mut |e| {
                    if let Some(scan) = scan.as_mut() {
                        scan.track(&e);
                    }
                    sink(e)
                })
            }?;
            // SAFETY: as above, `tail`'s.
            unsafe { scan::tail(io, session, sink) }?;
            if let Some(scan) = scan.as_mut() {
                scan.end = Some(ScanEnd::Stopped { ended });
            }
            Ok(())
        });
        self.scan = scan;
        stopped.and_then(|stopped| stopped)
    }

    /// `rtw89_hw_scan_complete` for the ended scan (M3 spec S8.3; see the
    /// module documentation): the switch back to [`Live::chan`] through
    /// [`Live::tune`], with its TX power and the receiver's tag, then
    /// [`scan::hw_scan_complete`] (the RX filter, the station receiver
    /// again, the address CAM, EDCCA, TSF sync, DIG), frames its CAM send
    /// drains held for the next [`Live::drain`]. The scan is over either way;
    /// a failure powers the chip off. Refused before any I/O on a chip off
    /// ([`POWERED_OFF`]), without a scan, and while the scan runs.
    ///
    /// # Safety
    /// [`start`]'s contract, still holding for this chip.
    pub unsafe fn end_scan(&mut self) -> Result<(), AxError> {
        if self.off {
            return Err(POWERED_OFF);
        }
        let edcca = match &self.scan {
            None => return Err(AxError::Scan("no scan to end")),
            Some(LiveScan { end: None, .. }) => {
                return Err(AxError::Scan("the scan is still running"));
            }
            Some(scan) => scan.edcca,
        };
        self.scan = None;
        // SAFETY: the caller's contract is `restore`'s.
        unsafe { self.restore(&edcca) }
    }

    /// [`scan::run_once`]'s scan, ended as `end`, with the EDCCA levels its
    /// start raised, for [`Live::end_scan`].
    pub(super) fn scanned(&mut self, edcca: EdccaBak, end: ScanEnd) {
        self.scan = Some(LiveScan {
            edcca,
            deadline_us: 0,
            status: None,
            end: Some(end),
        });
    }

    /// `rtw89_ops_hw_scan`'s abort of a failed offload: [`scan::stop`], whose
    /// failure `rtw89_hw_scan_abort` only warns of (`fw.c:10502-10504`), then
    /// the restore, whose failure powers the chip off and is left for the
    /// next call's [`POWERED_OFF`].
    ///
    /// # Safety
    /// [`start`]'s contract, still holding for this chip.
    unsafe fn abort(&mut self) {
        let mut held = core::mem::take(&mut self.held);
        // SAFETY: the caller's contract is `start`'s, which `stop`'s is, for
        // the live chip's own window and rings.
        let _ =
            self.session(|io, session| unsafe { scan::stop(io, session, &mut |e| held.keep(e)) });
        self.held = held;
        if let Some(scan) = self.scan.take() {
            // SAFETY: as above, `restore`'s.
            let _ = unsafe { self.restore(&scan.edcca) };
        }
    }

    /// [`Live::end_scan`]'s restore, once the scan record is gone: the
    /// switch back, then [`scan::hw_scan_complete`]; a failure of the latter
    /// powers the chip off, as one of the switch does.
    ///
    /// # Safety
    /// [`start`]'s contract, still holding for this chip.
    unsafe fn restore(&mut self, edcca: &EdccaBak) -> Result<(), AxError> {
        let chan = self.chan;
        // SAFETY: the caller's contract is `tune`'s, word for word.
        unsafe { self.tune(&chan) }?;
        let mac = self.running.mac;
        let mut held = core::mem::take(&mut self.held);
        let completed = self.session(|io, session| {
            // SAFETY: as above, `hw_scan_complete`'s, for the live chip's own
            // window and rings.
            unsafe { scan::hw_scan_complete(io, session, mac, edcca, &mut |e| held.keep(e)) }
        });
        self.held = held;
        match completed {
            Ok(Ok(())) => Ok(()),
            Ok(Err(error)) => {
                self.power_off();
                Err(error)
            }
            Err(off) => Err(off),
        }
    }

    /// Hand the held frames to `sink`, oldest first.
    fn release(&mut self, sink: &mut dyn FnMut(Event<'_>)) {
        while let Some(frame) = self.held.frames.pop_front() {
            sink(Event::Frame(frame));
        }
    }

    /// `rtw89_core_stop` after a failure ([`bringup::shut_down`]): every
    /// later call is refused with [`POWERED_OFF`], and a scan is gone with
    /// the firmware.
    fn power_off(&mut self) {
        bringup::shut_down(&mut self.io, &self.rings, self.host);
        self.off = true;
        self.scan = None;
    }

    /// Run `f` on the command session, resumed from where the last one was
    /// parked, then park it again. Refused with [`POWERED_OFF`] on a chip a
    /// failure powered off.
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
    /// Linux ignores them; nothing on a chip a failure already powered
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
