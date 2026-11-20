extern crate std;
use super::*;
use crate::wifi::realtek::ax::bringup::tests::{
    BOTH, Chip, DEINIT, GNT_WL_OFF, MAC, NO_LTR, REPORT, RF18, RFK,
    assert_dma_stopped_before_power_off, assert_ends_with_power_off,
    assert_radio_off_before_deinit, chip, cmac_off_after, count, deinit_writes, efuse_with_country,
    efuse_with_mac, first_of, never, same_access, tables_file, tables_file_without, txpwr_alone,
    txpwr_window,
};
use crate::wifi::realtek::ax::chan::{HwBand, Subband};
use crate::wifi::realtek::ax::command::H2cAck;
use crate::wifi::realtek::ax::fwdl::tests::image_bytes;
use crate::wifi::realtek::ax::receive::{Mode, Tag};
use crate::wifi::realtek::ax::regd::Regd;
use crate::wifi::realtek::ax::scan::run_once;
use crate::wifi::realtek::ax::tests::model::{Op, READ_FAILED, RegisterModel};
use crate::wifi::realtek::ax::txpwr_tables::ID_TXPWR_LMT_5G;
use crate::wifi::realtek::beacon::tests::{Domain, Model, Source};
use crate::wifi::realtek::firmware::{Ce, Rtw89};
use alloc::{boxed::Box, sync::Arc, vec::Vec};
use std::sync::Mutex;

type Started = Result<Live<RegisterModel>, StartError<RegisterModel>>;

/// [`start`] on `c`'s chip, cut 2, without LTR, with the elements of `file`
/// and the network country `country`.
fn start_on(c: Chip, file: &[u8], country: Option<[u8; 2]>) -> (Started, Arc<Mutex<Model>>) {
    let Chip { io, m, .. } = c;
    let bytes = image_bytes();
    let fw = Rtw89::parse(&bytes, 2, Ce::Never).unwrap();
    let firmware = FirmwareFile {
        image: &fw.image,
        file,
    };
    // SAFETY: the model domain isolates only the modeled requester behind
    // this test's `RegisterModel`; its pages are plain kept test RAM that no
    // real device can reach, and this test is the only worker.
    let started = unsafe {
        start(
            io,
            &Source,
            Arc::new(Domain(m.clone())),
            2,
            firmware,
            NO_LTR,
            country,
        )
    };
    (started, m)
}

/// A live chip from [`chip`] and [`tables_file`], with no country.
fn live() -> (Live<RegisterModel>, Arc<Mutex<Model>>) {
    let (started, m) = start_on(chip(efuse_with_mac(), 7, never, BOTH), &tables_file(), None);
    (started.unwrap_or_else(|e| panic!("{}", e.error)), m)
}

/// The failure of a start that must fail.
fn refused(started: Started) -> StartError<RegisterModel> {
    match started {
        Ok(live) => panic!("started: {}", live.running()),
        Err(failed) => failed,
    }
}

fn g2(channel: u8) -> Chan {
    Chan::new20(HwBand::G2, channel).unwrap()
}

fn g5(channel: u8) -> Chan {
    Chan::new20(HwBand::G5, channel).unwrap()
}

/// The world regd with no country anywhere: `00`.
const WW: (Regds, [u8; 2]) = (
    Regds {
        g2: Regd::Ww,
        g5: Regd::Ww,
    },
    *b"00",
);

/// The first trace entry after `from` that `want` accepts.
fn after(t: &[Op], from: usize, want: impl Fn(&Op) -> bool) -> usize {
    from + 1
        + t[from + 1..]
            .iter()
            .position(want)
            .unwrap_or_else(|| panic!("nothing after trace entry {from}"))
}

#[test]
fn start_runs_core_channel_txpwr_vif_in_order() {
    let (live, _m) = live();
    let t = &live.io().trace;
    // dm tail: cfg_txrx_path's first write, M2b2's last step.
    let dm_tail = first_of(t, |o| *o == Op::W32(0x14978, 3));
    // core start: cfg_ppdu_status_bands(true).
    let core = after(t, dm_tail, |o| *o == Op::W32(0xCE40, 0x2B));
    // set channel: help_enter's TX scheduler stop.
    let channel = after(t, core, |o| matches!(o, Op::W32(0xC398, _)));
    // tx power: by rate's first word, then path B's SAR ceiling last.
    let txpwr = after(t, channel, |o| matches!(o, Op::W32(0xD2C0, _)));
    let sar = after(t, txpwr, |o| *o == Op::W32(0x176CC, 0x7E));
    // vif init: port 0's beacon interval.
    let vif = after(t, sar, |o| *o == Op::W32(0xC414, 0x64));
    let order = [dm_tail, core, channel, txpwr, sar, vif];
    assert!(order.windows(2).all(|w| w[0] < w[1]), "{order:?}");
    // Ruling R1 (core.c:584-588): set_txpwr's last access comes right
    // before help_exit's first two, its CMAC check and the PPDU status.
    assert_eq!(
        t[sar + 1..sar + 3],
        [Op::R32(0xC000), Op::W32(0xCE40, 0x2B)]
    );
    // No teardown: no WL-off grant, no deinit, no power-off.
    assert!(!t.contains(&Op::W32(0xDA48, GNT_WL_OFF)));
    assert!(!t[dm_tail..].iter().any(|o| matches!(o, Op::W32(0x0074, _))));
    assert_eq!(count(live.io(), Op::W8(0x00AF, 0x80)), 0);
    // What M2b2 reported: no scan yet.
    assert_eq!(
        *live.running(),
        Running {
            cut: 2,
            firmware: (0, 27, 66, 1),
            mac: MAC,
            secure_boot: false,
            mac_init: true,
            ltr: false,
            bacam_reset_timed_out: false,
            h2c: H2cAck {
                rec: true,
                done: true,
                other_c2h: 0,
                unmatched_ack: 0,
                skipped: 0,
                log: None,
            },
            phy: Some(REPORT),
            rfk: Some(RFK),
            rf18: Some(RF18),
            scan: None,
        }
    );
    assert_eq!(live.mac(), MAC);
    assert_eq!(live.regds(), WW);
    assert_eq!(live.chan(), g2(1));
    // Spec S3.1: the BA CAM took seq 4-11 and vif init 12-16.
    assert_eq!(live.h2c(), H2c::at(17));
}

#[test]
fn start_leaves_a_station_receiver_on_its_channel_and_fresh_tx_pools() {
    let (live, m) = live();
    assert_eq!(live.receiver().mode(), Mode::Station);
    assert_eq!(
        live.receiver().tag(),
        Tag {
            band: HwBand::G2,
            channel: 1
        }
    );
    // "tx rings": the pools' own allocation of 174 080 bytes in 43 pages,
    // the last this start mapped, after the rings'.
    let model = m.lock().unwrap();
    let sizes: Vec<usize> = model.mappings.values().map(|(p, _)| p.bytes()).collect();
    assert_eq!(sizes.iter().filter(|&&b| b == 43 * 4096).count(), 1);
    assert_eq!(sizes.last(), Some(&(43 * 4096)), "{sizes:?}");
}

#[test]
fn start_takes_the_efuse_country_then_the_networks() {
    let fcc = Regds {
        g2: Regd::Fcc,
        g5: Regd::Fcc,
    };
    let etsi = Regds {
        g2: Regd::Etsi,
        g5: Regd::Etsi,
    };
    for (efuse, network, want) in [
        (efuse_with_mac(), None, WW),
        (efuse_with_mac(), Some(*b"US"), (fcc, *b"US")),
        // The efuse country wins when the firmware's map has it.
        (efuse_with_country(*b"DE"), Some(*b"US"), (etsi, *b"DE")),
        // One the map lacks falls to the network's.
        (efuse_with_country(*b"JP"), Some(*b"US"), (fcc, *b"US")),
        (efuse_with_country(*b"JP"), None, WW),
    ] {
        let (started, _m) = start_on(chip(efuse, 7, never, BOTH), &tables_file(), network);
        let live = started.unwrap_or_else(|e| panic!("{}", e.error));
        assert_eq!(live.regds(), want, "{network:?}");
    }
}

#[test]
fn start_failure_powers_off_and_hands_back_io() {
    // SWSI stays busy from help_enter's TX scheduler read on, so the RF
    // part of set channel times out.
    let mut c = chip(efuse_with_mac(), 7, never, BOTH);
    c.io.on_read(
        0xC398,
        Box::new(|m: &mut RegisterModel| {
            if m.trace.contains(&Op::W32(0x14978, 3)) {
                m.swsi_status = 0x0300_0000;
            }
            m.get32(0xC398)
        }),
    );
    let (started, _m) = start_on(c, &tables_file(), None);
    let StartError { error, io, rings } = refused(started);
    assert_eq!(error.step, "set channel", "{error}");
    assert!(matches!(error.error, AxError::Timeout { .. }), "{error}");
    // RF 0x18 was read before, so the failure keeps it.
    assert_eq!(error.rf18, Some(RF18));
    // The chip's own window comes back with the rings, torn down as
    // rtw89_core_stop does: the coex half, the deinit, the power-off.
    assert!(rings.is_some());
    assert_radio_off_before_deinit(&io);
    assert_eq!(deinit_writes(&io), DEINIT);
    assert_ends_with_power_off(&io);
    // set_txpwr never ran.
    let dm_tail = first_of(&io.trace, |o| *o == Op::W32(0x14978, 3));
    assert!(
        !io.trace[dm_tail..]
            .iter()
            .any(|o| matches!(o, Op::W32(0xD2C0, _)))
    );
}

#[test]
fn missing_tx_power_element_fails_tx_power() {
    let (started, _m) = start_on(
        chip(efuse_with_mac(), 7, never, BOTH),
        &tables_file_without(Some(ID_TXPWR_LMT_5G)),
        None,
    );
    let StartError { error, io, rings } = refused(started);
    assert_eq!(
        error,
        BringUpError::new(
            "tx power",
            AxError::TxPower("firmware lacks TXPWR_LMT_5GHZ")
        )
    );
    // As rtw89_chip_info_setup fails at rtw89_fw_recognize_elements
    // (core.c:7413): after the efuse, which the RFE type comes from, and
    // before the MAC init, so none of the file's PHY tables reached the
    // chip either; the DMA mac pre-init started is stopped, then the
    // power-off.
    assert!(rings.is_some());
    assert!(io.trace.contains(&Op::W32(0x0030, 0)));
    assert!(!io.trace.contains(&Op::W32(0x14AD4, 0x40)));
    // sys init's dmac_func_en_ax word, the MAC init's first after the efuse.
    assert!(!io.trace.contains(&Op::W32(0x8400, 0xFB7D_4000)));
    assert_dma_stopped_before_power_off(&io);
    assert_ends_with_power_off(&io);
}

#[test]
fn stop_runs_deinit_then_power_off() {
    let (live, m) = live();
    let started = live.io().trace.len();
    // SAFETY: as in `start_on`; the live chip is this test's alone.
    let (io, rings, pools) = unsafe { live.stop() };
    // rtw89_core_stop (core.c:6738-6767): the coex half first, then the
    // deinit, then rtw89_mac_pwr_off.
    assert_eq!(io.trace[started], Op::W32(0xDA48, GNT_WL_OFF));
    assert_radio_off_before_deinit(&io);
    assert_eq!(deinit_writes(&io), DEINIT);
    assert_ends_with_power_off(&io);
    // The rings and pools come back still mapped: nothing proves the DMA
    // stopped.
    assert!(m.lock().unwrap().mappings.len() > 1);
    drop((rings, pools));
}

#[test]
fn tune_sets_channel_then_tx_power_then_help_exit_and_retags() {
    let (mut live, _m) = live();
    let tuned = live.io().trace.len();
    // SAFETY: as in `start_on`.
    unsafe { live.tune(&g5(36)) }.unwrap();
    let t = &live.io().trace[tuned..];
    // __rtw89_set_channel (ruling R1): help_enter's TX scheduler stop, the
    // MAC, BB and RF parts, set_txpwr (103 accesses on 5 GHz, which has no
    // CCK filter), then help_exit, whose TX scheduler resume is the last.
    let enter = first_of(t, |o| matches!(o, Op::W32(0xC398, _)));
    let window = txpwr_window(t, enter);
    assert_eq!(window.len(), 103);
    assert!(same_access(&txpwr_alone(g5(36)), &t[window.clone()]));
    assert!(matches!(t[window.start - 1], Op::RfW(1, 0x8F, 0x60, _)));
    assert_eq!(
        t[window.end..window.end + 2],
        [Op::R32(0xC000), Op::W32(0xCE40, 0x2B)]
    );
    assert!(matches!(t.last(), Some(Op::W32(0xC398, _))));
    assert_eq!(live.chan(), g5(36));
    // The receiver tags what follows with the new channel.
    assert_eq!(
        live.receiver().tag(),
        Tag {
            band: HwBand::G5,
            channel: 36
        }
    );
    assert_eq!(live.receiver().mode(), Mode::Station);
}

#[test]
fn tune_refuses_an_unbuilt_channel_before_any_io() {
    let (mut live, _m) = live();
    let tuned = live.io().trace.len();
    let bad = Chan {
        channel: 6,
        band: HwBand::G2,
        freq: 2412,
        subband: Subband::G2,
    };
    // SAFETY: as in `start_on`.
    let refused = unsafe { live.tune(&bad) };
    assert_eq!(
        refused,
        Err(AxError::Phy("not a 20 MHz 2.4 or 5 GHz channel"))
    );
    assert_eq!(live.io().trace.len(), tuned);
    // The chip stays on: the next channel is set.
    // SAFETY: as above.
    unsafe { live.tune(&g2(6)) }.unwrap();
    assert_eq!(live.receiver().tag().channel, 6);
}

/// After a failed tune: the chip was torn down as a failed start is, and
/// nothing after it reaches the registers.
fn assert_powered_off_for_good(mut live: Live<RegisterModel>) {
    assert_radio_off_before_deinit(live.io());
    assert_eq!(deinit_writes(live.io()), DEINIT);
    assert_ends_with_power_off(live.io());
    let off = live.io().trace.len();
    // SAFETY: as in `start_on`.
    assert_eq!(unsafe { live.tune(&g2(1)) }, Err(POWERED_OFF));
    // No command session either: the scan is refused at its start.
    // SAFETY: as above.
    let scanned = unsafe { run_once(&mut live) };
    assert_eq!(
        scanned,
        Err(ScanError {
            step: "scan start",
            error: POWERED_OFF
        })
    );
    assert_eq!(live.io().trace.len(), off);
    // SAFETY: as above.
    let (io, _rings, _pools) = unsafe { live.stop() };
    assert_eq!(io.trace.len(), off, "a second teardown ran");
}

#[test]
fn tune_failure_in_tx_power_powers_off_and_refuses_later_calls() {
    // set_txpwr's SAR ceiling of path A (BB 0x56CC) cannot be read: the
    // switch stops with the TX scheduler stopped and the BB held in reset.
    let (mut live, _m) = live();
    live.io_mut().fail_reads_of(0x156CC);
    // SAFETY: as in `start_on`.
    assert_eq!(unsafe { live.tune(&g5(36)) }, Err(READ_FAILED));
    let t = &live.io().trace;
    let failed = t.iter().rposition(|o| *o == Op::R32(0x156CC)).unwrap();
    // help_exit never ran: no PPDU status report after the failed read.
    let off = after(t, failed, |o| *o == Op::W32(0xDA48, GNT_WL_OFF));
    assert!(!t[failed..off].contains(&Op::W32(0xCE40, 0x2B)));
    assert_powered_off_for_good(live);
}

#[test]
fn tune_failure_in_the_switch_powers_off_too() {
    // help_enter's CMAC check finds the CMAC off once.
    let (mut live, _m) = live();
    cmac_off_after(live.io_mut(), Op::W32(0x14978, 3));
    // SAFETY: as in `start_on`.
    let failed = unsafe { live.tune(&g2(11)) };
    assert_eq!(failed, Err(AxError::Mac("CMAC function is not enabled")));
    assert_powered_off_for_good(live);
}
