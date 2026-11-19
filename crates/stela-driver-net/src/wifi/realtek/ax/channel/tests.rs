// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
// Copyright(c) 2019-2022  Realtek Corporation
// Copyright(c) 2019-2020  Realtek Corporation
use super::*;
use crate::wifi::realtek::ax::chan::Subband;
use crate::wifi::realtek::ax::tests::model::{Op, RegisterModel};
use alloc::{vec, vec::Vec};

/// RF 0x18 as M2b's bring-up is expected to leave it (2G ch 1, 20 MHz
/// bits clear), on both paths, behind SWSI and in the direct window.
const RF18_RESET: u32 = 0x01001;
/// MMIO of RF 0x18 in the direct window: BB `rf_base_addr[path] + 0x18 * 4`
/// (0xE060, 0xF060) plus `CR_BASE`.
const DIRECT_18: [u32; 2] = [0x1E060, 0x1F060];

/// CMAC enabled and RF 0x18 at [`RF18_RESET`] everywhere.
fn model() -> RegisterModel {
    let mut m = RegisterModel::new();
    m.emulate_rf();
    m.set32(0xC000, 0x4000_0000);
    for p in 0..2 {
        m.set_rf(p, 0x18, RF18_RESET);
        m.set32(DIRECT_18[usize::from(p)], RF18_RESET);
    }
    m
}

fn g2(ch: u8) -> Chan {
    Chan::new20(HwBand::G2, ch).unwrap()
}

fn g5(ch: u8) -> Chan {
    Chan::new20(HwBand::G5, ch).unwrap()
}

/// A read-modify-write of `reg` that leaves `v`.
fn rmw(reg: u32, v: u32) -> [Op; 2] {
    [Op::R32(reg), Op::W32(reg, v)]
}

#[test]
fn help_enter_matches_linux_sequence() {
    let mut m = model();
    m.set32(0xC398, 0x3FFFF);
    // As `rtw89_mac_cfg_ppdu_status_bands(true)` and dm init leave them.
    m.set32(0xCE40, 0x2B);
    m.set32(0x10000, 0x8000_0000);
    assert_eq!(help_enter(&mut m, &g2(1)), Ok(0x3FFFF));

    let mut want = vec![
        // rtw89_mac_stop_sch_tx_v1 (mac.c:3444-3495): tx_en, then
        // rtw89_set_hw_sch_tx_en_v1 (mac.c:3385-3401).
        Op::R32(0xC398),
        Op::R32(0xC000),
        Op::R32(0xC398),
        Op::W32(0xC398, 0),
        // cfg_ppdu_status(false).
        Op::R32(0xC000),
        Op::R32(0xCE40),
        Op::W32(0xCE40, 0x2A),
    ];
    // dfs_en(false), tssi_cont_en_phyidx(false), adc_en(false).
    want.extend(rmw(0x10000, 0));
    want.extend(rmw(0x15818, 0x4000_0000));
    want.extend(rmw(0x15820, 0x8000_0000));
    want.extend(rmw(0x17818, 0x4000_0000));
    want.extend(rmw(0x17820, 0x8000_0000));
    want.extend(rmw(0x120FC, 0x0F00_0000));
    want.push(Op::Delay(40));
    // bb_reset_en(false) (rtw8852c.c:1686-1695).
    want.extend(rmw(0x12320, 1));
    want.extend(rmw(0x10C3C, 0x200));
    want.extend(rmw(0x11200, 0x7000_0000));
    want.extend(rmw(0x13200, 0x7000_0000));
    want.push(Op::Delay(1));
    want.extend(rmw(0x10704, 0));
    assert_eq!(m.trace, want);
}

#[test]
fn help_enter_writes_from_zero_in_brief_order() {
    let mut m = model();
    m.set32(0xC398, 0x3FFFF);
    help_enter(&mut m, &g2(1)).unwrap();
    assert_eq!(
        m.writes(),
        [
            Op::W32(0xC398, 0),
            Op::W32(0xCE40, 0),
            Op::W32(0x10000, 0),
            Op::W32(0x15818, 0x4000_0000),
            Op::W32(0x15820, 0x8000_0000),
            Op::W32(0x17818, 0x4000_0000),
            Op::W32(0x17820, 0x8000_0000),
            Op::W32(0x120FC, 0x0F00_0000),
            Op::Delay(40),
            Op::W32(0x12320, 1),
            Op::W32(0x10C3C, 0x200),
            Op::W32(0x11200, 0x7000_0000),
            Op::W32(0x13200, 0x7000_0000),
            Op::Delay(1),
            Op::W32(0x10704, 0),
        ]
    );
}

#[test]
fn help_enter_returns_the_whole_register_and_keeps_its_high_bits() {
    let mut m = model();
    m.set32(0xC398, 0xA005_5555);
    assert_eq!(help_enter(&mut m, &g5(36)), Ok(0xA005_5555));
    // Only TXEN_ALL [17:0] is cleared.
    assert_eq!(m.get32(0xC398), 0xA004_0000);
    // The 5 GHz enter disables RX CCA too: Linux's disable is band-blind.
    assert!(m.trace.contains(&Op::W32(0x12320, 1)));
}

#[test]
fn help_enter_refuses_cmac_off() {
    let mut m = model();
    m.set32(0xC000, 0);
    m.set32(0xC398, 0x3FFFF);
    assert_eq!(
        help_enter(&mut m, &g2(1)),
        Err(AxError::Mac("CMAC function is not enabled"))
    );
    assert!(m.writes().is_empty());
    assert_eq!(m.trace, [Op::R32(0xC398), Op::R32(0xC000)]);
}

/// The registers as `help_enter` leaves them, from the brief's zero start.
fn entered() -> RegisterModel {
    let mut m = model();
    m.set32(0xC398, 0x3FFFF);
    m.set32(0xCE40, 0x2B);
    m.set32(0x10000, 0x8000_0000);
    help_enter(&mut m, &g2(1)).unwrap();
    m.trace.clear();
    m
}

/// `help_exit`'s trace up to `bb_reset_en(true)`'s RSTB write, which both
/// bands share.
fn exit_common() -> Vec<Op> {
    let mut want = vec![
        // cfg_ppdu_status(true).
        Op::R32(0xC000),
        Op::W32(0xCE40, 0x2B),
        Op::R32(0x9C18),
        Op::W32(0x9C18, 1),
    ];
    // adc_en(true), dfs_en(true), tssi_cont_en_phyidx(true) without the DE.
    want.extend(rmw(0x120FC, 0));
    want.extend(rmw(0x10000, 0x8000_0000));
    want.extend(rmw(0x15818, 0));
    want.extend(rmw(0x15820, 0));
    want.extend(rmw(0x17818, 0));
    want.extend(rmw(0x17820, 0));
    // bb_reset_en(true) (rtw8852c.c:1677-1685).
    want.extend(rmw(0x11200, 0));
    want.extend(rmw(0x13200, 0));
    want.extend(rmw(0x10704, 2));
    want
}

/// rtw89_mac_resume_sch_tx_v1 (mac.c:3497-3507).
const RESUME: [Op; 3] = [Op::R32(0xC000), Op::R32(0xC398), Op::W32(0xC398, 0x3FFFF)];

#[test]
fn help_exit_matches_linux_sequence() {
    let mut m = entered();
    assert_eq!(help_exit(&mut m, &g2(1), 0x3FFFF), Ok(()));
    let mut want = exit_common();
    want.extend(rmw(0x12320, 0));
    want.extend(rmw(0x10C3C, 0));
    want.extend(RESUME);
    assert_eq!(m.trace, want);

    // 5 GHz leaves RX CCA disabled, as Linux's 2G-only re-enable does.
    let mut m = entered();
    assert_eq!(help_exit(&mut m, &g5(36), 0x3FFFF), Ok(()));
    let mut want = exit_common();
    want.extend(rmw(0x10C3C, 0));
    want.extend(RESUME);
    assert_eq!(m.trace, want);
    assert_eq!(m.get32(0x12320), 1);
}

#[test]
fn help_exit_resumes_only_the_txen_field() {
    let mut m = entered();
    m.set32(0xC398, 0xA004_0000);
    // tx_en's [17:0] is 0x3_5555; 0xC398's bit 18 and above stay.
    help_exit(&mut m, &g2(1), 0xFFFF_5555).unwrap();
    assert_eq!(m.get32(0xC398), 0xA007_5555);
}

#[test]
fn help_enter_then_exit_restores_the_2g_state() {
    let mut m = model();
    m.set32(0xC398, 0x3FFFF);
    m.set32(0xCE40, 0x2B);
    m.set32(0x10000, 0x8000_0000);
    m.set32(0x10704, 2);
    let before: Vec<u32> = [
        0xC398, 0xCE40, 0x10000, 0x15818, 0x15820, 0x17818, 0x17820, 0x120FC, 0x12320, 0x10C3C,
        0x11200, 0x13200, 0x10704,
    ]
    .iter()
    .map(|&r| m.get32(r))
    .collect();
    let tx_en = help_enter(&mut m, &g2(1)).unwrap();
    help_exit(&mut m, &g2(1), tx_en).unwrap();
    let after: Vec<u32> = [
        0xC398, 0xCE40, 0x10000, 0x15818, 0x15820, 0x17818, 0x17820, 0x120FC, 0x12320, 0x10C3C,
        0x11200, 0x13200, 0x10704,
    ]
    .iter()
    .map(|&r| m.get32(r))
    .collect();
    assert_eq!(before, after);
}

#[test]
fn help_exit_refuses_cmac_off() {
    let mut m = entered();
    m.set32(0xC000, 0);
    assert_eq!(
        help_exit(&mut m, &g2(1), 0x3FFFF),
        Err(AxError::Mac("CMAC function is not enabled"))
    );
    assert!(m.writes().is_empty());
}

#[test]
fn set_channel_mac_by_band() {
    let mut m = model();
    assert_eq!(set_channel_mac(&mut m, &g2(1)), Ok(()));
    assert_eq!(
        m.trace,
        [
            Op::R8(0xC010),
            Op::W8(0xC010, 0),
            Op::W32(0xC088, 0),
            Op::R8(0xC628),
            Op::W8(0xC628, 0),
            Op::R8(0xC628),
            Op::W8(0xC628, 0x10),
        ]
    );

    let mut m = model();
    assert_eq!(set_channel_mac(&mut m, &g5(36)), Ok(()));
    assert_eq!(
        m.trace,
        [
            Op::R8(0xC010),
            Op::W8(0xC010, 0),
            Op::W32(0xC088, 0),
            Op::R8(0xC628),
            Op::W8(0xC628, 0),
            Op::R8(0xC628),
            Op::W8(0xC628, 0x03),
        ]
    );
}

#[test]
fn set_channel_mac_keeps_unrelated_bits() {
    // RFMOD [1:0] and the three rate-check bits are replaced; the rest stays.
    let mut m = model();
    m.set32(0xC010, 0xFF);
    m.set32(0xC088, 0x0ABC);
    m.set32(0xC628, 0xFF);
    set_channel_mac(&mut m, &g5(149)).unwrap();
    assert_eq!(m.get32(0xC010), 0xFC);
    assert_eq!(m.get32(0xC088), 0);
    assert_eq!(m.trace[4], Op::W8(0xC628, 0xEC));
    assert_eq!(m.get32(0xC628), 0xEF);

    let mut m = model();
    m.set32(0xC628, 0xFF);
    set_channel_mac(&mut m, &g2(6)).unwrap();
    assert_eq!(m.get32(0xC628), 0xFC);
}

/// `_ch_setting(path, DAV)` then `(path, DDV)` at the RF level
/// (rtw8852c_rfk.c:3899-3933): the SWSI read with its 2 us settle, the SWSI
/// write, 100 us; then the direct window's write with its 1 us settle
/// (its two reads are not RF ops), 100 us.
fn ch_ops(p: u8, v: u32) -> Vec<Op> {
    vec![
        Op::RfR(p, 0x18),
        Op::Delay(2),
        Op::RfW(p, 0x18, RFREG_MASK, v),
        Op::Delay(100),
        Op::W32(DIRECT_18[usize::from(p)], v),
        Op::Delay(1),
        Op::Delay(100),
    ]
}

/// `_bw_setting(path, 20 MHz, DAV)` then `(path, DDV)` (:3820-3865): the
/// read, CFCH_BW0 [27:26] = 3 and CFCH_BW1 [8:5] = 0xF on the path's BB
/// registers, the write.
fn bw_ops(p: u8, v: u32) -> Vec<Op> {
    let bw0 = 0x1C0D4 | (u32::from(p) << 8);
    let bw1 = 0x1C0D8 | (u32::from(p) << 8);
    vec![
        Op::RfR(p, 0x18),
        Op::Delay(2),
        Op::W32(bw0, 0x0C00_0000),
        Op::W32(bw1, 0x1E0),
        Op::RfW(p, 0x18, RFREG_MASK, v),
        Op::W32(bw0, 0x0C00_0000),
        Op::W32(bw1, 0x1E0),
        Op::W32(DIRECT_18[usize::from(p)], v),
        Op::Delay(1),
    ]
}

/// `_set_rxbb_bw(path, 20 MHz)` and `_set_tia_bw` (:3961-3996).
fn rxbb_ops(p: u8) -> Vec<Op> {
    vec![
        Op::RfW(p, 0xEE, 0x4, 0x4),
        Op::RfW(p, 0x33, 0x1F, 0xA),
        Op::RfW(p, 0x3F, 0x3F, 0x1B),
        Op::RfW(p, 0xEE, 0x4, 0),
        Op::RfW(p, 0x8F, 0x60, 0x40),
    ]
}

fn rf_want(ch_val: u32, bw_val: u32) -> Vec<Op> {
    let mut want = Vec::new();
    for p in 0..2 {
        want.extend(ch_ops(p, ch_val));
    }
    for p in 0..2 {
        want.extend(bw_ops(p, bw_val));
    }
    for p in 0..2 {
        want.extend(rxbb_ops(p));
    }
    want
}

fn assert_rf18_everywhere(m: &RegisterModel, v: u32) {
    for p in 0..2 {
        assert_eq!(m.get_rf(p, 0x18), v, "SWSI RF 0x18 path {p}");
        assert_eq!(
            m.get32(DIRECT_18[usize::from(p)]),
            v,
            "direct RF 0x18 path {p}"
        );
    }
}

#[test]
fn set_channel_rf_2g_ch1_matches_linux_sequence() {
    let mut m = model();
    assert_eq!(set_channel_rf(&mut m, &g2(1)), Ok(()));
    assert_eq!(m.rf_ops(), rf_want(0x01001, 0x01C01));
    assert_rf18_everywhere(&m, 0x01C01);
    // Each direct-window access reads first: two in `_ch_setting`, two in
    // `_bw_setting` (the read, then the write's read-modify-write).
    for reg in DIRECT_18 {
        assert_eq!(m.trace.iter().filter(|op| **op == Op::R32(reg)).count(), 4);
    }
}

#[test]
fn set_channel_rf_5g_ch36_sets_band_bits() {
    let mut m = model();
    assert_eq!(set_channel_rf(&mut m, &g5(36)), Ok(()));
    // BAND1 [17:16] = 1, BAND0 [9:8] = 1, CH [7:0] = 36, then BW [11:10] = 3.
    assert_eq!(m.rf_ops(), rf_want(0x11124, 0x11D24));
    assert_rf18_everywhere(&m, 0x11D24);
}

#[test]
fn set_channel_rf_reads_each_window_on_its_own() {
    // The DAV (SWSI) and DDV (direct) registers are read and rewritten
    // separately, so a path whose two windows disagree keeps each one's own
    // untouched bits [19:18] and [15:12]. A channel field of 0 against 1 (the
    // ambiguous M2b path B readback) does not survive: CH [7:0] is replaced.
    let mut m = model();
    m.set_rf(1, 0x18, 0x01000);
    m.set32(DIRECT_18[1], 0xFFF0_0000 | 0x8_3001);
    set_channel_rf(&mut m, &g2(11)).unwrap();
    assert_eq!(m.get_rf(0, 0x18), 0x01C0B);
    assert_eq!(m.get_rf(1, 0x18), 0x01C0B);
    assert_eq!(m.get32(DIRECT_18[0]), 0x01C0B);
    // The direct window keeps its bits above RFREG_MASK too.
    assert_eq!(m.get32(DIRECT_18[1]), 0xFFF8_3C0B);
}

#[test]
fn set_channel_rf_moves_5g_back_to_2g() {
    let mut m = model();
    set_channel_rf(&mut m, &g5(165)).unwrap();
    assert_rf18_everywhere(&m, 0x11DA5);
    set_channel_rf(&mut m, &g2(1)).unwrap();
    assert_rf18_everywhere(&m, 0x01C01);
}

#[test]
fn set_channel_rf_skips_the_channel_outside_ctrl_ch_ranges() {
    // `_ctrl_ch` returns for a channel it does not program
    // (rtw8852c_rfk.c:3941-3944); `_ctrl_bw` and `_rxbb_bw` still run.
    let mut m = model();
    let chan = Chan {
        channel: 20,
        band: HwBand::G2,
        freq: 0,
        subband: Subband::G2,
    };
    set_channel_rf(&mut m, &chan).unwrap();
    let mut want = Vec::new();
    for p in 0..2 {
        want.extend(bw_ops(p, 0x01C01));
    }
    for p in 0..2 {
        want.extend(rxbb_ops(p));
    }
    assert_eq!(m.rf_ops(), want);
}

#[test]
fn set_channel_rf_refuses_6g_untouched() {
    let mut m = model();
    let chan = Chan {
        channel: 1,
        band: HwBand::G6,
        freq: 5955,
        subband: Subband::G2,
    };
    assert!(matches!(
        set_channel_rf(&mut m, &chan),
        Err(AxError::Phy(_))
    ));
    assert!(m.trace.is_empty());
}
