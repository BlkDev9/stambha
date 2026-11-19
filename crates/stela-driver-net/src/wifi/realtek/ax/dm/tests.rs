// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
// Copyright(c) 2019-2020  Realtek Corporation
// Copyright(c) 2019-2022  Realtek Corporation
use super::*;
use crate::wifi::realtek::ax::tests::model::{Op, RegisterModel};
use alloc::{boxed::Box, vec, vec::Vec};

/// A read-modify-write of BB-or-MAC `reg` that leaves `v`.
fn rmw(reg: u32, v: u32) -> [Op; 2] {
    [Op::R32(reg), Op::W32(reg, v)]
}

/// The A or B half of `rtw8852c_get_thermal` at the RF level: the three
/// trigger writes, the 200 us sleep, then the SWSI read with its own 2 us
/// settle (`rtw89_phy_read_rf_a`, phy.c:970).
fn thermal_ops(path: u8) -> Vec<Op> {
    vec![
        Op::RfW(path, 0x42, 0x80000, 0x80000),
        Op::RfW(path, 0x42, 0x80000, 0),
        Op::RfW(path, 0x42, 0x80000, 0x80000),
        Op::Delay(200),
        Op::RfR(path, 0x42),
        Op::Delay(2),
    ]
}

fn rf_model(a: u32, b: u32) -> RegisterModel {
    let mut m = RegisterModel::new();
    m.emulate_rf();
    m.set_rf(0, 0x42, a);
    m.set_rf(1, 0x42, b);
    m
}

#[test]
fn stat_init_reads_both_thermals() {
    let mut m = rf_model(0x3C, 0x3C);
    assert_eq!(stat_init(&mut m, Some([0x11, 0x10])), Ok([0x16, 0x26]));
    let mut want = thermal_ops(0);
    want.extend(thermal_ops(1));
    assert_eq!(m.rf_ops(), want);

    let mut m = rf_model(0x3C, 0x3C);
    assert_eq!(stat_init(&mut m, None), Ok([0x1E, 0x1E]));
    assert_eq!(m.rf_ops(), want);
}

#[test]
fn stat_init_compensates_an_unprogrammed_path_as_linux_does() {
    // `pg_thermal_trim` holds when either byte is programmed; a 0xFF path
    // then has VAL8 and SIGN set, so Linux takes 8 off it (rtw8852c.c:
    // 2599-2604).
    let mut m = rf_model(0x3C, 0x3C);
    assert_eq!(stat_init(&mut m, Some([0xFF, 0x10])), Ok([0x16, 0x26]));
    // A trim byte without VAL8 adds nothing.
    let mut m = rf_model(0x3C, 0x3C);
    assert_eq!(stat_init(&mut m, Some([0x0F, 0x01])), Ok([0x1E, 0x1E]));
    // `val + comp` is returned as a u8, so it wraps below zero.
    let mut m = rf_model(0, 0x7E);
    assert_eq!(stat_init(&mut m, Some([0x11, 0x10])), Ok([0xF8, 0x47]));
}

#[test]
fn stat_init_swsi_stuck_fails_without_writing() {
    let mut m = rf_model(0x3C, 0x3C);
    m.swsi_status = 1 << 24;
    assert!(matches!(
        stat_init(&mut m, None),
        Err(AxError::Timeout {
            register: 0x1174C,
            ..
        })
    ));
    assert!(m.rf_ops().iter().all(|op| !matches!(op, Op::RfW(..))));
}

/// `rtw8852c_bb_gpio_init`'s per-path head on BB `base` (0x5800 or
/// 0x7800), from zero.
fn gpio_head(base: u32) -> Vec<Op> {
    let mut ops = Vec::new();
    ops.extend(rmw(base | 0x68, 2));
    ops.extend(rmw(base | 0x68, 2));
    ops.extend(rmw(base | 0x68, 2));
    ops.push(Op::W32(base | 0x80, 0x7777_7777));
    ops.push(Op::W32(base | 0x84, 0x7777_7777));
    ops
}

const TRSW: [u32; 8] = [
    0x0001_0002,
    0x0009_0002,
    0x0029_0002,
    0x00A9_0002,
    0x01A9_0002,
    0x09A9_0002,
    0x29A9_0002,
    0xA9A9_0002,
];
const RFM: [u32; 3] = [0, 0x0400, 0x0008_0400];

#[test]
fn bb_sethw_matches_linux_sequence() {
    let mut m = RegisterModel::new();
    m.set32(0x14DF0, 0x0808_0618);
    assert_eq!(bb_sethw(&mut m), Ok([0x18, 0]));

    let mut want = Vec::new();
    want.extend(rmw(0x10A10, 1));
    want.extend(rmw(0x12A10, 1));
    // `rtw89_mac_txpwr_write32`: plain MAC writes, no read.
    for i in 0..128 {
        want.push(Op::W32(0xD36C + 4 * i, 0));
    }
    want.extend(gpio_head(0x15800));
    want.extend(gpio_head(0x17800));
    want.push(Op::W32(0x10334, 0xFFFF_FFFF));
    want.push(Op::W32(0x10338, 0));
    want.push(Op::W32(0x1033C, 0));
    want.push(Op::W32(0x10340, 0));
    for reg in [0x15868, 0x17868] {
        for v in TRSW {
            want.extend(rmw(reg, v));
        }
    }
    for reg in [0x15894, 0x17894] {
        for v in RFM {
            want.extend(rmw(reg, v));
        }
    }
    want.push(Op::R32(0x14DF0));
    want.push(Op::R32(0x16DF0));
    assert_eq!(m.trace, want);
    assert_eq!(*m.trace.last().unwrap(), Op::R32(0x16DF0));
}

#[test]
fn bb_sethw_offset_base_is_signed() {
    // `offset_base` is an s8 (core.h:7300): [7:0] of 0x4DF0 and 0x6DF0.
    let mut m = RegisterModel::new();
    m.set32(0x14DF0, 0xFFFF_FF80);
    m.set32(0x16DF0, 0x0000_01F8);
    assert_eq!(bb_sethw(&mut m), Ok([-128, -8]));
}

#[test]
fn bb_sethw_keeps_the_bits_outside_each_field() {
    let mut m = RegisterModel::new();
    for reg in [0x15868, 0x17868] {
        m.set32(reg, 0xFFFF_FFFF);
    }
    for reg in [0x15894, 0x17894] {
        m.set32(reg, 0xFFFF_FFFF);
    }
    bb_sethw(&mut m).unwrap();
    // TRSW_A set, TRSW_X and SO_A2 [7:5] cleared, [31:16] = 0xA9A9.
    assert_eq!(m.get32(0x15868), 0xA9A9_FF1B);
    assert_eq!(m.get32(0x17868), 0xA9A9_FF1B);
    // [23:0] = 0x080400, [31:24] kept.
    assert_eq!(m.get32(0x15894), 0xFF08_0400);
    assert_eq!(m.get32(0x17894), 0xFF08_0400);
}

#[test]
fn env_monitor_init_matches_linux_sequence() {
    let mut m = RegisterModel::new();
    env_monitor_init(&mut m).unwrap();
    let want: Vec<Op> = [
        (0x10C00, 1),
        (0x10C00, 3),
        (0x10C00, 7),
        (0x10C00, 7),
        (0x10C2C, 0),
        (0x10C30, 3),
        (0x10C34, 9),
        (0x10C38, 0x21),
        (0x10C2C, 0x0002_0000),
        (0x10C30, 0x0008_0003),
        (0x10C34, 0x0020_0009),
        (0x10C38, 0x0080_0021),
        (0x10C28, 0x1000),
        (0x10C2C, 0x0002_8000),
        (0x10C30, 0x0008_8003),
        (0x10C34, 0x0020_8009),
        (0x10C38, 0x0080_8021),
    ]
    .into_iter()
    .flat_map(|(reg, v)| rmw(reg, v))
    .collect();
    assert_eq!(m.trace, want);
}

#[test]
fn physts_parsing_init_rewrites_each_page() {
    let mut m = RegisterModel::new();
    let pages: Vec<u32> = (0..15).map(|p| 0x1073C + 4 * p).collect();
    for &reg in &pages {
        m.set32(reg, 0xF0);
    }
    physts_parsing_init(&mut m).unwrap();

    let mut want = Vec::new();
    want.extend(rmw(0x10738, 0x8));
    want.extend(rmw(0x10738, 0xC));
    // By bitmap index; index 9 is reserved and index 16 (EHT) is BE only.
    let values: [(u32, u32); 15] = [
        (0, 0xF0),
        (1, 0xF0),
        (2, 0xF0),
        (3, 0xF0),
        (4, 0xF0),
        (5, 0xF0),
        (6, 0x20F0),
        (7, 0x20F0),
        (8, 0xF0),
        (10, 0x20F2),
        (11, 0x2),
        (12, 0),
        (13, 0x0010_0000),
        (14, 0x0010_0000),
        (15, 0x0010_0000),
    ];
    for (i, v) in values {
        let page = if i > 9 { i - 1 } else { i };
        let reg = 0x1073C + 4 * page;
        want.push(Op::R32(reg));
        want.extend(rmw(reg, v));
    }
    assert_eq!(m.trace, want);
    // Page 9's index skipped: 0x10760 (index 10) follows 0x1075C (index 8).
    let at = |reg| m.trace.iter().position(|o| *o == Op::R32(reg)).unwrap();
    assert!(at(0x1075C) < at(0x10760));
    assert!(!m.trace.contains(&Op::R32(0x10778)));
}

#[test]
fn dig_init_matches_linux_sequence() {
    const REGS: [u32; 8] = [
        0x1481C, 0x1481C, 0x14B74, 0x14B64, 0x14C24, 0x14C28, 0x14CE8, 0x14CEC,
    ];
    let mut m = RegisterModel::new();
    dig_init(&mut m).unwrap();
    let from_zero = [0, 0, 0, 0x8000_0000, 0, 0, 0, 0];
    let want: Vec<Op> = REGS
        .iter()
        .zip(from_zero)
        .flat_map(|(&reg, v)| rmw(reg, v))
        .collect();
    assert_eq!(m.trace, want);

    let mut m = RegisterModel::new();
    for reg in REGS {
        m.set32(reg, 0xFFFF_FFFF);
    }
    dig_init(&mut m).unwrap();
    let from_ones = [
        0xFFFF_F83F,
        0xDFFF_F83F,
        0xBFFF_FFFF,
        0x80FF_FFFF,
        0xFFFF_FFDF,
        0xFFFF_FFDF,
        0xFFFF_FFDF,
        0xFFFF_FFDF,
    ];
    let want: Vec<Op> = REGS
        .iter()
        .zip(from_ones)
        .flat_map(|(&reg, v)| rmw(reg, v))
        .collect();
    assert_eq!(m.trace, want);
}

/// An XTAL SI controller behind 0x0270: a command (bit 31 set) is carried
/// out at the first read, which then reports it done; a read command
/// returns `read(offset, stored)` in [15:8]. A write stores under its mask.
fn xtal_model(read: fn(u8, u8) -> u8) -> RegisterModel {
    let mut m = RegisterModel::new();
    let mut file = [0u8; 256];
    m.on_read(
        0x0270,
        Box::new(move |m: &mut RegisterModel| {
            let cmd = m.get32(0x0270);
            if cmd & 0x8000_0000 == 0 {
                return cmd;
            }
            let offset = cmd as u8;
            let (value, mask) = ((cmd >> 8) as u8, (cmd >> 16) as u8);
            match (cmd >> 24) & 0x3 {
                0 => {
                    let old = file[usize::from(offset)];
                    file[usize::from(offset)] = (old & !mask) | (value & mask);
                    cmd & 0x7FFF_FFFF
                }
                _ => {
                    let data = read(offset, file[usize::from(offset)]);
                    (cmd & 0x7FFF_00FF) | u32::from(data) << 8
                }
            }
        }),
    );
    m
}

#[test]
fn cfo_init_writes_the_crystal_cap() {
    let mut m = xtal_model(|_, stored| stored);
    m.set32(0xD248, 0x0776_3333);
    // xtal_k 0xAF: B_AX_XTAL_SC_MASK keeps cap 0x2F.
    assert_eq!(cfo_init(&mut m, 0xAF), Ok((0x2F, 0x2F)));
    let writes: Vec<Op> = m
        .trace
        .iter()
        .copied()
        .filter(|o| matches!(o, Op::W32(..)))
        .collect();
    assert_eq!(
        writes,
        [
            Op::W32(0x0270, 0x80FF_2F05),
            Op::W32(0x0270, 0x80FF_2F04),
            Op::W32(0x0270, 0x8100_0005),
            Op::W32(0x0270, 0x8100_0004),
            Op::W32(0x14494, 0x2000_0000),
            Op::W32(0x14490, 0x0800_0000),
            Op::W32(0x14264, 1),
            Op::W32(0xD248, 0x0776_3330),
        ]
    );
    // dcfo_comp_init is read-modify-write throughout.
    let tail = &m.trace[m.trace.len() - 8..];
    let mut want = Vec::new();
    want.extend(rmw(0x14494, 0x2000_0000));
    want.extend(rmw(0x14490, 0x0800_0000));
    want.extend(rmw(0x14264, 1));
    want.extend(rmw(0xD248, 0x0776_3330));
    assert_eq!(tail, want.as_slice());
}

#[test]
fn cfo_init_returns_xo_then_xi() {
    // XO is offset 0x05 and XI 0x04 (mac.h:1678-1681): make them differ.
    let mut m = xtal_model(|offset, stored| if offset == 0x05 { 0x41 } else { stored });
    assert_eq!(cfo_init(&mut m, 0x2F), Ok((0x41, 0x2F)));
    // An unprogrammed 0xFF crystal cap is masked, never defaulted.
    let mut m = xtal_model(|_, stored| stored);
    assert_eq!(cfo_init(&mut m, 0xFF), Ok((0x7F, 0x7F)));
    assert!(m.trace.contains(&Op::W32(0x0270, 0x80FF_7F05)));
}

#[test]
fn cfo_init_xtal_timeout_fails() {
    // 0x0270 bit 31 never clears.
    let mut m = RegisterModel::new();
    assert!(matches!(
        cfo_init(&mut m, 0xAF),
        Err(AxError::Timeout {
            register: 0x270,
            ..
        })
    ));
    // Nothing past the first command was issued.
    let writes: Vec<Op> = m
        .trace
        .iter()
        .copied()
        .filter(|o| matches!(o, Op::W32(..)))
        .collect();
    assert_eq!(writes, [Op::W32(0x0270, 0x80FF_2F05)]);
}

#[test]
fn edcca_init_sets_t2r_collision() {
    let mut m = RegisterModel::new();
    edcca_init(&mut m).unwrap();
    assert_eq!(m.trace, rmw(0x10C70, 0x0290_0000));

    let mut m = RegisterModel::new();
    m.set32(0x10C70, 0xFFFF_FFFF);
    edcca_init(&mut m).unwrap();
    assert_eq!(m.get32(0x10C70), 0xFE9F_FFFF);
}

#[test]
fn cck_pd_lower_bound_follows_linux_arithmetic() {
    // The first start: min(22, 0) - 23 = -23, raised to -18, less 110.
    assert_eq!(cck_pd_lower_bound(22, 0), (-128i32) as u32);
    // A linked RSSI above the floor: min(80, 60) - 23 = 37, less 110.
    assert_eq!(cck_pd_lower_bound(80, 60), (-73i32) as u32);
}
