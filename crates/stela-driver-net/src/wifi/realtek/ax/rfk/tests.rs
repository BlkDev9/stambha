// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
// Copyright(c) 2019-2022  Realtek Corporation
use super::*;
use crate::wifi::realtek::ax::tests::model::{Op, RegisterModel};
use alloc::{boxed::Box, vec, vec::Vec};

const F: u32 = 0xFFFFF;

/// A model whose RF 0x05 holds 0x12345 on both paths, whose RF 0x1C has b3
/// (RCK done) set when `done`, and whose RF 0x1B reads `0x3C00 | stored`, so
/// its [14:10] field is 0x0F after the 0x240 trigger.
fn rck_model(done: bool) -> RegisterModel {
    let mut m = RegisterModel::new();
    m.emulate_rf();
    for p in 0..2u8 {
        m.set_rf(p, 0x05, 0x12345);
        m.set_rf(p, 0x1C, if done { 0x8 } else { 0 });
        m.on_rf_read(p, 0x1B, Box::new(move |m| 0x3C00 | m.get_rf(p, 0x1B)));
    }
    m
}

/// Only the decoded SWSI accesses.
fn rf_only(m: &RegisterModel) -> Vec<Op> {
    m.rf_ops()
        .into_iter()
        .filter(|op| matches!(op, Op::RfW(..) | Op::RfR(..)))
        .collect()
}

/// `_rck` for path `p` at the RF level when the first poll sample is done,
/// with each SWSI read's own 2 us settle (phy.c:970).
fn rck_ops(p: u8) -> Vec<Op> {
    vec![
        Op::RfR(p, 0x05),
        Op::Delay(2),
        Op::RfW(p, 0x05, 0x1, 0),
        Op::RfW(p, 0x00, 0xF0000, 0x30000),
        Op::RfR(p, 0x00),
        Op::Delay(2),
        Op::RfW(p, 0x1B, F, 0x240),
        Op::RfR(p, 0x1C),
        Op::Delay(2),
        Op::RfR(p, 0x1B),
        Op::Delay(2),
        Op::RfW(p, 0x1B, F, 0x0F),
        Op::RfW(p, 0x05, F, 0x12345),
        Op::RfR(p, 0x1B),
        Op::Delay(2),
        Op::RfR(p, 0x1C),
        Op::Delay(2),
    ]
}

#[test]
fn rck_matches_linux_sequence() {
    let mut m = rck_model(true);
    assert_eq!(rck(&mut m), Ok(([0x0F, 0x0F], [false, false])));
    let mut want = rck_ops(0);
    want.extend(rck_ops(1));
    assert_eq!(m.rf_ops(), want);
    // The brief's SWSI-only view, path A first.
    let mut swsi = Vec::new();
    for p in 0..2u8 {
        swsi.extend([
            Op::RfR(p, 0x05),
            Op::RfW(p, 0x05, 0x1, 0),
            Op::RfW(p, 0x00, 0xF0000, 0x30000),
            Op::RfR(p, 0x00),
            Op::RfW(p, 0x1B, F, 0x240),
            Op::RfR(p, 0x1C),
            Op::RfR(p, 0x1B),
            Op::RfW(p, 0x1B, F, 0x0F),
            Op::RfW(p, 0x05, F, 0x12345),
            Op::RfR(p, 0x1B),
            Op::RfR(p, 0x1C),
        ]);
    }
    assert_eq!(rf_only(&m), swsi);
    // RF 0x05 is restored whole. (RF 0x1B's unshifted write of the code is
    // pinned by the trace above; the model's read hook stores 0x3C00 | it
    // again on the closing debug read.)
    for p in 0..2u8 {
        assert_eq!(m.get_rf(p, 0x05), 0x12345);
        assert_eq!(m.get_rf(p, 0x1B), 0x3C0F);
        assert_eq!(m.get_rf(p, 0x00) & 0xF0000, 0x30000);
    }
}

#[test]
fn rck_timeout_is_recorded_and_writes_back() {
    let mut m = rck_model(false);
    assert_eq!(rck(&mut m), Ok(([0x0F, 0x0F], [true, true])));
    let ops = rf_only(&m);
    for p in 0..2u8 {
        let trigger = ops
            .iter()
            .position(|op| *op == Op::RfW(p, 0x1B, F, 0x240))
            .unwrap();
        let result = trigger
            + 1
            + ops[trigger + 1..]
                .iter()
                .position(|op| *op == Op::RfR(p, 0x1B))
                .unwrap();
        // read_poll_timeout_atomic(.., 2, 20, ..): 11 samples.
        assert_eq!(result - trigger - 1, 11);
        assert!(
            ops[trigger + 1..result]
                .iter()
                .all(|op| *op == Op::RfR(p, 0x1C))
        );
        // Then the write-back, the restore and the two debug reads anyway.
        assert_eq!(
            ops[result..result + 5],
            [
                Op::RfR(p, 0x1B),
                Op::RfW(p, 0x1B, F, 0x0F),
                Op::RfW(p, 0x05, F, 0x12345),
                Op::RfR(p, 0x1B),
                Op::RfR(p, 0x1C),
            ]
        );
        // 11 poll samples plus the closing debug read.
        assert_eq!(ops.iter().filter(|op| **op == Op::RfR(p, 0x1C)).count(), 12);
        assert_eq!(m.get_rf(p, 0x05), 0x12345);
    }
    // Ten 2 us waits between the samples of each path.
    let polls = m.rf_ops();
    let a = polls
        .iter()
        .position(|op| *op == Op::RfW(0, 0x1B, F, 0x240))
        .unwrap();
    let delays: Vec<u64> = polls[a + 1..]
        .iter()
        .take_while(|op| **op != Op::RfR(0, 0x1B))
        .filter_map(|op| match op {
            Op::Delay(us) => Some(*us),
            _ => None,
        })
        .collect();
    // Each sample's own 2 us SWSI settle, and a 2 us poll wait between them.
    assert_eq!(delays, vec![2; 11 + 10]);
}

#[test]
fn rck_swsi_stuck_fails() {
    let mut m = rck_model(true);
    m.swsi_status = 1 << 24;
    assert!(matches!(
        rck(&mut m),
        Err(AxError::Timeout {
            register: 0x1174C,
            ..
        })
    ));
    assert!(rf_only(&m).is_empty());
}

/// DRCK's MMIO registers: `R_DRCK` 0xC0C4, `R_DRCK_RES` 0xC0C8 and
/// `R_DRCK_FH` 0xC094 behind the BB base.
const DRCK: u32 = 0x1C0C4;
const DRCK_RES: u32 = 0x1C0C8;
const DRCK_FH: u32 = 0x1C094;

#[test]
fn drck_matches_linux_sequence() {
    let mut m = RegisterModel::new();
    m.set32(DRCK_RES, 0x0006_8008);
    assert_eq!(drck(&mut m), Ok((0x0D, false)));
    assert_eq!(
        m.writes(),
        [
            Op::W32(DRCK, 0x40),
            Op::W32(DRCK, 0),
            Op::W32(DRCK_FH, 0x200),
            Op::Delay(1),
            Op::W32(DRCK_FH, 0),
            Op::W32(DRCK, 0),
            Op::W32(DRCK, 0x0D),
        ]
    );
    assert_eq!(
        m.trace,
        [
            Op::R32(DRCK),
            Op::W32(DRCK, 0x40),
            // The poll's one sample.
            Op::R32(DRCK_RES),
            Op::R32(DRCK),
            Op::W32(DRCK, 0),
            Op::R32(DRCK_FH),
            Op::W32(DRCK_FH, 0x200),
            Op::Delay(1),
            Op::R32(DRCK_FH),
            Op::W32(DRCK_FH, 0),
            Op::R32(DRCK_RES),
            Op::R32(DRCK),
            Op::W32(DRCK, 0),
            Op::R32(DRCK),
            Op::W32(DRCK, 0x0D),
            // The debug read of the whole register.
            Op::R32(DRCK),
        ]
    );
}

#[test]
fn drck_keeps_bits_outside_its_fields() {
    let mut m = RegisterModel::new();
    m.set32(DRCK, 0xFFFF_FFFF);
    m.set32(DRCK_FH, 0x8000_0001);
    m.set32(DRCK_RES, 0x000F_8008);
    assert_eq!(drck(&mut m), Ok((0x1F, false)));
    // b6 and b9 cleared, [4:0] the code.
    assert_eq!(m.get32(DRCK), 0xFFFF_FDBF);
    assert_eq!(m.get32(DRCK_FH), 0x8000_0001);
}

#[test]
fn drck_timeout_is_recorded() {
    let mut m = RegisterModel::new();
    m.set32(DRCK_RES, 0x0006_8000);
    assert_eq!(drck(&mut m), Ok((0x0D, true)));
    let samples = m
        .trace
        .iter()
        .take_while(|op| **op != Op::W32(DRCK, 0))
        .filter(|op| **op == Op::R32(DRCK_RES))
        .count();
    // read_poll_timeout_atomic(.., 1, 10000, ..): 9992 samples, 9991 waits.
    assert_eq!(samples, 9992);
    let waits = m
        .trace
        .iter()
        .take_while(|op| **op != Op::W32(DRCK, 0))
        .filter(|op| **op == Op::Delay(1))
        .count();
    assert_eq!(waits, 9991);
    // The rest of the sequence still runs after the enable and the waits.
    let writes = m.writes();
    assert_eq!(writes[0], Op::W32(DRCK, 0x40));
    assert_eq!(
        writes[1 + 9991..],
        [
            Op::W32(DRCK, 0),
            Op::W32(DRCK_FH, 0x200),
            Op::Delay(1),
            Op::W32(DRCK_FH, 0),
            Op::W32(DRCK, 0),
            Op::W32(DRCK, 0x0D),
        ]
    );
    assert_eq!(m.trace.last(), Some(&Op::R32(DRCK)));
}

/// ADDCK's MMIO registers: `R_ADDCK0`/`R_ADDCK1` 0xC0F4/0xC1F4, their
/// results `R_ADDCKR0`/`R_ADDCKR1` 0xC0FC/0xC1FC and reloads
/// `R_ADDCK0_RL`/`R_ADDCK1_RL` 0xC0F8/0xC1F8, behind the BB base.
const ADDCK: [u32; 2] = [0x1C0F4, 0x1C1F4];
const ADDCKR: [u32; 2] = [0x1C0FC, 0x1C1FC];
const ADDCK_RL: [u32; 2] = [0x1C0F8, 0x1C1F8];

/// One path of `_addck` from zero, its one poll sample met.
fn addck_ops(s: usize) -> Vec<Op> {
    let (r, res) = (ADDCK[s], ADDCKR[s]);
    vec![
        Op::R32(r),
        Op::W32(r, 0x4),
        Op::R32(r),
        Op::W32(r, 0x14),
        Op::R32(r),
        Op::W32(r, 0x4),
        Op::Delay(1),
        Op::R32(r),
        Op::W32(r, 0x104),
        Op::R32(res),
        Op::R32(r),
        Op::W32(r, 0x100),
    ]
}

#[test]
fn addck_matches_linux_sequence() {
    let mut m = RegisterModel::new();
    m.set32(ADDCKR[0], 1);
    m.set32(ADDCKR[1], 1);
    assert_eq!(addck(&mut m), Ok([false, false]));
    let mut want = addck_ops(0);
    want.extend(addck_ops(1));
    assert_eq!(m.trace, want);
}

#[test]
fn addck_s1_timeout_is_recorded_on_path_b() {
    let mut m = RegisterModel::new();
    m.set32(ADDCKR[0], 1);
    assert_eq!(addck(&mut m), Ok([false, true]));
    let samples = m
        .trace
        .iter()
        .filter(|op| **op == Op::R32(ADDCKR[1]))
        .count();
    assert_eq!(samples, 9992);
    // S1's reset bit is still cleared after the timeout.
    assert_eq!(m.trace.last(), Some(&Op::W32(ADDCK[1], 0x100)));
}

#[test]
fn addck_s0_timeout_does_not_skip_s1() {
    let mut m = RegisterModel::new();
    m.set32(ADDCKR[1], 1);
    assert_eq!(addck(&mut m), Ok([true, false]));
    assert!(m.trace.ends_with(&addck_ops(1)));
}

#[test]
fn addck_reload_matches_linux_sequence() {
    let mut m = RegisterModel::new();
    m.set32(ADDCKR[0], 0x0008_0601);
    m.set32(ADDCKR[1], 0x0008_0601);
    assert_eq!(addck_reload(&mut m), Ok([(0x201, 0x201); 2]));
    assert_eq!(
        m.writes(),
        [
            Op::W32(ADDCK[0], 0),
            Op::W32(ADDCK[1], 0),
            Op::W32(ADDCK_RL[0], 0x0804_0000),
            Op::W32(ADDCK_RL[0], 0x0806_0100),
            Op::W32(ADDCK_RL[0], 0x3806_0100),
            Op::W32(ADDCK_RL[1], 0x0804_0000),
            Op::W32(ADDCK_RL[1], 0x0806_0100),
            Op::W32(ADDCK_RL[1], 0x3806_0100),
        ]
    );
    // Backup: each path's trigger field cleared before its two result
    // reads.
    assert_eq!(
        m.trace[..10],
        [
            Op::R32(ADDCK[0]),
            Op::W32(ADDCK[0], 0),
            Op::R32(ADDCKR[0]),
            Op::R32(ADDCKR[0]),
            Op::R32(ADDCK[1]),
            Op::W32(ADDCK[1], 0),
            Op::R32(ADDCKR[1]),
            Op::R32(ADDCKR[1]),
            Op::R32(ADDCK_RL[0]),
            Op::W32(ADDCK_RL[0], 0x0804_0000),
        ]
    );
}

#[test]
fn addck_reload_splits_each_path_and_keeps_other_bits() {
    let mut m = RegisterModel::new();
    m.set32(ADDCK[0], 0xFFFF_FFFF);
    m.set32(ADDCKR[0], 0xFFF0_0000 | (0x155 << 10) | 0x2AA);
    m.set32(ADDCKR[1], (0x3FF << 10) | 0x001);
    m.set32(ADDCK_RL[1], 0xC000_00FF);
    assert_eq!(addck_reload(&mut m), Ok([(0x155, 0x2AA), (0x3FF, 0x001)]));
    // [9:8] cleared only.
    assert_eq!(m.get32(ADDCK[0]), 0xFFFF_FCFF);
    assert_eq!(
        m.get32(ADDCK_RL[0]),
        (3 << 28) | (0x155 << 18) | (0x2AA << 8)
    );
    assert_eq!(
        m.get32(ADDCK_RL[1]),
        0xC000_00FF | (3 << 28) | (0x3FF << 18) | (0x001 << 8)
    );
}
