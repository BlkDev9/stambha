// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
// Copyright(c) 2019-2022  Realtek Corporation
//! The analog front-end calibrations an RTL8852C runs first in
//! `rtw8852c_rfk_init` (pinned rtw89/rtw8852c.c:1985-1997), from
//! rtw8852c_rfk.c, with the DRCK table from rtw8852c_rfk_table.c:
//! - [`rck`] is `rtw8852c_rck` (rtw8852c_rfk.c:4133-4139), `_rck`
//!   (:1455-1489) on path A and then B: the RF RC filter calibration;
//! - [`drck`] is `_drck` (:568-586) with `rtw8852c_drck_defs`
//!   (rtw8852c_rfk_table.c:50-55): the DAC reference clock;
//! - [`addck`] is `_addck` (:369-405): the ADC DC offset of S0 and S1;
//! - [`addck_reload`] is `_addck_backup` (:218-233) then `_addck_reload`
//!   (:235-249).
//!
//! BB registers go through [`bb`] (`rtw89_phy_write32_mask` and
//! `rtw89_phy_read32_mask`, every write a read-modify-write) and RF
//! registers through [`rf`]. `rtw89_rfk_parser`'s delay entries are
//! `udelay` (phy.c:8435-8439), and `_addck`'s `fsleep(1)` and `udelay(1)`
//! are both a 1 us busy wait (`fsleep` takes `udelay` up to 10 us,
//! include/linux/delay.h).
//!
//! Every poll is Linux's `read_poll_timeout_atomic`, with its iteration
//! budget ([`poll_sample_atomic`]). Linux only logs a timeout and carries
//! on, so these do too and return it to the caller to record (spec ruling
//! S1.3), each on the path it happened on: Linux records S1's ADDCK timeout
//! in `addck_timeout[0]` (:402), which only a debug dump reads. An RF access
//! SWSI never serves still fails, as [`rf`] says. The register reads
//! Linux makes only for its debug prints are kept (spec ruling S1.4).
//!
//! This module grants no DACK, no coex notify (`rtw8852c_dack` wraps
//! `_dac_cal` in it, and these calls are its parts), no RX DCK, IQK, DPK,
//! TSSI or LCK, no PHY 1 and no channel: it leaves the calibration results
//! in the registers and returns them.
use super::bb;
use super::elements::RfPath;
use super::io::{AxError, AxIo, Polled, poll_sample_atomic};
use super::rf::{self, RFREG_MASK};

/// `RR_MOD` (reg.h:8535) and its `RR_MOD_MASK` [19:16], set to
/// `RR_MOD_V_RX` (reg.h:8539, 8547).
const RR_MOD: u32 = 0x00;
const RR_MOD_MASK: u32 = 0xF_0000;
const RR_MOD_V_RX: u32 = 0x3;
/// `RR_RSV1` and `RR_RSV1_RST` b0 (reg.h:8565-8566).
const RR_RSV1: u32 = 0x05;
const RR_RSV1_RST: u32 = 1 << 0;
/// `RR_RCKC` with its `RR_RCKC_CA` [14:10], and `RR_RCKS` (reg.h:8612-8614).
const RR_RCKC: u32 = 0x1B;
const RR_RCKC_CA: u32 = 0x7C00;
const RR_RCKS: u32 = 0x1C;
/// The RCK trigger `_rck` writes to `RR_RCKC` (rtw8852c_rfk.c:1472).
const RCK_TRIGGER: u32 = 0x00240;
/// The bit `_rck` polls in RF 0x1C (rtw8852c_rfk.c:1474-1475).
const RCK_DONE: u32 = 1 << 3;

/// `R_DRCK_FH` 0xC094 with `B_DRCK_LAT` b9; `R_DRCK` 0xC0C4 with
/// `B_DRCK_IDLE` b9, `B_DRCK_EN` b6 and `B_DRCK_VAL` [4:0]; `R_DRCK_RES`
/// 0xC0C8 with `B_DRCK_RES` [19:15] and `B_DRCK_POL` b3 (reg.h:10367-10376).
const R_DRCK_FH: u32 = 0xC094;
const B_DRCK_LAT: u32 = 1 << 9;
const R_DRCK: u32 = 0xC0C4;
const B_DRCK_IDLE: u32 = 1 << 9;
const B_DRCK_EN: u32 = 1 << 6;
const B_DRCK_VAL: u32 = 0x1F;
const R_DRCK_RES: u32 = 0xC0C8;
const B_DRCK_RES: u32 = 0xF_8000;
const B_DRCK_POL: u32 = 1 << 3;

/// `R_ADDCK0`/`R_ADDCK1` with `B_ADDCK0`/`B_ADDCK1` [9:8], `_EN` b4 and
/// `_RST` b2; `R_ADDCK0_RL`/`R_ADDCK1_RL` with `_RLS` [29:28], `_RL1`
/// [27:18] and `_RL0` [17:8]; `R_ADDCKR0`/`R_ADDCKR1` with `_A0` [19:10] and
/// `_A1` [9:0] (reg.h:10405-10420, 10464-10476). Index 0 is S0, 1 is S1.
const R_ADDCK: [u32; 2] = [0xC0F4, 0xC1F4];
const B_ADDCK: u32 = 0x300;
const B_ADDCK_EN: u32 = 1 << 4;
const B_ADDCK_RST: u32 = 1 << 2;
const R_ADDCK_RL: [u32; 2] = [0xC0F8, 0xC1F8];
const B_ADDCK_RLS: u32 = 0x3000_0000;
const B_ADDCK_RL1: u32 = 0x0FFC_0000;
const B_ADDCK_RL0: u32 = 0x0003_FF00;
const R_ADDCKR: [u32; 2] = [0xC0FC, 0xC1FC];
const B_ADDCKR_A0: u32 = 0xF_FC00;
const B_ADDCKR_A1: u32 = 0x3FF;
/// The done bit `_addck` polls in each result register (:382-383, :398-399).
const ADDCK_DONE: u32 = 1 << 0;

/// The `read_poll_timeout_atomic(.., 1, 10000, false, ..)` of DRCK and
/// ADDCK: at most 9992 samples, 1 us apart.
const DACK_POLL_DELAY_US: u64 = 1;
const DACK_POLL_TIMEOUT_US: u64 = 10_000;

/// `rtw8852c_rck`: `_rck` on path A, then B. Returns each path's RCK code,
/// `RR_RCKC_CA` as read back after the poll, and whether its poll timed out.
pub fn rck<I: AxIo + ?Sized>(io: &mut I) -> Result<([u8; 2], [bool; 2]), AxError> {
    let mut code = [0; 2];
    let mut timed_out = [false; 2];
    for (i, path) in [RfPath::A, RfPath::B].into_iter().enumerate() {
        (code[i], timed_out[i]) = rck_path(io, path)?;
    }
    Ok((code, timed_out))
}

/// `_rck` (rtw8852c_rfk.c:1455-1489) on one path.
fn rck_path<I: AxIo + ?Sized>(io: &mut I, path: RfPath) -> Result<(u8, bool), AxError> {
    let reg5 = rf::read_rf(io, path, RR_RSV1, RFREG_MASK)?;
    rf::write_rf(io, path, RR_RSV1, RR_RSV1_RST, 0)?;
    rf::write_rf(io, path, RR_MOD, RR_MOD_MASK, RR_MOD_V_RX)?;
    // The "[RCK] RF0x00" debug print's read.
    rf::read_rf(io, path, RR_MOD, RFREG_MASK)?;
    rf::write_rf(io, path, RR_RCKC, RFREG_MASK, RCK_TRIGGER)?;
    // read_poll_timeout_atomic(rtw89_read_rf, val, val, 2, 20, false, ..,
    // path, 0x1c, BIT(3)): at most 11 samples.
    let timed_out = matches!(
        poll_sample_atomic(
            io,
            2,
            20,
            false,
            |io| rf::read_rf(io, path, RR_RCKS, RCK_DONE),
            |v| v != 0,
        )?,
        Polled::Expired(_)
    );
    let code = rf::read_rf(io, path, RR_RCKC, RR_RCKC_CA)?;
    // Written back unshifted over the whole register, as Linux does.
    rf::write_rf(io, path, RR_RCKC, RFREG_MASK, code)?;
    rf::write_rf(io, path, RR_RSV1, RFREG_MASK, reg5)?;
    // The "[RCK] RF 0x1b / 0x1c" debug print's two reads. C leaves their
    // order unspecified; this takes them as written.
    rf::read_rf(io, path, RR_RCKC, RFREG_MASK)?;
    rf::read_rf(io, path, RR_RCKS, RFREG_MASK)?;
    // A 5-bit field.
    Ok((code as u8, timed_out))
}

/// A DACK-side `read_poll_timeout_atomic(rtw89_phy_read32_mask, val, val, 1,
/// 10000, false, .., reg, bit)`: whether it timed out.
fn poll_bit<I: AxIo + ?Sized>(io: &mut I, reg: u32, bit: u32) -> Result<bool, AxError> {
    Ok(matches!(
        poll_sample_atomic(
            io,
            DACK_POLL_DELAY_US,
            DACK_POLL_TIMEOUT_US,
            false,
            |io| bb::read32_mask(io, reg, bit),
            |v| v != 0,
        )?,
        Polled::Expired(_)
    ))
}

/// `_drck` (rtw8852c_rfk.c:568-586). Returns the DRCK code `B_DRCK_RES`, as
/// written to `B_DRCK_VAL`, and whether the poll timed out (Linux only
/// prints "[DACK]DRCK timeout").
#[cfg_attr(
    not(test),
    expect(dead_code, reason = "rtw8852c_dack's _dac_cal runs it, ported next")
)]
pub(super) fn drck<I: AxIo + ?Sized>(io: &mut I) -> Result<(u8, bool), AxError> {
    bb::field32(io, R_DRCK, B_DRCK_EN, 1)?;
    let timed_out = poll_bit(io, R_DRCK_RES, B_DRCK_POL)?;
    // rtw8852c_drck_defs (rtw8852c_rfk_table.c:50-55).
    bb::field32(io, R_DRCK, B_DRCK_EN, 0)?;
    bb::field32(io, R_DRCK_FH, B_DRCK_LAT, 1)?;
    io.delay_us(1);
    bb::field32(io, R_DRCK_FH, B_DRCK_LAT, 0)?;
    let code = bb::read32_mask(io, R_DRCK_RES, B_DRCK_RES)?;
    bb::field32(io, R_DRCK, B_DRCK_IDLE, 0)?;
    bb::field32(io, R_DRCK, B_DRCK_VAL, code)?;
    // The "[DACK]0xc0c4" debug print's read.
    bb::read32(io, R_DRCK)?;
    // A 5-bit field.
    Ok((code as u8, timed_out))
}

/// `_addck` (rtw8852c_rfk.c:369-405): S0, then S1. Returns whether each
/// side's poll timed out, S1's in index 1 where Linux writes index 0.
#[cfg_attr(
    not(test),
    expect(dead_code, reason = "rtw8852c_dack's _dac_cal runs it, ported next")
)]
pub(super) fn addck<I: AxIo + ?Sized>(io: &mut I) -> Result<[bool; 2], AxError> {
    let mut timed_out = [false; 2];
    for (s, out) in timed_out.iter_mut().enumerate() {
        let reg = R_ADDCK[s];
        bb::field32(io, reg, B_ADDCK_RST, 1)?;
        bb::field32(io, reg, B_ADDCK_EN, 1)?;
        bb::field32(io, reg, B_ADDCK_EN, 0)?;
        // S0's fsleep(1) and S1's udelay(1): 1 us either way.
        io.delay_us(1);
        bb::field32(io, reg, B_ADDCK, 1)?;
        *out = poll_bit(io, R_ADDCKR[s], ADDCK_DONE)?;
        bb::field32(io, reg, B_ADDCK_RST, 0)?;
    }
    Ok(timed_out)
}

/// `_addck_backup` (rtw8852c_rfk.c:218-233), then `_addck_reload`
/// (:235-249). Returns each side's `(A0 [19:10], A1 [9:0])` result, Linux's
/// `addck_d[s][0]` and `[1]`, as reloaded into `_RL1` and `_RL0`.
#[cfg_attr(
    not(test),
    expect(dead_code, reason = "rtw8852c_dack's _dac_cal runs it, ported next")
)]
pub(super) fn addck_reload<I: AxIo + ?Sized>(io: &mut I) -> Result<[(u16, u16); 2], AxError> {
    let mut addck = [(0, 0); 2];
    for (s, d) in addck.iter_mut().enumerate() {
        bb::field32(io, R_ADDCK[s], B_ADDCK, 0)?;
        // 10-bit fields.
        let a0 = bb::read32_mask(io, R_ADDCKR[s], B_ADDCKR_A0)? as u16;
        let a1 = bb::read32_mask(io, R_ADDCKR[s], B_ADDCKR_A1)? as u16;
        *d = (a0, a1);
    }
    for (s, &(a0, a1)) in addck.iter().enumerate() {
        bb::field32(io, R_ADDCK_RL[s], B_ADDCK_RL1, a0.into())?;
        bb::field32(io, R_ADDCK_RL[s], B_ADDCK_RL0, a1.into())?;
        bb::field32(io, R_ADDCK_RL[s], B_ADDCK_RLS, 0x3)?;
    }
    Ok(addck)
}

#[cfg(test)]
mod tests;
