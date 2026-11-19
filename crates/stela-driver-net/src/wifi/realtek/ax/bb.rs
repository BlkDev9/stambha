// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
// Copyright(c) 2019-2020  Realtek Corporation
//! Baseband (BB) register access for an AX-generation rtw89 chip, as the
//! `rtw89_phy_read32*`/`rtw89_phy_write32*` accessors (pinned rtw89/phy.h:
//! 712-790) reach it: every BB offset is MMIO `offset + CR_BASE`, with
//! `rtw89_phy_gen_ax.cr_base = 0x10000` (phy.c:9042). The masked helpers
//! are `rtw89_write32_mask`/`_set`/`_clr` and `rtw89_read32_mask`
//! (core.h:8185-8235) over that address; the PHY write goes straight to
//! MMIO, as `rtw89_raw_phy_write32` does (fw.c:12690).
//!
//! Only the accessors are here: this module grants no BB table, reset,
//! calibration or other BB setup.
use super::io::{self, AxError, AxIo};

/// `rtw89_phy_gen_ax.cr_base` (phy.c:9042).
pub const CR_BASE: u32 = 0x10000;

/// The MMIO address of BB `offset`, refused if it would wrap.
fn at(offset: u32) -> Result<u32, AxError> {
    offset
        .checked_add(CR_BASE)
        .ok_or(AxError::OutOfWindow { register: offset })
}

/// `rtw89_phy_read32`.
pub fn read32<I: AxIo + ?Sized>(io: &mut I, offset: u32) -> Result<u32, AxError> {
    io.read32(at(offset)?)
}

/// `rtw89_phy_write32`: a plain write, with no read first.
pub fn write32<I: AxIo + ?Sized>(io: &mut I, offset: u32, v: u32) -> Result<(), AxError> {
    io.write32(at(offset)?, v)
}

/// `rtw89_phy_write32_mask`: a read-modify-write of the `mask` field, even
/// when the mask is all ones, with `v` shifted to the mask's lowest bit.
pub fn field32<I: AxIo + ?Sized>(
    io: &mut I,
    offset: u32,
    mask: u32,
    v: u32,
) -> Result<(), AxError> {
    io::field32(io, at(offset)?, mask, v)
}

/// `rtw89_phy_write32_set`.
pub fn set32<I: AxIo + ?Sized>(io: &mut I, offset: u32, bits: u32) -> Result<(), AxError> {
    io::set32(io, at(offset)?, bits)
}

/// `rtw89_phy_write32_clr`.
pub fn clear32<I: AxIo + ?Sized>(io: &mut I, offset: u32, bits: u32) -> Result<(), AxError> {
    io::clear32(io, at(offset)?, bits)
}

/// `rtw89_phy_read32_mask`: `(v & mask) >> ffs(mask)`. An empty mask, for
/// which Linux's `__ffs` is undefined, reads 0.
pub fn read32_mask<I: AxIo + ?Sized>(io: &mut I, offset: u32, mask: u32) -> Result<u32, AxError> {
    let v = read32(io, offset)?;
    Ok((v & mask).checked_shr(mask.trailing_zeros()).unwrap_or(0))
}

#[cfg(test)]
mod tests;
