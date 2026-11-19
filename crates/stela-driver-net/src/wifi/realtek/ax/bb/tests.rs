// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
// Copyright(c) 2019-2020  Realtek Corporation
use super::*;
use crate::wifi::realtek::ax::tests::model::{Op, RegisterModel};
use alloc::vec::Vec;

#[test]
fn bb_helpers_add_the_cr_base() {
    let mut m = RegisterModel::new();
    field32(&mut m, 0x0704, 0x2, 1).unwrap();
    assert_eq!(m.trace, [Op::R32(0x10704), Op::W32(0x10704, 0x2)]);

    let mut m = RegisterModel::new();
    write32(&mut m, 0x4AD4, 0x40).unwrap();
    assert_eq!(m.trace, [Op::W32(0x14AD4, 0x40)]);
}

#[test]
fn bb_set_clear_and_reads_add_the_cr_base() {
    let mut m = RegisterModel::new();
    m.set32(0x10A10, 0x1234_5678);
    set32(&mut m, 0x0A10, 0x8000_0000).unwrap();
    clear32(&mut m, 0x0A10, 0x0000_0078).unwrap();
    assert_eq!(read32(&mut m, 0x0A10), Ok(0x9234_5600));
    // (0x9234_5600 & 0x00FF_0000) >> 16
    assert_eq!(read32_mask(&mut m, 0x0A10, 0x00FF_0000), Ok(0x34));
    assert_eq!(
        m.trace,
        [
            Op::R32(0x10A10),
            Op::W32(0x10A10, 0x9234_5678),
            Op::R32(0x10A10),
            Op::W32(0x10A10, 0x9234_5600),
            Op::R32(0x10A10),
            Op::R32(0x10A10),
        ]
    );
}

#[test]
fn bb_field32_is_a_read_modify_write_even_for_a_full_mask() {
    // core.h:8224-8235: rtw89_write32_mask always reads first.
    let mut m = RegisterModel::new();
    m.set32(0x10370, 0xFFFF_FFFF);
    field32(&mut m, 0x0370, u32::MAX, 0x1050_0000).unwrap();
    assert_eq!(m.trace, [Op::R32(0x10370), Op::W32(0x10370, 0x1050_0000)]);
}

#[test]
fn bb_read32_mask_of_an_empty_mask_is_zero() {
    let mut m = RegisterModel::new();
    m.set32(0x10000, 0xFFFF_FFFF);
    assert_eq!(read32_mask(&mut m, 0, 0), Ok(0));
}

#[test]
fn bb_offset_past_the_address_space_is_refused_untouched() {
    let mut m = RegisterModel::new();
    let past = u32::MAX - 0xFFFF;
    assert_eq!(
        read32(&mut m, past),
        Err(AxError::OutOfWindow { register: past })
    );
    assert_eq!(
        write32(&mut m, past, 0),
        Err(AxError::OutOfWindow { register: past })
    );
    assert_eq!(m.trace, Vec::new());
}
