// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
// Copyright(c) 2019-2020  Realtek Corporation
//! RTL8852CE (rtw89, the "AX" generation) bring-up to running firmware and
//! an initialized MAC: its register access, with masked and polling helpers
//! that mirror rtw89's `rtw89_read*`/`rtw89_write*`,
//! `rtw89_write*_set`/`_clr`/`_mask` and `read_poll_timeout` uses from
//! pinned rtw89/core.h; MAC power on and off; the efuse; the DMAC and PCIe
//! pre-init with its rings; the firmware download; the MAC init of
//! `rtw89_mac_init` (BB/RF function enable, system, DMAC, CMAC band 0 and
//! IMR init, PCIe post-init); and one H2C, OFLD_CFG, whose acknowledgement
//! is read from RXQ by polling. [`bring_up`] puts these in Linux's order
//! before powering the chip off again. The PHY tables the firmware file
//! carries are found ([`elements`]) and can be walked by RFE type and cut
//! ([`phy_table`]), and BB and RF registers can be reached ([`bb`], and
//! [`rf`] through the direct window or SWSI). The BB/RF reset
//! ([`sys::reset_bb_rf`]) and `rtw89_phy_init_bb_reg` ([`bb::init_bb_reg`])
//! are ported, but the bring-up runs neither yet and no radio table is
//! written. The bring-up writes no BB or RF register and grants no calibration,
//! no station and no interrupts: the chip is powered off at the end, and
//! nothing past that one H2C runs.
pub mod bb;
pub mod bringup;
pub mod c2h;
pub mod cmac;
pub mod command;
pub mod dmac;
pub mod efuse;
pub mod elements;
pub mod fwdl;
pub mod h2c;
pub mod io;
pub mod pci;
pub mod phy_table;
pub mod power;
pub mod rf;
pub mod rx;
pub mod sys;
#[cfg(test)]
pub(crate) mod tests;
pub mod trx;

pub use bringup::{BringUpError, Earlier, Host, Running, bring_up, quiesce};
pub use command::H2cAck;
pub use fwdl::download;
pub use io::{
    AxError, AxIo, NativeAxIo, Polled, clear8, clear16, clear32, field8, field16, field32,
    poll_sample, poll_sample_atomic, poll32, poll32_atomic, poll32_value, poll32_value_atomic,
    set8, set16, set32,
};
// `pci::pre_init` and `dmac::pre_init` are reached by their module only.
pub use pci::{Ch12, Rings, encode_txbd};
pub use power::{power_on, xtal_si_read, xtal_si_write};
