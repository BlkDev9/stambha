// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
// Copyright(c) 2019-2022  Realtek Corporation
// Copyright(c) 2019-2020  Realtek Corporation
//! The first half of an RTL8852C's channel switch, `__rtw89_set_channel`
//! (`core.c:568-598`) on MAC 0 and PHY 0 without DBCC, for a 20 MHz 2.4 or
//! 5 GHz [`Chan`]:
//! - [`help_enter`] is `rtw8852c_set_channel_help(enter)`
//!   (`rtw8852c.c:1960-1974`): `rtw89_mac_stop_sch_tx_v1` (`mac.c:3444-3495`
//!   with `rtw89_set_hw_sch_tx_en_v1`, `:3385-3401`), PPDU status off
//!   (`cmac::cfg_ppdu_status`), `rtw8852c_dfs_en` (`:1942-1948`),
//!   `rtw8852c_tssi_cont_en_phyidx` (`rtw8852c_rfk.c:3789-3820`),
//!   `rtw8852c_adc_en` (`:1950-1958`) and `rtw8852c_bb_reset_en`
//!   (`:1673-1695`);
//! - [`set_channel_mac`] is `rtw8852c_set_channel_mac` (`rtw8852c.c:822-890`);
//! - [`set_channel_rf`] is `rtw8852c_set_channel_rf` (`rtw8852c_rfk.c:
//!   4078-4094`): `_ctrl_ch` (`:3935-3959`) with `_ch_setting`
//!   (`:3899-3933`), `_ctrl_bw` (`:3867-3897`) with `_bw_setting`
//!   (`:3822-3865`), and `_rxbb_bw` (`:3998-4012`) with `_set_rxbb_bw` and
//!   `_set_tia_bw` (`:3961-3996`);
//! - [`help_exit`] is `rtw8852c_set_channel_help(exit)` (`:1975-1982`), with
//!   `rtw89_mac_resume_sch_tx_v1` (`mac.c:3497-3507`).
//!
//! `rtw8852c_set_channel_bb` runs between the MAC and RF parts in Linux and
//! is not here. RF 0x18 (`RR_CFGCH`) is reached twice per path: as 0x18,
//! which `rtw89_phy_write_rf_v1` sends over SWSI (the "DAV" register), and
//! as 0x10018 (`RR_CFGCH_V1`, address bit 16 set), which goes to the
//! path's direct BB window at `rf_base_addr[path] + 0x18 * 4` (MMIO
//! 0x1E060 for A, 0x1F060 for B; the "DDV" register). Each is read and
//! rewritten from its own value ([`super::rf`]).
//!
//! Where Linux ignores a failed CMAC check (`rtw8852c.c:1967-1969`,
//! `:1976`, `:1981`), this fails, as a disabled CMAC is a broken MAC (spec
//! S6.1). The TSSI DE write of `tssi_cont_en(true)` (`_tssi_set_efuse_to_de`)
//! only sets TX power and is not ported (spec S1.6), nor is `_ctrl_bw`'s
//! path-B workaround for the CAV cut (`:3888-3895`, `hal.cv == CHIP_CAV`;
//! this chip is CBV). This module grants no BB channel setup, no TX power,
//! no 6 GHz and no 40, 80 or 160 MHz channel, and transmits nothing.
use super::bb;
use super::chan::{Chan, HwBand};
use super::cmac::{cfg_ppdu_status, check_cmac_en};
use super::elements::RfPath;
use super::io::{AxError, AxIo};
use super::rf::{RFREG_MASK, read_rf, write_rf};

// reg.h (pinned rtw89): MAC registers of band 0.
const R_AX_CTN_DRV_TXEN: u32 = 0xC398;
const B_AX_CTN_TXEN_ALL_MASK_V1: u32 = 0x3_FFFF; // GENMASK(17, 0)
const R_AX_WMAC_RFMOD: u32 = 0xC010;
const B_AX_WMAC_RFMOD_MASK: u8 = 0x3; // GENMASK(1, 0)
const AX_WMAC_RFMOD_20M: u8 = 0;
const R_AX_TX_SUB_CARRIER_VALUE: u32 = 0xC088;
const R_AX_TXRATE_CHK: u32 = 0xC628;
const B_AX_BAND_MODE: u8 = 1 << 4;
const B_AX_RTS_LIMIT_IN_OFDM6: u8 = 1 << 1;
const B_AX_CHECK_CCK_EN: u8 = 1 << 0;

// reg.h: BB registers (offsets from `bb::CR_BASE`).
const R_UPD_P0: u32 = 0x0000;
const B_UPD_P0_EN: u32 = 1 << 31;
const R_ADC_FIFO: u32 = 0x20FC;
const B_ADC_FIFO_RST: u32 = 0xFF00_0000; // GENMASK(31, 24)
const R_RXCCA_V1: u32 = 0x2320;
const B_RXCCA_DIS_V1: u32 = 1 << 0;
const R_PD_CTRL: u32 = 0x0C3C;
const B_PD_HIT_DIS: u32 = 1 << 9;
const R_S0_HW_SI_DIS: u32 = 0x1200;
const R_S1_HW_SI_DIS: u32 = 0x3200;
/// `B_S0_HW_SI_DIS_W_R_TRIG` and `B_S1_...`: GENMASK(30, 28).
const B_HW_SI_DIS_W_R_TRIG: u32 = 0x7000_0000;
const R_RSTB_ASYNC: u32 = 0x0704;
const B_RSTB_ASYNC_ALL: u32 = 1 << 1;
/// `rtw8852c_tssi_cont_en`'s `tssi_trk` (BIT(30)) and `tssi_en` (BIT(31))
/// registers, path A then B.
const TSSI_TRK: [u32; 2] = [0x5818, 0x7818];
const TSSI_EN: [u32; 2] = [0x5820, 0x7820];
const B_TSSI_TRK: u32 = 1 << 30;
const B_TSSI_EN: u32 = 1 << 31;
/// `R_P0_CFCH_BW0` / `B_P0_CFCH_BW0` [27:26] and `R_P0_CFCH_BW1` /
/// `B_P0_CFCH_BW1` [8:5]; path B is `| 1 << 8`.
const R_P0_CFCH_BW0: u32 = 0xC0D4;
const B_P0_CFCH_BW0: u32 = 0x0C00_0000;
const R_P0_CFCH_BW1: u32 = 0xC0D8;
const B_P0_CFCH_BW1: u32 = 0x01E0;

// reg.h: RF registers.
/// `RR_CFGCH` (DAV, over SWSI) and `RR_CFGCH_V1` (DDV, the direct window).
const RR_CFGCH: u32 = 0x18;
const RR_CFGCH_V1: u32 = 0x10018;
const RR_CFGCH_BAND1: u32 = 0x3_0000; // GENMASK(17, 16)
const RR_CFGCH_BW: u32 = 0x0C00; // GENMASK(11, 10)
const RR_CFGCH_BAND0: u32 = 0x0300; // GENMASK(9, 8)
const RR_CFGCH_CH: u32 = 0x00FF; // GENMASK(7, 0)
/// `CFGCH_BAND1_5G` and `CFGCH_BAND0_5G` (1) in their fields; 2G is 0.
const CFGCH_BAND_5G: u32 = (1 << 16) | (1 << 8);
/// `CFGCH_BW_20M` (3) in `RR_CFGCH_BW`.
const CFGCH_BW_20M: u32 = 3 << 10;
const RR_LUTWE2: u32 = 0xEE;
const RR_LUTWE2_RTXBW: u32 = 1 << 2;
const RR_LUTWA: u32 = 0x33;
const RR_LUTWA_M2: u32 = 0x1F; // GENMASK(4, 0)
const RR_LUTWD0: u32 = 0x3F;
const RR_LUTWD0_LB: u32 = 0x3F; // GENMASK(5, 0)
const RR_RXBB2: u32 = 0x8F;
const RR_RXBB2_EBW: u32 = 0x60; // GENMASK(6, 5)

/// `_kpath` without DBCC is `RF_AB` (`rtw8852c_rfk.c:80-92`).
const KPATH: [RfPath; 2] = [RfPath::A, RfPath::B];

/// `rtw89_set_hw_sch_tx_en_v1(MAC_0, tx_en, B_AX_CTN_TXEN_ALL_MASK_V1)`
/// (`mac.c:3385-3401`): the CMAC check, then the TXEN field of 0xC398
/// replaced by `tx_en`'s.
fn set_sch_tx_en<I: AxIo + ?Sized>(io: &mut I, tx_en: u32) -> Result<(), AxError> {
    check_cmac_en(io)?;
    let value = io.read32(R_AX_CTN_DRV_TXEN)?;
    io.write32(
        R_AX_CTN_DRV_TXEN,
        (value & !B_AX_CTN_TXEN_ALL_MASK_V1) | (tx_en & B_AX_CTN_TXEN_ALL_MASK_V1),
    )
}

/// `rtw8852c_tssi_cont_en_phyidx(en, PHY0)` without DBCC: path A then B,
/// `tssi_trk` BIT(30) and `tssi_en` BIT(31) set to `!en`.
fn tssi_cont_en<I: AxIo + ?Sized>(io: &mut I, en: bool) -> Result<(), AxError> {
    let v = u32::from(!en);
    for path in 0..2 {
        bb::field32(io, TSSI_TRK[path], B_TSSI_TRK, v)?;
        bb::field32(io, TSSI_EN[path], B_TSSI_EN, v)?;
        // `_tssi_set_efuse_to_de` would follow here when `en`: TX power only,
        // not ported (spec S1.6).
    }
    Ok(())
}

/// `rtw8852c_set_channel_help(true, ..)` (`rtw8852c.c:1960-1974`): stop the
/// TX scheduler, turn the PPDU status report, DFS, TSSI tracking and the
/// ADC FIFO off, then hold the BB in reset. The result is `tx_en`, the
/// whole of 0xC398 as `rtw89_mac_stop_sch_tx_v1` reads it before the CMAC
/// check, for [`help_exit`]. Linux passes the channel, but nothing in the
/// enter half reads it: `bb_reset_en(false)` disables RX CCA on every band.
pub fn help_enter<I: AxIo + ?Sized>(io: &mut I, _chan: &Chan) -> Result<u32, AxError> {
    // rtw89_mac_stop_sch_tx_v1(MAC_0, SEL_ALL).
    let tx_en = io.read32(R_AX_CTN_DRV_TXEN)?;
    set_sch_tx_en(io, 0)?;
    cfg_ppdu_status(io, false)?;
    // rtw8852c_dfs_en(false).
    bb::field32(io, R_UPD_P0, B_UPD_P0_EN, 0)?;
    tssi_cont_en(io, false)?;
    // rtw8852c_adc_en(false).
    bb::field32(io, R_ADC_FIFO, B_ADC_FIFO_RST, 0xF)?;
    io.delay_us(40);
    // rtw8852c_bb_reset_en(band, PHY0, false) (`:1686-1695`).
    bb::field32(io, R_RXCCA_V1, B_RXCCA_DIS_V1, 1)?;
    bb::field32(io, R_PD_CTRL, B_PD_HIT_DIS, 1)?;
    bb::field32(io, R_S0_HW_SI_DIS, B_HW_SI_DIS_W_R_TRIG, 0x7)?;
    bb::field32(io, R_S1_HW_SI_DIS, B_HW_SI_DIS_W_R_TRIG, 0x7)?;
    io.delay_us(1);
    bb::field32(io, R_RSTB_ASYNC, B_RSTB_ASYNC_ALL, 0)?;
    Ok(tx_en)
}

/// `rtw8852c_set_channel_help(false, ..)` (`rtw8852c.c:1975-1982`): the
/// PPDU status report, ADC FIFO, DFS and TSSI tracking back on, the BB out
/// of reset (RX CCA re-enabled on 2.4 GHz only, as Linux does), then the TX
/// scheduler resumed with `tx_en`'s TXEN field.
pub fn help_exit<I: AxIo + ?Sized>(io: &mut I, chan: &Chan, tx_en: u32) -> Result<(), AxError> {
    cfg_ppdu_status(io, true)?;
    // rtw8852c_adc_en(true).
    bb::field32(io, R_ADC_FIFO, B_ADC_FIFO_RST, 0)?;
    // rtw8852c_dfs_en(true).
    bb::field32(io, R_UPD_P0, B_UPD_P0_EN, 1)?;
    tssi_cont_en(io, true)?;
    // rtw8852c_bb_reset_en(band, PHY0, true) (`:1676-1685`).
    bb::field32(io, R_S0_HW_SI_DIS, B_HW_SI_DIS_W_R_TRIG, 0)?;
    bb::field32(io, R_S1_HW_SI_DIS, B_HW_SI_DIS_W_R_TRIG, 0)?;
    bb::field32(io, R_RSTB_ASYNC, B_RSTB_ASYNC_ALL, 1)?;
    if chan.band == HwBand::G2 {
        bb::field32(io, R_RXCCA_V1, B_RXCCA_DIS_V1, 0)?;
    }
    bb::field32(io, R_PD_CTRL, B_PD_HIT_DIS, 0)?;
    // rtw89_mac_resume_sch_tx_v1(MAC_0, tx_en).
    set_sch_tx_en(io, tx_en)
}

/// `rtw8852c_set_channel_mac(chan, MAC_0)` (`rtw8852c.c:822-890`) at 20 MHz:
/// RFMOD 20M and no TX sub-carrier, then the TX rate checks of the band
/// (2.4 GHz `BAND_MODE`; 5 and 6 GHz `CHECK_CCK_EN | RTS_LIMIT_IN_OFDM6`).
pub fn set_channel_mac<I: AxIo + ?Sized>(io: &mut I, chan: &Chan) -> Result<(), AxError> {
    let chk_rate_mask = match chan.band {
        HwBand::G2 => B_AX_BAND_MODE,
        HwBand::G5 | HwBand::G6 => B_AX_CHECK_CCK_EN | B_AX_RTS_LIMIT_IN_OFDM6,
    };
    super::io::field8(io, R_AX_WMAC_RFMOD, B_AX_WMAC_RFMOD_MASK, AX_WMAC_RFMOD_20M)?;
    io.write32(R_AX_TX_SUB_CARRIER_VALUE, 0)?;
    super::io::clear8(
        io,
        R_AX_TXRATE_CHK,
        B_AX_BAND_MODE | B_AX_CHECK_CCK_EN | B_AX_RTS_LIMIT_IN_OFDM6,
    )?;
    super::io::set8(io, R_AX_TXRATE_CHK, chk_rate_mask)
}

/// Whether `_ctrl_ch` programs `ch` on 2.4 or 5 GHz (`rtw8852c_rfk.c:
/// 3941-3944`); it returns without I/O for any other.
fn ctrl_ch_programs(ch: u8) -> bool {
    !((ch > 14 && ch < 36) || (ch > 64 && ch < 100) || (ch > 144 && ch < 149) || ch > 177)
}

/// `_ch_setting(path, ch, band, is_dav)` (`rtw8852c_rfk.c:3899-3933`): the
/// band and channel fields of RF 0x18 (DAV) or 0x10018 (DDV), read, changed
/// and written whole, then 100 us.
fn ch_setting<I: AxIo + ?Sized>(
    io: &mut I,
    path: RfPath,
    chan: &Chan,
    addr: u32,
) -> Result<(), AxError> {
    let mut rf18 = read_rf(io, path, addr, RFREG_MASK)?;
    rf18 &= !(RR_CFGCH_BAND1 | RR_CFGCH_BAND0 | RR_CFGCH_CH);
    rf18 |= u32::from(chan.channel) & RR_CFGCH_CH;
    if chan.band == HwBand::G5 {
        rf18 |= CFGCH_BAND_5G;
    }
    write_rf(io, path, addr, RFREG_MASK, rf18)?;
    io.delay_us(100);
    Ok(())
}

/// `_bw_setting(path, 20 MHz, is_dav)` (`rtw8852c_rfk.c:3822-3865`): RF 0x18
/// or 0x10018 read, the path's BB CFCH_BW0 = 3 and CFCH_BW1 = 0xF, then the
/// register written back with BW 20M.
fn bw_setting<I: AxIo + ?Sized>(io: &mut I, path: RfPath, addr: u32) -> Result<(), AxError> {
    let mut rf18 = read_rf(io, path, addr, RFREG_MASK)?;
    rf18 = (rf18 & !RR_CFGCH_BW) | CFGCH_BW_20M;
    let shift = (path as u32) << 8;
    bb::field32(io, R_P0_CFCH_BW0 | shift, B_P0_CFCH_BW0, 0x3)?;
    bb::field32(io, R_P0_CFCH_BW1 | shift, B_P0_CFCH_BW1, 0xF)?;
    write_rf(io, path, addr, RFREG_MASK, rf18)
}

/// `rtw8852c_set_channel_rf(chan, PHY0)` (`rtw8852c_rfk.c:4078-4094`), that
/// is `rtw8852c_ctrl_bw_ch` at 20 MHz with `kpath = RF_AB`:
/// 1. `_ctrl_ch`: for path A then B, `_ch_setting` on DAV (0x18, SWSI) then
///    DDV (0x10018, the direct window);
/// 2. `_ctrl_bw`: for path A then B, `_bw_setting` on DAV then DDV;
/// 3. `_rxbb_bw`: for path A then B, the RX baseband LUT (0xEE, 0x33, 0x3F)
///    and the TIA bandwidth (RF 0x8F bits 6:5 = 2).
///
/// From RF 0x18 = 0x01001, 2.4 GHz channel 1 leaves 0x01C01 and 5 GHz
/// channel 36 leaves 0x11D24 on both paths and both windows. A 6 GHz
/// channel is refused before any I/O; its band bits and channel ranges are
/// not ported.
pub fn set_channel_rf<I: AxIo + ?Sized>(io: &mut I, chan: &Chan) -> Result<(), AxError> {
    if chan.band == HwBand::G6 {
        return Err(AxError::Phy("6 GHz channels are not ported"));
    }
    if ctrl_ch_programs(chan.channel) {
        for path in KPATH {
            ch_setting(io, path, chan, RR_CFGCH)?;
            ch_setting(io, path, chan, RR_CFGCH_V1)?;
        }
    }
    for path in KPATH {
        bw_setting(io, path, RR_CFGCH)?;
        bw_setting(io, path, RR_CFGCH_V1)?;
        // The CAV cut's path-B workaround (`:3888-3895`) is not ported.
    }
    for path in KPATH {
        // _set_rxbb_bw(path, 20 MHz).
        write_rf(io, path, RR_LUTWE2, RR_LUTWE2_RTXBW, 0x1)?;
        write_rf(io, path, RR_LUTWA, RR_LUTWA_M2, 0xA)?;
        write_rf(io, path, RR_LUTWD0, RR_LUTWD0_LB, 0x1B)?;
        write_rf(io, path, RR_LUTWE2, RR_LUTWE2_RTXBW, 0x0)?;
        // _set_tia_bw(path, 20 MHz).
        write_rf(io, path, RR_RXBB2, RR_RXBB2_EBW, 0x2)?;
    }
    Ok(())
}

#[cfg(test)]
mod tests;
