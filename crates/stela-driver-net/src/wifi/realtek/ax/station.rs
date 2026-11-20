// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
// Copyright(c) 2018-2019 Realtek Corporation
// Copyright(c) 2019-2020  Realtek Corporation
//! An RTL8852CE as the connection keeper's radio (M3 spec S14). rtw89
//! leaves the MAC's soft half to mac80211, as rtw88 does, so the station's
//! orchestration is Stela's, after rtw88's `realtek/station.rs`; the
//! hardware beneath it is [`chip::Chip`].
pub mod chip;
