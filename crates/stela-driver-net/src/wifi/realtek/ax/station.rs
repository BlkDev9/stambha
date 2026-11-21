// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
// Copyright(c) 2018-2019 Realtek Corporation
// Copyright(c) 2019-2020  Realtek Corporation
//! An RTL8852CE as the connection keeper's radio (M3 spec S14). rtw89
//! leaves the MAC's soft half to mac80211, as rtw88 does, so the station's
//! orchestration is Stela's, after rtw88's `realtek/station.rs` (its
//! `Rtl8822c`, cited per method); the hardware beneath it is
//! [`chip::Chip`], which a live chip implements.
//!
//! - A scan is one firmware offload (`rtw89_ops_hw_scan`, spec S8) of every
//!   channel the country leaves, each passive (no probe templates, spec
//!   S1.3); its end restores the chip (`rtw89_hw_scan_complete`) before
//!   the keeper retires it.
//! - A join tunes to the BSS and sets its BSSID (`ieee80211_prep_connection`
//!   over `rtw89_set_channel` and `BSS_CHANGED_BSSID`), then authenticates
//!   and associates through the shared `Station`, whose frames leave on the
//!   chip's management queue at the BSS's lowest basic rate, each ticketed
//!   one's TX status from its release report. The AP's acceptance of the
//!   authentication moves its station to AUTH ([`chip::Chip::authenticated`]).
//! - The association programs the chip with the negotiated peer as
//!   `BSS_CHANGED_ASSOC` does (spec S10), on the channel the join switched
//!   to.
//! - EAPOL runs over the data queues; data frames wait in `held` while the
//!   chip has no room, management frames in the outbox.
//! - The handshake's keys go to the chip's security CAM (`rtw89_ops_set_key`,
//!   spec S12), CCMP-128 only (spec S1.2): the PTK with a host packet
//!   number, the GTK receive-only; a key of an index already held replaces
//!   it as `ieee80211_key_replace` does, the old one deleted first, and an
//!   identical one is not installed again (`ieee80211_key_identical`). The
//!   port opens on both, and under PMF on the IGTK too, which stays on the
//!   host (`BipKey`); a rekey while it is open authorizes it again on the
//!   keys then held, as mac80211's station stays authorized across one.
//! - Data under the PTK leaves as `Protection::HostIv` plans it: the
//!   station reserves its PN and writes its CCMP header, which
//!   [`super::txd::strip_ccmp`] checks against the reservation and removes;
//!   the chip puts the PN in the TX descriptor and the hardware encrypts
//!   (`hw_sec_hdr`, spec S5.4). A reservation is spent whether the frame
//!   leaves, waits or is dropped, so PNs only increase. Robust management
//!   frames are sealed on the host under the PTK (`realtek::ccmp`), as
//!   rtw88 seals them, and leave without a hardware key.
//! - Received data the hardware decrypted (its CCMP header and MIC kept,
//!   spec S7.3) is taken under the key its key ID names, through the
//!   station's replay and port checks, fragments reassembled and A-MSDUs
//!   split as rtw88 takes them; protected management frames are opened on
//!   the host when the hardware left them sealed.
//! - A departure leaves through the chip (spec S11), which releases its
//!   channel. From an association, mac80211's `ieee80211_set_disassoc`
//!   flushes first ([`chip::Chip::flush`] with drop, then without it after
//!   a deauthentication of the station's own); each key is deleted after a
//!   flush without drop, as `rtw89_ops_set_key`'s DISABLE_KEY flushes, the
//!   PTK first, then the group keys, before the chip disassociates.
//!
//! The radio polls (spec S1.4): it never signals, and the keeper services
//! it from `carrier()` and `receive()`. Its diagnostic is the progress line
//! of spec S14.6, which names no address, then the chip's own counts.
//!
//! This module does not grant: any cipher but CCMP-128 (TKIP, GCMP,
//! CCMP-256, WEP), hardware-encrypted management frames
//! (`hw_mgmt_tx_encrypt`), SAE, RX Block Ack and reordering, beacon loss,
//! A-MSDU or aggregated TX, power save, templates, the watchdog, SA Query
//! requests of its own, or following the AP's beacons and channel
//! switches. The GTK is deleted before the chip disassociates, where Linux
//! deletes a station's group key (a vif key in station mode, cfg.c's
//! `ieee80211_add_key` without an address) only after cfg80211's
//! disconnect (`__cfg80211_disconnected`'s `rdev_del_key` loop,
//! `net/wireless/sme.c:1375-1391`); either way the chip ends keyless.
pub mod chip;

use self::chip::{Association, Chip, Data, HardwareKey, Incoming, Management, Received, Token};
use super::assoc::HtPeer;
use super::txd;
use crate::{
    Clock, Link, LinkError, Mac,
    wifi::{
        Band, Channel, RadioStatus, RawRate, Timestamp,
        connection::keeper::{self, Advertisement, Keeper, Peer, Radio, Stage},
        frame::Mpdu,
        network::Network,
        realtek::ccmp,
        regulatory::{channel::DISABLED, database::Country},
        scan::{self, Outcome, Retirement, Scanner, result::Bss},
        security::{
            self, Cipher, InstallOutcome, Integrity, KeyIdentity, KeyScope, KeySlot, PacketNumber,
            ReplaySeed, SecurityStatus,
            aggregate::{OrderedGroup, Shape},
            management::{self as protection, BipKey, ManagementCipher},
            tx::TransmitSeed,
        },
        station::{
            Action, AssociationContext, Completion, Failure, Join, NativeBssObservation,
            Phase as Peering, RegulatoryPolicy, Station, Ticket,
            data::{self, Ingress, Protection, Qos, SequenceOwner, TxClass},
            external,
            fragments::Fragments,
            multi_link::Affiliation,
            phy::{Capabilities, McsMaps, Role},
            wire::six_ghz::Selection,
        },
    },
};
use alloc::{collections::VecDeque, format, string::String, sync::Arc, vec::Vec};
use core::fmt;
use stela_driver_base::entropy::Entropy;
use zeroize::Zeroizing;

/// How long the radio may take from its start to carrier (M3 spec S1.5):
/// one scan's budget, `SAE_BUDGET_US` and `HANDSHAKE_TIMEOUT_US`, 28.98 s,
/// rounded up.
pub const WIFI_CARRIER_WAIT_US: u64 = 30_000_000;
/// `IEEE80211_SCAN_RESULT_EXPIRE`.
const BSS_EXPIRY_US: u64 = 30_000_000;
/// The data frames that wait for room, the oldest dropped past it.
const MAX_HELD: usize = 256;
/// The EAPOL frames, and the ticketed management frames the station no
/// longer awaits, whose release reports the progress line still counts.
const MAX_EAPOL_REPORTS: usize = 64;
/// What a key the chip cannot take gets (spec S1.2).
const CCMP_128_ONLY: &str = "a key other than CCMP-128 on an RTL8852C";
/// `IEEE80211_CCMP_HDR_LEN`, and a management frame's header, before which
/// a sealed frame's CCMP header goes.
const CCMP_HEADER: usize = 8;
const MANAGEMENT_HEADER: usize = 24;
/// What SAE gets in M3a (M3 Task 17).
const SAE_BEFORE_M3B: &str = "SAE before M3b";
/// `ieee80211_add_wmm_info_ie`, without U-APSD.
const WMM_ELEMENT: [u8; 9] = [221, 7, 0, 0x50, 0xf2, 2, 0, 1, 0];
/// `rtw89_init_ht_cap` (`core.c:6063-6090`) at 20 MHz with two streams
/// (spec S1.7): capability info 0x09AD (Linux's 0x19E3 without 20/40,
/// SGI 40 and DSSS/CCK 40, SM power save disabled), A-MPDU 64K without
/// density, MCS 0-15 without MCS 32, 144 Mb/s, TX MCS defined.
const HT_ELEMENT: [u8; 28] = [
    45, 26, 0xAD, 0x09, 0x03, 0xff, 0xff, 0, 0, 0, 0, 0, 0, 0, 0, 144, 0, 1, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0,
];
/// `rtw89_ext_capa_sta` (`core.c:208-212`), the station's extended
/// capabilities: extended channel switching, multiple BSSID, operating mode
/// notification.
const EXTENDED_CAPABILITIES: [u8; 10] = [127, 8, 0x04, 0, 0x40, 0, 0, 0, 0, 0x40];
/// `rtw89_channels_2ghz` and `rtw89_channels_5ghz` (`core.c:40-86`).
const CHANNELS_2GHZ: [u16; 14] = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14];
const CHANNELS_5GHZ: [u16; 28] = [
    36, 40, 44, 48, 52, 56, 60, 64, 100, 104, 108, 112, 116, 120, 124, 128, 132, 136, 140, 144,
    149, 153, 157, 161, 165, 169, 173, 177,
];
/// The Supported Channels a spectrum-managed request names (spec S14.4).
const SUPPORTED_2GHZ: [u8; 2] = [1, 14];
const SUPPORTED_5GHZ: [u8; 4] = [36, 28, 149, 9];

#[derive(Debug)]
pub enum Error {
    /// The chip's own error text, which names no address.
    Chip(String),
    Station(crate::wifi::station::Error),
    Data(data::Error),
    Key(security::Error),
    Scan(scan::Error),
    Refused(&'static str),
    Unsupported(&'static str),
    /// Frames must drain before this can go on.
    Pending,
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{self:?}")
    }
}

/// The keeper's handle on the IGTK the host holds.
pub struct ManagementKey;

/// One key the chip's security CAM holds: its slot, kind, key ID, entry
/// and material (for the host's sealing and opening under the PTK).
struct Key {
    slot: KeySlot,
    pairwise: bool,
    index: u8,
    cam: u8,
    material: Zeroizing<Vec<u8>>,
}

/// Whether `a` and `b` are the same key material, compared in time that
/// does not depend on where they differ.
fn same_material(a: &[u8], b: &[u8]) -> bool {
    a.len() == b.len() && a.iter().zip(b).fold(0u8, |d, (a, b)| d | (a ^ b)) == 0
}

/// A BSS and when it was last heard, a `cfg80211_internal_bss`.
struct Heard {
    observation: Arc<NativeBssObservation>,
    at: u64,
}

/// A data frame waiting for room: its MPDU (without its CCMP header when
/// it goes under a key), the queue's TID, whether it is EAPOL, the peer's
/// rates, and the key it goes under: CAM entry, key ID and its reserved PN.
struct Held {
    bytes: Vec<u8>,
    tid: u8,
    eapol: bool,
    supported: u16,
    key: Option<(u8, u8, u64)>,
}

/// What a management frame is to the progress line.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Kind {
    Authentication,
    Association,
    Other,
}

impl Kind {
    fn of(bytes: &[u8]) -> Self {
        match bytes[0] & 0xfc {
            0xb0 => Self::Authentication,
            0x00 | 0x20 => Self::Association,
            _ => Self::Other,
        }
    }
}

/// Frames of one exchange the chip took and saw acknowledged, and the
/// AP's last answer.
#[derive(Clone, Copy, Debug, Default)]
struct Exchange {
    sent: u32,
    acked: u32,
    status: Option<u16>,
}

#[derive(Clone, Copy, Debug, Default)]
struct Eapol {
    received: u32,
    sent: u32,
    acked: u32,
}

/// The progress line's counts (spec S14.6), each `None` until reached.
#[derive(Clone, Debug, Default)]
struct Progress {
    scans: u32,
    auth: Option<Exchange>,
    assoc: Option<Exchange>,
    aid: Option<u16>,
    eapol: Option<Eapol>,
    keys: Option<u32>,
    last: Option<&'static str>,
}

fn bump(counter: &mut u32) {
    *counter = counter.saturating_add(1);
}

/// `-` for a field not reached yet.
fn or_dash<T: fmt::Display>(value: Option<T>) -> String {
    value.map_or_else(|| String::from("-"), |value| format!("{value}"))
}

fn exchange(exchange: Option<Exchange>) -> String {
    exchange.map_or_else(
        || String::from("-"),
        |e| format!("{}/{} status {}", e.sent, e.acked, or_dash(e.status)),
    )
}

/// The channels the country admits, which the real check gates at each frame.
struct Admitted<'a>(&'a [(Channel, u32)]);

impl RegulatoryPolicy for Admitted<'_> {
    fn allow_management_tx(&mut self, channel: Channel) -> bool {
        self.0.iter().any(|(known, _)| *known == channel)
    }
    fn allow_data_tx(&mut self, channel: Channel) -> bool {
        self.allow_management_tx(channel)
    }
}

impl scan::Policy for Admitted<'_> {
    fn allow_receive(&mut self, channel: Channel) -> bool {
        self.0.iter().any(|(known, _)| *known == channel)
    }
    fn allow_probe(&mut self, _: Channel) -> bool {
        false
    }
    fn lookup_channel(&mut self, band: Band, number: u16) -> Option<Channel> {
        self.0
            .iter()
            .map(|(channel, _)| *channel)
            .find(|channel| channel.band == band && channel.number == number)
    }
}

/// A management frame from `local` to `ap` (rtw88's `management`).
fn management(local: Mac, ap: Mac, subtype: u16, protected: bool, body: &[u8]) -> Vec<u8> {
    let mut bytes = alloc::vec![0; 24];
    let control = subtype | if protected { 0x4000 } else { 0 };
    bytes[..2].copy_from_slice(&control.to_le_bytes());
    bytes[4..10].copy_from_slice(&ap.0);
    bytes[10..16].copy_from_slice(&local.0);
    bytes[16..22].copy_from_slice(&ap.0);
    bytes.extend_from_slice(body);
    bytes
}

/// The rates of `rtw89_bitrates` (`core.c:153-166`) on `band`, as the
/// Supported and Extended Supported Rates elements.
fn rate_elements(band: Band) -> &'static [u8] {
    match band {
        Band::Ghz2 => &[1, 8, 2, 4, 11, 22, 12, 18, 24, 36, 50, 4, 48, 72, 96, 108],
        _ => &[1, 8, 12, 18, 24, 36, 48, 72, 96, 108],
    }
}

/// The channels mac80211 registers for rtw89, as `country` leaves them.
fn channels(country: &Country) -> Vec<(Channel, u32)> {
    let mut out = Vec::new();
    let mut band = |band: Band, numbers: &[u16], frequency: fn(u16) -> u16| {
        let centers: Vec<u16> = numbers.iter().map(|&number| frequency(number)).collect();
        for (&number, regulated) in numbers
            .iter()
            .zip(country.band(&centers).unwrap_or_default())
        {
            if regulated.flags & DISABLED == 0 {
                out.push((
                    Channel {
                        band,
                        number,
                        frequency_mhz: frequency(number),
                    },
                    regulated.flags,
                ));
            }
        }
    };
    band(Band::Ghz2, &CHANNELS_2GHZ, |number| {
        if number == 14 {
            2484
        } else {
            2407 + 5 * number
        }
    });
    band(Band::Ghz5, &CHANNELS_5GHZ, |number| 5000 + 5 * number);
    out
}

/// `context` as `BSS_CHANGED_ASSOC` finds it (spec S10): the AP, its AID
/// (`& 0x7ff`, as mac80211 keeps it), the channel joined, the BSS's beacon
/// interval and DTIM period, the peer's rates, its negotiated HT
/// (`ieee80211_ht_cap_ie_to_sta_ht_cap`), the AP's WMM parameters when it
/// has WMM (none otherwise: the chip programs mac80211's QoS defaults),
/// and the slot `ieee80211_handle_bss_capability` takes from the
/// response's capability, always short off 2.4 GHz.
fn association(context: &AssociationContext) -> Association {
    let negotiated = context.negotiated();
    let channel = context.channel();
    Association {
        bssid: context.bssid(),
        aid: context.response().aid & 0x7ff,
        channel,
        beacon_interval: context.beacon().beacon_interval,
        dtim_period: context.dtim_period(),
        legacy: negotiated.legacy.supported,
        ht: negotiated.ht.as_ref().map(|ht| HtPeer {
            rx_mask: [
                ht.peer_rx_mcs[0],
                ht.peer_rx_mcs[1],
                ht.peer_rx_mcs[2],
                ht.peer_rx_mcs[3],
            ],
            ldpc: ht.capability & 0x0001 != 0,
            rx_stbc: ht.capability & 0x0300 != 0,
            sgi20: ht.capability & 0x0020 != 0,
        }),
        edca: negotiated.wmm.as_ref().map(|wmm| wmm.access_categories),
        short_slot: channel.band != Band::Ghz2 || context.response().capabilities & (1 << 10) != 0,
    }
}

/// An RTL8852CE kept connected to one network.
pub type Wifi<C> = Keeper<Rtl8852c<C>>;

/// An RTL8852CE's association state over its chip.
pub struct Rtl8852c<C: Chip> {
    chip: C,
    address: Mac,
    clock: Clock,
    country: Country,
    channels: Vec<(Channel, u32)>,
    station: Station,
    scanner: Scanner,
    /// The scan the chip runs, until it ends.
    scanning: Option<scan::Ticket>,
    /// A scan ended and waits for the keeper.
    scanned: Option<scan::Ticket>,
    observed: Vec<Heard>,
    outbox: VecDeque<(Option<Ticket>, Vec<u8>)>,
    events: VecDeque<keeper::Event>,
    received: VecDeque<Vec<u8>>,
    held: VecDeque<Held>,
    admitted: Option<(Ticket, Arc<AssociationContext>)>,
    context: Option<Arc<AssociationContext>>,
    /// The BSS joined, as the keeper's profile of it.
    peer: Option<Peer>,
    /// The ticketed management frame whose report the station awaits.
    reporting: Option<(Token, Kind)>,
    /// Ticketed management frames the chip took before the one the station
    /// awaits, until their reports: counted, not the station's.
    earlier: VecDeque<(Token, Kind)>,
    /// The EAPOL frames the chip took, until their reports.
    eapol_reports: VecDeque<Token>,
    /// The keys the chip's security CAM holds.
    keys: Vec<Key>,
    /// The next `KeySlot` number: each key gets its own.
    next_slot: u16,
    /// The join's management cipher, and the IGTK the host checks group
    /// frames under.
    management_cipher: Option<ManagementCipher>,
    integrity: Option<BipKey>,
    fragments: Fragments,
    port_open: bool,
    departing: Option<Ticket>,
    left: bool,
    reported: Vec<&'static str>,
    /// The configured network's name, compared and never printed.
    ssid: Vec<u8>,
    progress: Progress,
}

impl<C: Chip> Rtl8852c<C> {
    /// `chip`, running at `address` under `country`, as a keeper's radio.
    pub fn new(chip: C, address: Mac, country: Country, clock: Clock) -> Result<Self, Error> {
        let channels = channels(&country);
        Ok(Self {
            chip,
            address,
            clock,
            country,
            channels,
            station: Station::new(address).map_err(Error::Station)?,
            scanner: Scanner::new(address).map_err(Error::Scan)?,
            scanning: None,
            scanned: None,
            observed: Vec::new(),
            outbox: VecDeque::new(),
            events: VecDeque::new(),
            received: VecDeque::new(),
            held: VecDeque::new(),
            admitted: None,
            context: None,
            peer: None,
            reporting: None,
            earlier: VecDeque::new(),
            eapol_reports: VecDeque::new(),
            keys: Vec::new(),
            next_slot: 0,
            management_cipher: None,
            integrity: None,
            fragments: Fragments::default(),
            port_open: false,
            departing: None,
            left: false,
            reported: Vec::new(),
            ssid: Vec::new(),
            progress: Progress::default(),
        })
    }

    /// Keep `self` connected to `network`; `entropy` gives each handshake's
    /// SNonce.
    pub fn keep(mut self, network: Network, entropy: Arc<dyn Entropy>) -> Wifi<C> {
        let clock = self.clock;
        self.ssid = network.ssid.clone();
        Keeper::keep(self, network, entropy, clock)
    }

    /// The chip, as the radio left it.
    pub fn into_chip(self) -> C {
        self.chip
    }

    fn chip<T>(result: Result<T, String>) -> Result<T, Error> {
        result.map_err(Error::Chip)
    }

    /// `result`, its refusal kept as the progress line's `last`: what the
    /// radio itself refused the keeper.
    fn note<T>(&mut self, result: Result<T, Error>) -> Result<T, Error> {
        if let Err(Error::Refused(why) | Error::Unsupported(why)) = &result {
            self.progress.last = Some(why);
        }
        result
    }

    /// `operation` declined, reported once. A frame dropped is no refusal
    /// of the keeper's: the progress line's `last` stays.
    fn unsupported(&mut self, operation: &'static str) {
        if !self.reported.contains(&operation) {
            self.reported.push(operation);
            self.events.push_back(keeper::Event::Unsupported(operation));
        }
    }

    fn act(&mut self, action: Action) {
        match action {
            Action::Transmit { ticket, bytes, .. } => self.queue_management(Some(ticket), bytes),
            Action::Configure {
                ticket,
                context: Some(context),
                ..
            } => self.admitted = Some((ticket, context)),
            // Every join here is a modern one, which carries its context.
            Action::Configure { .. } => {
                self.admitted = None;
                if let Ok(Action::Remove { ticket, .. }) = self.station.disconnect() {
                    self.depart(ticket);
                }
            }
            Action::Remove { ticket, .. } => self.depart(ticket),
        }
    }

    /// `bytes` to the outbox, a protected frame sealed first: mac80211
    /// encrypts a frame as it enters its TX path. One that cannot be sealed
    /// (no PTK) is dropped, reported once.
    fn queue_management(&mut self, ticket: Option<Ticket>, bytes: Vec<u8>) {
        if bytes.get(1).is_some_and(|fc| fc & 0x40 != 0) {
            match self.seal(&bytes) {
                Ok(sealed) => self.outbox.push_back((ticket, sealed)),
                Err(_) => self.unsupported("a protected management frame without its key"),
            }
        } else {
            self.outbox.push_back((ticket, bytes));
        }
    }

    /// The station left of the AP's doing or its own: an association's
    /// `ieee80211_set_disassoc` without a frame of its own flushes with
    /// drop first, its failure passed over as Linux's is.
    fn depart(&mut self, ticket: Ticket) {
        if self.context.is_some() {
            let _ = self.chip.flush(true);
        }
        self.leave(ticket);
    }

    /// The station left: nothing it queued leaves after it.
    fn leave(&mut self, ticket: Ticket) {
        self.departing = Some(ticket);
        self.left = true;
        self.port_open = false;
        self.admitted = None;
        self.outbox.clear();
        self.held.clear();
    }

    /// The station's state gone after its retirement, the keys with it. A
    /// report still awaited is counted when it comes.
    fn clear(&mut self) {
        self.context = None;
        self.peer = None;
        self.admitted = None;
        self.departing = None;
        self.left = false;
        if let Some(report) = self.reporting.take() {
            self.keep_earlier(report);
        }
        self.port_open = false;
        self.outbox.clear();
        self.held.clear();
        self.keys.clear();
        self.management_cipher = None;
        self.integrity = None;
        self.fragments.clear();
    }

    /// A report the station no longer awaits, kept for the progress line.
    fn keep_earlier(&mut self, report: (Token, Kind)) {
        if self.earlier.len() >= MAX_EAPOL_REPORTS {
            self.earlier.pop_front();
        }
        self.earlier.push_back(report);
    }

    fn observed(&self, handle: usize) -> Result<Arc<NativeBssObservation>, Error> {
        self.observed
            .get(handle)
            .map(|heard| heard.observation.clone())
            .ok_or(Error::Refused("the BSS is gone"))
    }

    /// The basic rates management frames leave at: the BSS's, once joined
    /// (`link->conf->basic_rates`, which mac80211 sets as it prepares the
    /// connection).
    fn basic_rates(&self) -> u16 {
        self.peer.map_or(0, |peer| peer.basic_rates)
    }

    /// rtw88's `send_management` (`station.rs:889`): the outbox, oldest
    /// first, while the chip has room; a ticketed frame's token is kept
    /// for its report, the token it replaces among the earlier ones. A
    /// protected frame was sealed as it was queued.
    fn send_management(&mut self) -> Result<(), Error> {
        let basic_rates = self.basic_rates();
        while let Some((_, bytes)) = self.outbox.front() {
            let frame = Management { bytes, basic_rates };
            let Some(token) = Self::chip(self.chip.transmit_management(&frame))? else {
                return Ok(());
            };
            let Some((ticket, bytes)) = self.outbox.pop_front() else {
                break;
            };
            let kind = Kind::of(&bytes);
            match kind {
                Kind::Authentication => bump(&mut self.progress.auth.get_or_insert_default().sent),
                Kind::Association => bump(&mut self.progress.assoc.get_or_insert_default().sent),
                Kind::Other => {}
            }
            if let Some(ticket) = ticket {
                if let Some(earlier) = self.reporting.replace((token, kind)) {
                    self.keep_earlier(earlier);
                }
                self.station.submitted(ticket).map_err(Error::Station)?;
            }
        }
        Ok(())
    }

    /// rtw88's `send_data` (`station.rs:943`): `held`, oldest first, while
    /// the chip has room. Each EAPOL frame's token is kept for its report.
    fn send_data(&mut self) -> Result<(), Error> {
        while let Some(held) = self.held.front() {
            let frame = Data {
                bytes: &held.bytes,
                tid: held.tid,
                eapol: held.eapol,
                supported_rates: held.supported,
                key: held.key,
            };
            let Some(token) = Self::chip(self.chip.transmit_data(&frame))? else {
                return Ok(());
            };
            let eapol = held.eapol;
            self.held.pop_front();
            if eapol {
                bump(&mut self.progress.eapol.get_or_insert_default().sent);
                if self.eapol_reports.len() >= MAX_EAPOL_REPORTS {
                    self.eapol_reports.pop_front();
                }
                self.eapol_reports.push_back(token);
            }
        }
        Ok(())
    }

    /// rtw88's `seal` (`station.rs:918`), `IEEE80211_KEY_FLAG_SW_MGMT_TX`'s
    /// host half: `bytes`, a robust management frame, sealed under the PTK
    /// at the next PN of its slot, which data shares, as one key's
    /// `tx_pn` counts both in Linux.
    fn seal(&mut self, bytes: &[u8]) -> Result<Vec<u8>, Error> {
        let key = self
            .keys
            .iter_mut()
            .find(|key| key.pairwise)
            .ok_or(Error::Refused("no pairwise key"))?;
        let identity = key.slot.installed().ok_or(Error::Refused("no key"))?;
        if bytes.len() < MANAGEMENT_HEADER {
            return Err(Error::Refused("a management frame shorter than its header"));
        }
        let nonce = key
            .slot
            .reserve_host_tx_nonce(identity)
            .map_err(Error::Key)?;
        let pn = nonce.packet_number();
        let header: [u8; CCMP_HEADER] = nonce.into_header();
        let material: &[u8; 16] = key.material[..]
            .try_into()
            .map_err(|_| Error::Unsupported(CCMP_128_ONLY))?;
        Ok(ccmp::seal(
            material,
            &bytes[..MANAGEMENT_HEADER],
            &header,
            pn,
            &bytes[MANAGEMENT_HEADER..],
        ))
    }

    /// rtw88's `send` (`station.rs:1121`, spec S13.1) without A-MSDU or TX
    /// Block Ack: the MSDU as the station prepares it under the keys held,
    /// a `HostIv` frame's CCMP header checked against its reservation and
    /// removed (spec S5.4), held, then sent while the chip has room, as
    /// mac80211 hands a frame to the driver at once. A frame the strip
    /// refuses is dropped, its PN spent.
    fn send(&mut self, ethernet: &[u8], class: TxClass) -> Result<(), Error> {
        let epoch = self
            .station
            .association_epoch()
            .ok_or(Error::Refused("no association"))?;
        let context = self.context.clone().ok_or(Error::Refused("no context"))?;
        let tid = data::data_tid(&context, None, None, ethernet);
        let mut policy = Admitted(&self.channels);
        let mut keys: Vec<&mut KeySlot> = self.keys.iter_mut().map(|key| &mut key.slot).collect();
        let plan = match tid {
            Some(tid) => self.station.prepare_qos_msdu_refs(
                epoch,
                ethernet,
                class,
                Qos {
                    context: &context,
                    tid,
                },
                &mut keys,
                &mut policy,
            ),
            None => self.station.prepare_msdu_refs(
                epoch,
                ethernet,
                class,
                SequenceOwner::Host,
                &mut keys,
                &mut policy,
            ),
        }
        .map_err(Error::Data)?;
        let (_, _, mut mpdu, protection, _, _) = plan.into_parts();
        let key = match protection {
            Protection::None => None,
            Protection::HostIv { key, pn } => {
                let cam = self
                    .keys
                    .iter()
                    .find(|held| held.slot.installed() == Some(key))
                    .map(|held| held.cam)
                    .ok_or(Error::Refused("a key the chip does not hold"))?;
                txd::strip_ccmp(&mut mpdu, pn.value(), key.index())
                    .map_err(|error| Error::Chip(format!("{error}")))?;
                Some((cam, key.index(), pn.value()))
            }
            Protection::Firmware { .. } => return Err(Error::Refused("a firmware key")),
        };
        if self.held.len() >= MAX_HELD {
            self.held.pop_front();
        }
        self.held.push_back(Held {
            bytes: mpdu,
            tid: tid.unwrap_or(0),
            eapol: class != TxClass::Data,
            supported: context.negotiated().legacy.supported,
            key,
        });
        self.send_data()
    }

    /// What the chip handed on since the last poll, every item taken; then
    /// the poll's own error, or the first an item gave.
    fn take_incoming(&mut self, now: u64) -> Result<(), Error> {
        let mut incoming = Vec::new();
        let mut first = Self::chip(self.chip.poll(&mut |item| incoming.push(item)));
        for item in incoming {
            let taken = match item {
                Incoming::Transmitted {
                    token,
                    acknowledged,
                } => self.transmitted(token, acknowledged, now),
                Incoming::Frame(received) => match Self::mpdu(received) {
                    Some(mpdu) => match self.frame(mpdu, now) {
                        // mac80211 drops a frame the station cannot use;
                        // the exchange it answers retries or times out.
                        Err(Error::Station(_)) => {
                            self.unsupported("a management frame the station refuses");
                            Ok(())
                        }
                        taken => taken,
                    },
                    None => Ok(()),
                },
                Incoming::ScanEnded { completed } => self.scan_ended(completed),
                // Beacon loss is M3 Task 18's; no chip reports it yet.
                Incoming::BeaconLost => Ok(()),
            };
            if first.is_ok() {
                first = taken;
            }
        }
        first
    }

    /// A frame's release report: the ticketed management frame's goes to
    /// the station as its TX status (rtw88's `Report::Transmitted`,
    /// `station.rs:1275-1288`, spec S1.10), and is counted; one the station
    /// no longer awaits, and an EAPOL frame's, is counted only.
    fn transmitted(&mut self, token: Token, acknowledged: bool, now: u64) -> Result<(), Error> {
        if let Some((_, kind)) = self.reporting.filter(|&(awaited, _)| awaited == token) {
            self.reporting = None;
            self.count_ack(kind, acknowledged);
            return self
                .station
                .management_status(acknowledged, now)
                .map_err(Error::Station);
        }
        if let Some(at) = self.earlier.iter().position(|&(sent, _)| sent == token) {
            if let Some((_, kind)) = self.earlier.remove(at) {
                self.count_ack(kind, acknowledged);
            }
            return Ok(());
        }
        if let Some(at) = self.eapol_reports.iter().position(|&sent| sent == token) {
            self.eapol_reports.remove(at);
            if acknowledged && let Some(eapol) = &mut self.progress.eapol {
                bump(&mut eapol.acked);
            }
        }
        Ok(())
    }

    /// An acknowledged authentication or association, counted.
    fn count_ack(&mut self, kind: Kind, acknowledged: bool) {
        let exchange = match kind {
            Kind::Authentication => self.progress.auth.as_mut(),
            Kind::Association => self.progress.assoc.as_mut(),
            Kind::Other => None,
        };
        if acknowledged && let Some(exchange) = exchange {
            bump(&mut exchange.acked);
        }
    }

    /// The scan ended (spec S8.2): its ticket completed, the chip restored
    /// (`rtw89_hw_scan_complete`, spec S8.3), and the scan left for the
    /// keeper to retire, even when the restore failed, which powered the
    /// chip off.
    fn scan_ended(&mut self, completed: bool) -> Result<(), Error> {
        let Some(ticket) = self.scanning.take() else {
            return Ok(());
        };
        let outcome = if completed {
            Outcome::Completed
        } else {
            Outcome::Aborted
        };
        let recorded = self.scanner.completed(ticket, outcome).map_err(Error::Scan);
        let restored = Self::chip(self.chip.end_scan());
        self.scanned = Some(ticket);
        bump(&mut self.progress.scans);
        recorded.and(restored)
    }

    /// `received` as an MPDU on the channel the chip heard it on, its
    /// security status (spec S7.3) and FCS status with it.
    fn mpdu(received: Received) -> Option<Mpdu> {
        let channel = received.channel;
        let radio = RadioStatus {
            band: channel.band,
            channel: channel.number,
            frequency_mhz: channel.frequency_mhz,
            signal_dbm: received.signal_dbm,
            chain_signal_dbm: [None; 4],
            rate: RawRate {
                layout: 0,
                words: [0; 4],
            },
            rx_rate: None,
            tsf: Some(Timestamp::Low32(received.tsf)),
        };
        let security = SecurityStatus {
            decrypted: received.decrypted,
            integrity: if received.decrypted {
                Integrity::Verified
            } else {
                Integrity::Unknown
            },
            icv_error: received.icv_error,
            ..SecurityStatus::default()
        };
        let mut mpdu = Mpdu::new(received.bytes, radio, security).ok()?;
        mpdu.fcs_error = received.fcs_error;
        Some(mpdu)
    }

    /// rtw88's `frame` (`station.rs:1344`) without TKIP or Block Ack
    /// reordering (spec S13.3): a scan's beacons and probe responses to the
    /// scanner, management frames to [`Self::management_frame`], and data
    /// with a body to the station, a protected frame under the key its key
    /// ID names (pairwise for unicast, a group key for a group address),
    /// dropped without one, failing its ICV, or left encrypted by the
    /// hardware (mac80211 would decrypt it in software; no Stela station
    /// does). Fragments are reassembled.
    fn frame(&mut self, mut frame: Mpdu, now: u64) -> Result<(), Error> {
        let header = *frame.header();
        if let Some(ticket) = self.scanning
            && matches!(header.control & 0xfc, 0x80 | 0x50)
        {
            let mut policy = Admitted(&self.channels);
            if let Ok(bss) = self.scanner.observe(ticket, &frame, &mut policy) {
                self.hear(bss, now);
            }
            return Ok(());
        }
        if header.control & 12 == 0 {
            return self.management_frame(frame, now);
        }
        if header.control & 12 != 8 || !header.data_present() {
            return Ok(());
        }
        if header.protected() {
            let multicast = header.receiver.is_multicast();
            let index = frame.bytes().get(header.length + 3).map(|byte| byte >> 6);
            let Some(key) = self
                .keys
                .iter()
                .find(|key| key.pairwise != multicast && Some(key.index) == index)
                .and_then(|key| key.slot.installed())
            else {
                return Ok(());
            };
            if frame.security.icv_error || !frame.security.decrypted {
                return Ok(());
            }
            frame.security.key = Some(key);
        }
        if header.fragmented() {
            self.reassemble(&frame, now);
            return Ok(());
        }
        self.deliver(frame);
        Ok(())
    }

    /// rtw88's `reassemble` (`station.rs:1415`): `frame`, one fragment;
    /// the station has the MSDU with the last of them.
    fn reassemble(&mut self, frame: &Mpdu, now: u64) {
        let key = frame.security.key;
        let slot = self
            .keys
            .iter_mut()
            .map(|key| &mut key.slot)
            .find(|slot| key.is_some() && slot.installed() == key);
        if let Some(whole) = self.fragments.add(frame, slot, now) {
            self.deliver(whole);
        }
    }

    /// rtw88's `deliver` (`station.rs:1451`): `frame`, an MSDU or an
    /// A-MSDU, to the station; mac80211 drops what its replay and port
    /// checks refuse. EAPOL goes to the keeper.
    fn deliver(&mut self, frame: Mpdu) {
        let ingress = if frame.header().amsdu {
            let Some(context) = self.context.clone() else {
                return;
            };
            let key = frame.security.key;
            // SAFETY: an A-MSDU is one record here, delivered as received
            // (no reorder buffer holds it), and `clear` drops what an
            // older association left.
            let group =
                unsafe { OrderedGroup::new(context, alloc::vec![frame], Shape::WholeMpdu, ()) };
            let slot = self
                .keys
                .iter_mut()
                .map(|key| &mut key.slot)
                .find(|slot| key.is_some() && slot.installed() == key);
            let Ok(received) = self.station.receive_ordered_group(&group, slot) else {
                return;
            };
            let keys: Vec<&KeySlot> = self.keys.iter().map(|key| &key.slot).collect();
            match self.station.receive_group_refs(&received, &keys) {
                Ok(ingress) => ingress,
                Err(_) => return,
            }
        } else {
            let Some(epoch) = self.station.association_epoch() else {
                return;
            };
            let mut keys: Vec<&mut KeySlot> =
                self.keys.iter_mut().map(|key| &mut key.slot).collect();
            match self.station.receive_msdu_refs(epoch, &frame, &mut keys) {
                Ok(ingress) => alloc::vec![ingress],
                Err(_) => return,
            }
        };
        for ingress in ingress {
            match ingress {
                Ingress::Data(ethernet) => self.received.push_back(ethernet),
                Ingress::ControlPort { ethernet, .. } => {
                    bump(&mut self.progress.eapol.get_or_insert_default().received);
                    self.events.push_back(keeper::Event::Eapol(ethernet));
                }
            }
        }
    }

    fn hear(&mut self, mut bss: Bss, now: u64) {
        let known = self
            .observed
            .iter_mut()
            .find(|heard| heard.observation.bss().bssid == bss.bssid);
        match known {
            Some(heard) => {
                bss.keep_name_of(heard.observation.bss());
                heard.observation = Arc::new(NativeBssObservation { bss });
                heard.at = now;
            }
            None => self.observed.push(Heard {
                observation: Arc::new(NativeBssObservation { bss }),
                at: now,
            }),
        }
    }

    /// rtw88's `management_frame` (`station.rs:1514`): what the station's
    /// authentication and association take, and the AP's departures; a
    /// protected frame to [`Self::protected_management`]. Under PMF a
    /// robust frame to everyone must carry its IGTK's MIC (BIP on the host)
    /// and a departure to the station alone must come protected once keys
    /// are installed (`ieee80211_drop_unencrypted_mgmt`). Beacons and
    /// probe responses outside a scan and unprotected action frames (Block
    /// Ack is M3 Task 16's; channel switches are not followed) are dropped,
    /// as is everything before a join.
    fn management_frame(&mut self, frame: Mpdu, now: u64) -> Result<(), Error> {
        let header = *frame.header();
        let subtype = header.control & 0xfc;
        if matches!(subtype, 0x80 | 0x50) {
            return Ok(());
        }
        if header.protected() {
            return self.protected_management(frame);
        }
        if subtype == 0xd0 || self.station.phase() == Peering::ExternalAuthenticating {
            return Ok(());
        }
        let Some(epoch) = self.station.association_epoch() else {
            return Ok(());
        };
        let pmf = self.station.requires_protected_management();
        if pmf && header.receiver.is_multicast() && protection::robust(&frame) {
            let Some(integrity) = &mut self.integrity else {
                return Ok(());
            };
            let Ok(authenticated) = integrity.authenticate(&frame) else {
                return Ok(());
            };
            if matches!(subtype, 0xa0 | 0xc0) {
                let body = authenticated.body();
                let action = self
                    .station
                    .receive_protected_departure(epoch, &frame, body)
                    .map_err(Error::Station)?;
                self.events.push_back(keeper::Event::ConnectionLost);
                self.act(action);
            }
            return Ok(());
        }
        if pmf && matches!(subtype, 0xa0 | 0xc0) && !self.keys.is_empty() {
            return Ok(());
        }
        let before = self.station.phase();
        let mut policy = Admitted(&self.channels);
        let action = self
            .station
            .receive(epoch, &frame, now, &mut policy)
            .map_err(Error::Station)?;
        let authenticated = self.answered(before, subtype, &action);
        self.act(action);
        if authenticated {
            Self::chip(self.chip.authenticated())?;
        }
        Ok(())
    }

    /// rtw88's `protected_management` (`station.rs:1598`) without Block
    /// Ack (M3 Task 16) or channel switches: a frame under the PTK, opened
    /// on the host when the hardware left it sealed, taken once under its
    /// packet number (`ieee80211_crypto_ccmp_decrypt` keeps management's
    /// apart from data's); a departure ends the association, and an SA
    /// Query request is answered, sealed in turn.
    fn protected_management(&mut self, mut frame: Mpdu) -> Result<(), Error> {
        let local = self.address;
        let Some(epoch) = self.station.association_epoch() else {
            return Ok(());
        };
        let header = *frame.header();
        let Some(key) = self.keys.iter_mut().find(|key| key.pairwise) else {
            return Ok(());
        };
        if frame.security.icv_error {
            return Ok(());
        }
        if !frame.security.decrypted {
            let number = frame
                .bytes()
                .get(header.length..)
                .and_then(|iv| PacketNumber::from_header(iv).ok());
            let material: Option<&[u8; 16]> = key.material[..].try_into().ok();
            let (Some((number, _)), Some(material)) = (number, material) else {
                return Ok(());
            };
            let Some(opened) = ccmp::open(material, frame.bytes(), number) else {
                return Ok(());
            };
            let security = SecurityStatus {
                decrypted: true,
                integrity: Integrity::Verified,
                ..SecurityStatus::default()
            };
            let Ok(clear) = Mpdu::new(opened, frame.radio.clone(), security) else {
                return Ok(());
            };
            frame = clear;
        }
        frame.security.key = key.slot.installed();
        let Ok(authenticated) = key.slot.authorize_unicast_management(&frame, local) else {
            return Ok(());
        };
        let body = authenticated.body();
        match header.control & 0xfc {
            0xa0 | 0xc0 => {
                let action = self
                    .station
                    .receive_protected_departure(epoch, &frame, body)
                    .map_err(Error::Station)?;
                self.events.push_back(keeper::Event::ConnectionLost);
                self.act(action);
            }
            0xd0 if body.len() >= 4 && body[..2] == [8, 0] => {
                let response = [8, 1, body[2], body[3]];
                let Some(ap) = header.transmitter else {
                    return Ok(());
                };
                let bytes = management(local, ap, 0xd0, true, &response);
                self.queue_management(None, bytes);
            }
            _ => {}
        }
        Ok(())
    }

    /// The AP's answer `action` came of, for the progress line: whether it
    /// accepted the authentication.
    fn answered(&mut self, before: Peering, subtype: u16, action: &Action) -> bool {
        let rejected = match action {
            Action::Remove {
                failure: Some(Failure::PeerRejected(status)),
                ..
            } => Some(*status),
            _ => None,
        };
        match (before, subtype) {
            (Peering::Authenticating, 0xb0) => {
                let accepted = self.station.phase() == Peering::Associating;
                let status = if accepted { Some(0) } else { rejected };
                if let (Some(status), Some(auth)) = (status, &mut self.progress.auth) {
                    auth.status = Some(status);
                }
                if accepted {
                    self.progress.assoc.get_or_insert_default();
                }
                accepted
            }
            (Peering::Associating, 0x10 | 0x30) => {
                let assoc = self.progress.assoc.get_or_insert_default();
                if let Action::Configure { response, .. } = action {
                    assoc.status = Some(response.status);
                    self.progress.aid = Some(response.aid & 0x7ff);
                } else if rejected.is_some() {
                    assoc.status = rejected;
                }
                false
            }
            _ => false,
        }
    }

    fn operate(&mut self) -> Result<(), Error> {
        let now = self.clock.now_us();
        let mut first = self.take_incoming(now);
        let mut policy = Admitted(&self.channels);
        if !self.left {
            match self.station.poll(now, &mut policy) {
                Ok(Some(action)) => self.act(action),
                Ok(None) => {}
                Err(error) => {
                    if first.is_ok() {
                        first = Err(Error::Station(error));
                    }
                }
            }
        }
        let sent = self.send_management();
        if first.is_ok() {
            first = sent;
        }
        let sent = self.send_data();
        if first.is_ok() {
            first = sent;
        }
        first
    }

    /// The progress line (spec S14.6), then the chip's counts.
    fn progress(&self) -> String {
        let p = &self.progress;
        let target = if p.scans == 0 {
            String::from("-")
        } else {
            self.observed
                .iter()
                .map(|heard| heard.observation.bss())
                .filter(|bss| bss.ssid.as_deref() == Some(&self.ssid[..]))
                .max_by_key(|bss| bss.signal_dbm.unwrap_or(i8::MIN))
                .map_or_else(
                    || String::from("missing"),
                    |bss| {
                        format!(
                            "found ch {} {} dBm",
                            bss.channel.number,
                            or_dash(bss.signal_dbm)
                        )
                    },
                )
        };
        let mut line = format!(
            "scans {} ({} networks, target {target}), auth {}",
            p.scans,
            self.observed.len(),
            exchange(p.auth),
        );
        match p.assoc {
            Some(_) => line.push_str(&format!(
                ", assoc {} aid {}",
                exchange(p.assoc),
                or_dash(p.aid)
            )),
            None => line.push_str(", assoc -"),
        }
        match p.eapol {
            Some(e) => line.push_str(&format!(
                ", eapol rx {} tx {} acked {}",
                e.received, e.sent, e.acked
            )),
            None => line.push_str(", eapol -"),
        }
        line.push_str(&format!(", keys {}", or_dash(p.keys)));
        line.push_str(if self.port_open {
            ", port open"
        } else {
            ", port closed"
        });
        if let Some(last) = p.last {
            line.push_str(&format!(", last \"{last}\""));
        }
        line.push_str("; ");
        line.push_str(&self.chip.diagnostic());
        line
    }

    fn begin_scan_with(&mut self, request: scan::Request, budget_us: u64) -> Result<(), Error> {
        let now = self.clock.now_us();
        self.observed
            .retain(|heard| now.saturating_sub(heard.at) < BSS_EXPIRY_US);
        let channels = request.channels().to_vec();
        let mut policy = Admitted(&self.channels);
        let ticket = self
            .scanner
            .begin(request, now, budget_us, &mut policy)
            .map_err(Error::Scan)?;
        let validated = self
            .scanner
            .validate_submission(ticket, now, &mut policy)
            .and_then(|()| self.scanner.submitted(ticket));
        let begun = match validated {
            Ok(()) => Self::chip(self.chip.begin_scan(&channels)),
            Err(error) => Err(Error::Scan(error)),
        };
        if let Err(error) = begun {
            // Nothing runs: the chip refused before its scan started, or
            // aborted and restored it (`rtw89_hw_scan_abort`).
            let _ = self.scanner.start_rejected(ticket, 0);
            let _ = self.scanner.retire(ticket, Retirement::Confirmed);
            return Err(error);
        }
        self.scanning = Some(ticket);
        Ok(())
    }

    fn join_with(&mut self, handle: usize, join: Join, peer: Peer) -> Result<(), Error> {
        let observed = self.observed(handle)?;
        let bss = observed.bss().clone();
        // `ieee80211_prep_connection`: the channel, then the BSSID.
        Self::chip(self.chip.tune(bss.channel))?;
        Self::chip(self.chip.set_bssid(bss.bssid))?;
        self.peer = Some(peer);
        self.management_cipher = join.request.pmf().map(|pmf| pmf.management());
        self.progress.auth.get_or_insert_default();
        let now = self.clock.now_us();
        let mut policy = Admitted(&self.channels);
        let begun = if join.request.pmf().is_some() {
            self.station
                .begin_modern_pmf(join, observed, now, &mut policy)
        } else {
            self.station.begin_modern(join, bss, now, &mut policy)
        };
        match begun {
            Ok(action) => {
                self.act(action);
                Ok(())
            }
            Err(error) => {
                // No station began: the BSSID goes again, so a scan may run.
                self.peer = None;
                let _ = self.chip.disassociate();
                Err(Error::Station(error))
            }
        }
    }

    /// rtw88's `configure` (`station.rs:2113`) on rtw89 (spec S14.5): the
    /// chip's association on the join's channel, without a switch of the
    /// radio's own, then the station configured. RX Block Ack is M3 Task
    /// 16's.
    fn configure_admitted(&mut self) -> Result<(), Error> {
        let (ticket, context) = self
            .admitted
            .take()
            .ok_or(Error::Refused("configuration"))?;
        Self::chip(self.chip.associate(&association(&context)))?;
        self.context = Some(context);
        self.station
            .complete_configuration(ticket, Completion::Accepted, self.clock.now_us())
            .map_err(Error::Station)?;
        self.progress.eapol.get_or_insert_default();
        Ok(())
    }

    /// rtw88's `deauthenticate` (`station.rs:2296`): the deauthentication
    /// after what the outbox holds, then the station's departure, even when
    /// the chip refused the frame. Held data never leaves after it. From an
    /// association, as `ieee80211_set_disassoc` sends it: a flush with drop
    /// first (mac80211 purges its queues, the chip drops what it holds),
    /// the frame, then a flush without drop so it leaves (`mlme.c:4594-4604`
    /// at the donor's mac80211); `ieee80211_mgd_deauth` flushes neither
    /// during an authentication or association. A flush's failure is passed
    /// over, as Linux's is.
    fn deauthenticate_with(&mut self, reason: u16) -> Result<(), Error> {
        if self.left || self.station.phase() == Peering::Idle {
            return Ok(());
        }
        let associated = self.context.is_some();
        if associated {
            self.held.clear();
            let _ = self.chip.flush(true);
        }
        if let Ok((local, ap)) = self.station.addresses() {
            let protected = self.station.protected_port_authorized();
            let bytes = management(local, ap, 0xc0, protected, &reason.to_le_bytes());
            self.queue_management(None, bytes);
        }
        let sent = self.send_management();
        if associated {
            let _ = self.chip.flush(false);
        }
        match self.station.disconnect().map_err(Error::Station)? {
            Action::Remove { ticket, .. } => self.leave(ticket),
            action => self.act(action),
        }
        sent
    }

    /// rtw88's `retire_peer` (`station.rs:2311`) on rtw89 (spec S14.5):
    /// `sta_info_flush`'s key deletes first, the PTK, then the group keys by
    /// key ID, each as `rtw89_ops_set_key`'s DISABLE_KEY runs it, a flush
    /// without drop, then the CAM delete (`mac80211.c:1010-1015`); then the
    /// chip's disassociation (spec S11), which releases the channel; then
    /// the removal is accepted. Every step runs whatever an earlier one
    /// returned, as mac80211 goes on past a driver's failures, and the
    /// first error is returned. A failed disassociation powered the chip
    /// off, which holds no association either.
    fn retire(&mut self) -> Result<(), Error> {
        let mut keys = core::mem::take(&mut self.keys);
        keys.sort_by_key(|key| (!key.pairwise, key.index));
        let mut first = Ok(());
        for key in &keys {
            let _ = self.chip.flush(false);
            let removed = Self::chip(self.chip.remove_key(key.cam));
            if first.is_ok() {
                first = removed;
            }
        }
        drop(keys);
        let left = Self::chip(self.chip.disassociate());
        if first.is_ok() {
            first = left;
        }
        if let Some(ticket) = self.departing.take() {
            self.station
                .complete_removal(ticket, Completion::Accepted)
                .map_err(Error::Station)?;
        }
        self.clear();
        first
    }

    /// `rtw89_ops_set_key`'s SET_KEY (spec S12, S14.5) of a CCMP-128 key:
    /// the pairwise key with `TransmitSeed::host(0)`, a group key
    /// receive-only. As `ieee80211_key_link`: an identical key of the same
    /// kind and index is not installed again (-EALREADY, which mac80211
    /// takes as success); another replaces it (`ieee80211_key_replace`),
    /// the old one deleted first (a flush without drop, then the CAM
    /// delete, its failure passed over), then the new one installed. The
    /// key counts as installed only once the chip's `add_key` returned its
    /// CAM entry, the firmware having acknowledged it; otherwise its slot
    /// is never completed and goes. An open port is authorized again on the
    /// keys then held.
    fn install(
        &mut self,
        pairwise: bool,
        index: u8,
        material: &[u8],
        receive: ReplaySeed,
    ) -> Result<KeyIdentity, Error> {
        self.progress.keys.get_or_insert(0);
        let temporal: &[u8; 16] = material
            .try_into()
            .map_err(|_| Error::Unsupported(CCMP_128_ONLY))?;
        let peer = self
            .station
            .pairwise_peer()
            .ok_or(Error::Refused("no peer"))?;
        if let Some(at) = self
            .keys
            .iter()
            .position(|key| key.pairwise == pairwise && key.index == index)
        {
            if same_material(&self.keys[at].material, material)
                && let Some(identity) = self.keys[at].slot.installed()
            {
                return Ok(identity);
            }
            let old = self.keys.remove(at);
            // What still waits under the old key goes with it: its CAM
            // entry may hold the new key by the time the chip has room.
            self.held
                .retain(|held| held.key.is_none_or(|(cam, _, _)| cam != old.cam));
            let _ = self.chip.flush(false);
            let _ = self.chip.remove_key(old.cam);
        }
        let scope = if pairwise {
            KeyScope::Pairwise(peer)
        } else {
            KeyScope::Group(peer)
        };
        let transmit = if pairwise {
            TransmitSeed::host(PacketNumber::new(0).ok_or(Error::Refused("packet number"))?)
        } else {
            TransmitSeed::RECEIVE_ONLY
        };
        // SAFETY: the counter gives each key of the radio its own slot,
        // which no other `KeySlot` controls.
        let mut slot = unsafe { KeySlot::new(self.next_slot) }.map_err(Error::Key)?;
        self.next_slot = self.next_slot.wrapping_add(1);
        let ticket = slot
            .begin_install(index, Cipher::Ccmp128, material, receive, scope, transmit)
            .map_err(Error::Key)?;
        let identity = ticket.identity();
        let cam = Self::chip(self.chip.add_key(&HardwareKey {
            pairwise,
            index,
            material: temporal,
        }))?;
        slot.complete_install(ticket, InstallOutcome::Accepted)
            .map_err(Error::Key)?;
        self.keys.push(Key {
            slot,
            pairwise,
            index,
            cam,
            material: Zeroizing::new(material.to_vec()),
        });
        if let Some(keys) = &mut self.progress.keys {
            bump(keys);
        }
        // mac80211 keeps the AP's station authorized across a rekey; the
        // station layer authorizes one PTK and GTK, which a replaced or
        // newer key leaves behind, so the port is authorized again on the
        // keys now held. The keeper installs a key only from an
        // authenticated handshake.
        if self.port_open {
            let epoch = self
                .station
                .association_epoch()
                .ok_or(Error::Refused("no association"))?;
            self.authorize(epoch)?;
        }
        Ok(identity)
    }

    /// The host's IGTK (spec S1.2), rtw88's `install_management_key`
    /// (`station.rs:2236`): BIP under the join's management cipher, beside
    /// the GTK, which must be installed first (`Pending` until then).
    fn install_integrity(
        &mut self,
        index: u8,
        material: &[u8],
        receive: PacketNumber,
    ) -> Result<Arc<ManagementKey>, Error> {
        let cipher = self
            .management_cipher
            .ok_or(Error::Refused("no management cipher"))?;
        let gtk = self
            .keys
            .iter()
            .rev()
            .find(|key| !key.pairwise)
            .and_then(|key| key.slot.installed())
            .ok_or(Error::Pending)?;
        self.integrity = Some(
            BipKey::new(gtk, cipher, index, material, receive)
                .map_err(|_| Error::Refused("the IGTK"))?,
        );
        Ok(Arc::new(ManagementKey))
    }

    /// rtw88's `authorize_port` (`station.rs:2271`): the PTK and the last
    /// GTK, under PMF with the association's protection.
    fn authorize(&mut self, epoch: u64) -> Result<(), Error> {
        let context = self.context.clone().ok_or(Error::Refused("no context"))?;
        let ptk = self
            .keys
            .iter()
            .find(|key| key.pairwise)
            .ok_or(Error::Refused("no pairwise key"))?;
        let gtk = self
            .keys
            .iter()
            .rev()
            .find(|key| !key.pairwise)
            .ok_or(Error::Refused("no group key"))?;
        let links = [(&ptk.slot, &gtk.slot)];
        if self.station.requires_protected_management() {
            self.station
                .authorize_protected_port(&context, epoch, &links)
        } else {
            self.station.authorize_port(epoch, &links)
        }
        .map_err(Error::Station)?;
        self.port_open = true;
        Ok(())
    }
}

impl<C: Chip> Link for Rtl8852c<C> {
    fn mac(&self) -> Mac {
        self.address
    }
    fn carrier(&mut self) -> bool {
        self.port_open
    }
    /// Spec S1.5, through the keeper.
    fn carrier_wait_us(&self) -> u64 {
        WIFI_CARRIER_WAIT_US
    }
    fn transmit(&mut self, frame: &[u8]) -> Result<(), LinkError> {
        if !self.port_open {
            return Err(LinkError::NoCarrier);
        }
        let sent = self.send(frame, TxClass::Data);
        self.note(sent)
            .map_err(|error| LinkError::Adapter(format!("{error}")))
    }
    fn receive(&mut self) -> Result<Option<Vec<u8>>, LinkError> {
        Ok(self.received.pop_front())
    }
    fn diagnostic(&mut self) -> Option<String> {
        Some(self.progress())
    }
}

impl<C: Chip> Radio for Rtl8852c<C> {
    type Handle = usize;
    type Profile = Peer;
    type Management = Arc<ManagementKey>;
    type Error = Error;

    const NAME: &'static str = "rtw89";
    /// rtw89 leaves `max_listen_interval` at mac80211's default, as rtw88.
    const LISTEN_INTERVAL: u16 = 5;

    fn deferred(error: &Error) -> bool {
        matches!(error, Error::Pending)
    }

    fn service(&mut self) -> Result<(), Error> {
        let served = self.operate();
        self.note(served)
    }

    fn take_event(&mut self) -> Option<keeper::Event> {
        self.events.pop_front()
    }

    fn stage(&self) -> Stage {
        if self.left {
            Stage::NeedsDrain
        } else if self.scanned.is_some() {
            Stage::Scanned
        } else if self.admitted.is_some() {
            Stage::Admitted
        } else if self.scanning.is_none() && self.station.phase() == Peering::Idle {
            Stage::Idle
        } else {
            Stage::Busy
        }
    }

    fn station(&self) -> &Station {
        &self.station
    }

    fn channels(&self, band: Band) -> Vec<(Channel, u32)> {
        self.channels
            .iter()
            .filter(|(channel, _)| channel.band == band)
            .copied()
            .collect()
    }

    /// Never: the firmware has no probe template (spec S1.3).
    fn probes(&self, _: Channel) -> bool {
        false
    }

    fn supported_channels(&self, band: Band) -> &'static [u8] {
        match band {
            Band::Ghz2 => &SUPPORTED_2GHZ,
            Band::Ghz5 => &SUPPORTED_5GHZ,
            Band::Ghz6 => &[],
        }
    }

    fn regulated_power_dbm(&self, frequency_mhz: u16) -> i32 {
        self.country.channel(frequency_mhz).max_reg_dbm
    }

    /// HT 20 MHz with two streams, no VHT or HE (spec S1.7), WMM.
    fn advertisement(&self, band: Band) -> Result<Advertisement, Error> {
        let mut elements = rate_elements(band).to_vec();
        elements.extend_from_slice(&HT_ELEMENT);
        elements.extend_from_slice(&EXTENDED_CAPABILITIES);
        elements.extend_from_slice(&WMM_ELEMENT);
        let capabilities = Capabilities::parse(band, Role::Station, &elements)
            .map_err(|_| Error::Refused("advertisement"))?;
        Ok(Advertisement {
            capabilities,
            elements,
            he_source_maps: [McsMaps {
                rx: 0xffff,
                tx: 0xffff,
            }; 3],
            multi_link: Some(false),
        })
    }

    /// The BSS's profile, kept for its basic rates (spec S1.9).
    fn profile(&self, peer: Peer) -> Result<Peer, Error> {
        Ok(peer)
    }

    /// rtw88's `begin_scan` (`station.rs:1968`) as one firmware offload of
    /// the request's channels, which the chip bounds by its own budget.
    fn begin_scan(&mut self, request: scan::Request, budget_us: u64) -> Result<(), Error> {
        let begun = self.begin_scan_with(request, budget_us);
        self.note(begun)
    }

    /// The scan's end handed on every frame it heard, and the chip was
    /// restored: retirement is confirmed.
    unsafe fn retire_scan(&mut self) -> Result<(), Error> {
        let ticket = self.scanned.take().ok_or(Error::Refused("scan"));
        let ticket = self.note(ticket)?;
        self.scanner
            .retire(ticket, Retirement::Confirmed)
            .map_err(Error::Scan)?;
        Ok(())
    }

    fn six_ghz_scan(
        &self,
        _: &scan::Request,
        _: scan::six_ghz::Options,
    ) -> Result<Option<scan::Request>, Error> {
        Ok(None)
    }

    fn bss(&self) -> Vec<(usize, &Bss)> {
        self.observed
            .iter()
            .enumerate()
            .map(|(handle, heard)| (handle, heard.observation.bss()))
            .collect()
    }

    fn observation(&self, handle: usize) -> Result<Arc<NativeBssObservation>, Error> {
        self.observed(handle)
    }

    /// rtw88's `join` (`station.rs:2022`): the channel and BSSID, then
    /// authentication.
    fn join(&mut self, handle: usize, join: Join, peer: Peer) -> Result<(), Error> {
        let joined = self.join_with(handle, join, peer);
        self.note(joined)
    }

    fn begin_external_authentication(
        &mut self,
        _: usize,
        _: Peer,
        _: Option<Affiliation>,
        _: u64,
    ) -> Result<Arc<external::Session>, Error> {
        self.note(Err(Error::Unsupported(SAE_BEFORE_M3B)))
    }

    fn begin_six_ghz_external_authentication(
        &mut self,
        _: usize,
        _: Selection,
        _: Peer,
        _: Option<Affiliation>,
        _: u64,
    ) -> Result<Arc<external::Session>, Error> {
        self.note(Err(Error::Unsupported("6 GHz on an RTL8852C")))
    }

    fn transmit_external_authentication(
        &mut self,
        _: &external::Session,
        _: external::Frame<'_>,
    ) -> Result<(), Error> {
        self.note(Err(Error::Unsupported(SAE_BEFORE_M3B)))
    }

    fn associate_external(
        &mut self,
        _: &external::Session,
        _: usize,
        _: Join,
    ) -> Result<(), Error> {
        self.note(Err(Error::Unsupported(SAE_BEFORE_M3B)))
    }

    fn configure(&mut self) -> Result<(), Error> {
        let configured = self.configure_admitted();
        self.note(configured)
    }

    fn send_control(&mut self, ethernet: &[u8], key: Option<KeyIdentity>) -> Result<(), Error> {
        let class = key.map_or(TxClass::ControlPortPlaintext, TxClass::ControlPortProtected);
        let sent = self.send(ethernet, class);
        self.note(sent)
    }

    /// `rtw89_ops_set_key` of CCMP-128 alone (spec S1.2), into the chip's
    /// security CAM (its `install`).
    fn install_key(
        &mut self,
        pairwise: bool,
        index: u8,
        material: &[u8],
        receive: ReplaySeed,
    ) -> Result<KeyIdentity, Error> {
        let installed = self.install(pairwise, index, material, receive);
        self.note(installed)
    }

    fn install_link_key(&mut self, _: u8, _: u8, _: &[u8], _: ReplaySeed) -> Result<(), Error> {
        self.note(Err(Error::Unsupported("MLO on an RTL8852C")))
    }

    /// The IGTK stays on the host, where mac80211 runs BIP (spec S1.2).
    unsafe fn install_management_key(
        &mut self,
        _: u64,
        index: u8,
        material: &[u8],
        receive: PacketNumber,
    ) -> Result<Arc<ManagementKey>, Error> {
        let installed = self.install_integrity(index, material, receive);
        self.note(installed)
    }

    /// A rekeyed IGTK replaces the host's.
    unsafe fn rotate_management_key(
        &mut self,
        _: &Arc<ManagementKey>,
        index: u8,
        material: &[u8],
        receive: PacketNumber,
    ) -> Result<Arc<ManagementKey>, Error> {
        let installed = self.install_integrity(index, material, receive);
        self.note(installed)
    }

    /// The port opens on an installed PTK and GTK.
    fn authorize_port(&mut self, epoch: u64) -> Result<(), Error> {
        let authorized = self.authorize(epoch);
        self.note(authorized)
    }

    fn deauthenticate(&mut self, reason: u16) -> Result<(), Error> {
        let left = self.deauthenticate_with(reason);
        self.note(left)
    }

    unsafe fn retire_peer(&mut self) -> Result<(), Error> {
        let retired = self.retire();
        self.note(retired)
    }
}

#[cfg(test)]
mod tests;
