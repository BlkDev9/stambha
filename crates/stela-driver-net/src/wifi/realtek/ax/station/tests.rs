// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
// Copyright (C) 2026 Stela contributors
//! The rtw89 radio through the keeper, over a model of the chip that plays
//! the AP, shaped as rtw88's `station/tests.rs` `Air`: a firmware scan
//! hears the AP's beacon on its channel and ends, the AP answers
//! authentication and association, and EAPOL runs against the supplicant
//! tests' `Authenticator`. The model decrypts nothing: it records the keys
//! the station installs and the key each data frame names, and hands back
//! the AP's protected frames as the hardware leaves them, marked decrypted
//! with their CCMP header and MIC in place.
use super::chip::{Association, Chip, Data, HardwareKey, Incoming, Management, Received, Token};
use super::*;
use crate::LinkError;
use crate::wifi::connection::security::Security;
use crate::wifi::intel::policy::tests::regdb;
use crate::wifi::network::{Network, Secret};
use crate::wifi::realtek::ccmp;
use crate::wifi::station::phy::AccessCategory;
use crate::wifi::supplicant::tests::Authenticator;
use alloc::{rc::Rc, vec};
use core::cell::RefCell;
use core::sync::atomic::{AtomicU8, Ordering};
use stela_driver_base::entropy::EntropyError;
use zeroize::Zeroizing;

const CLOCK: Clock = Clock {
    ticks_per_microsecond: 1,
};
/// The plan's test addresses.
const LOCAL: [u8; 6] = [0x02, 0x11, 0x22, 0x33, 0x44, 0x55];
const AP: [u8; 6] = [0x02, 0x03, 0x04, 0x05, 0x06, 0x07];
const SSID: &[u8] = b"test";
const RSN: [u8; 22] = [
    48, 20, 1, 0, 0, 0x0f, 0xac, 4, 1, 0, 0, 0x0f, 0xac, 4, 1, 0, 0, 0x0f, 0xac, 2, 0, 0,
];
/// [`RSN`] with management frame protection capable (MFPC, b7), so the
/// station protects its management frames (`ieee80211w=1`).
const RSN_PMF: [u8; 22] = [
    48, 20, 1, 0, 0, 0x0f, 0xac, 4, 1, 0, 0, 0x0f, 0xac, 4, 1, 0, 0, 0x0f, 0xac, 2, 0x80, 0,
];
/// The IGTK KDE a PMF message 3 carries: key ID 4, IPN 9, the key 0x77s.
const IGTK_KDE: [u8; 30] = [
    0xdd, 28, 0, 0x0f, 0xac, 9, 4, 0, 9, 0, 0, 0, 0, 0, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77,
    0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77,
];
/// An AP's HT capabilities (SGI 20, one RX STBC stream, SM power save
/// static) and operation, one stream at 20 MHz.
const HT: [u8; 52] = [
    45, 26, 0x2c, 1, 0x17, 0xff, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    61, 22, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
];
/// A WMM parameter element: best effort at the defaults of each class.
const WMM: [u8; 26] = [
    221, 24, 0, 0x50, 0xf2, 2, 1, 1, 0, 0, 3, 0xa4, 0, 0, 0x27, 0xa4, 0, 0, 0x42, 0x43, 0x5e, 0,
    0x62, 0x32, 0x2f, 0,
];
/// A US-like country: 2.4 GHz channels 1-11, and 5 GHz 36-64, 100-144 and
/// 149-165.
const US: [(u32, u32, u8, u16); 5] = [
    (2_402_000, 2_472_000, 0, 3000),
    (5_170_000, 5_250_000, 0, 2300),
    (5_250_000, 5_330_000, 0, 2300),
    (5_490_000, 5_730_000, 0, 2300),
    (5_735_000, 5_835_000, 0, 3000),
];

/// Each read one byte over the last, from 0x5a.
#[derive(Default)]
struct FixedEntropy(AtomicU8);

impl Entropy for FixedEntropy {
    fn fill(&self, bytes: &mut [u8]) -> Result<(), EntropyError> {
        bytes.fill(0x5a + self.0.fetch_add(1, Ordering::Relaxed));
        Ok(())
    }
}

/// An association as the chip was handed it.
#[derive(Clone, Debug, PartialEq, Eq)]
struct Joined {
    bssid: Mac,
    aid: u16,
    channel: Channel,
    beacon_interval: u16,
    dtim_period: u8,
    legacy: u16,
    ht: Option<HtPeer>,
    edca: Option<[AccessCategory; 4]>,
    short_slot: bool,
}

/// The CAM entry, key ID and PN a data frame goes under, if any.
type FrameKey = Option<(u8, u8, u64)>;

/// One call the chip took, in order.
#[derive(Clone, Debug, PartialEq, Eq)]
enum Call {
    Tune(Channel),
    BeginScan(Vec<Channel>),
    EndScan,
    SetBssid(Mac),
    Authenticated,
    /// A management frame and the basic rates it went with.
    Management(Vec<u8>, u16),
    /// A data frame, with the CAM entry, key ID and PN it went under.
    Data {
        bytes: Vec<u8>,
        tid: u8,
        eapol: bool,
        supported: u16,
        key: FrameKey,
    },
    Associate(Joined),
    Disassociate,
    /// A key the chip installed, in CAM entry `cam`.
    AddKey {
        pairwise: bool,
        index: u8,
        material: Vec<u8>,
        cam: u8,
    },
    RemoveKey(u8),
    /// mac80211's flush, with drop or without.
    Flush(bool),
}

/// What the chip was told, and what it holds for the host.
#[derive(Default)]
struct Air {
    /// The AP's channel.
    channel: Option<Channel>,
    calls: Vec<Call>,
    incoming: VecDeque<Incoming>,
    /// The channels of a scan begun and not yet ended.
    scanning: Option<Vec<Channel>>,
    next_token: u16,
    /// Frames the chip took that the AP has yet to answer.
    unanswered: VecDeque<(Token, Vec<u8>)>,
    /// The AP neither acknowledges nor answers an authentication.
    deaf_to_auth: bool,
    /// The AP acknowledges an association request but never answers.
    mute_on_assoc: bool,
    /// The AP has no WMM.
    no_wmm: bool,
    /// The scan's beacon fails its FCS.
    bad_fcs: bool,
    /// The chip has no room for management or for data frames.
    full_management: bool,
    full_data: bool,
    /// Frames refused for no room.
    refused: usize,
    /// The AP protects management frames (beacons [`RSN_PMF`]).
    pmf: bool,
    /// The firmware never acknowledges a key.
    refuse_keys: bool,
    /// The CAM entries holding a key.
    cam: Vec<u8>,
    /// An authentication's report comes only after the association
    /// request has been taken, as the M3a boot saw it.
    late_auth_report: bool,
    /// Reports held for [`Air::late_auth_report`].
    late: Vec<Incoming>,
}

impl Air {
    fn hear(&mut self, bytes: Vec<u8>) {
        self.hear_as(bytes, false, false);
    }

    /// A frame the hardware `decrypted`, or found failing its ICV.
    fn hear_as(&mut self, bytes: Vec<u8>, decrypted: bool, icv_error: bool) {
        let channel = self.channel.unwrap();
        self.incoming.push_back(Incoming::Frame(Received {
            bytes,
            channel,
            signal_dbm: None,
            tsf: 0,
            decrypted,
            icv_error,
            fcs_error: false,
        }));
    }

    /// Each frame's release report, and the AP's answer to it.
    fn serve(&mut self) {
        while let Some((token, bytes)) = self.unanswered.pop_front() {
            let deaf = bytes[0] == 0xb0 && self.deaf_to_auth;
            let report = Incoming::Transmitted {
                token,
                acknowledged: !deaf,
            };
            if bytes[0] == 0x00 {
                let late = core::mem::take(&mut self.late);
                self.incoming.extend(late);
            }
            if bytes[0] == 0xb0 && self.late_auth_report {
                self.late.push(report);
            } else {
                self.incoming.push_back(report);
            }
            if deaf {
                continue;
            }
            match bytes[0] {
                0xb0 => self.hear(management(0xb0, &[0, 0, 2, 0, 0, 0])),
                0x00 if !self.mute_on_assoc => {
                    let mut body = vec![0x11, 0, 0, 0, 1, 0xc0];
                    body.extend(RATES);
                    body.extend(HT);
                    if !self.no_wmm {
                        body.extend(WMM);
                    }
                    self.hear(management(0x10, &body));
                }
                _ => {}
            }
        }
    }

    fn token(&mut self) -> Token {
        self.next_token += 1;
        Token(self.next_token)
    }

    fn position(&self, want: impl FnMut(&Call) -> bool) -> Option<usize> {
        self.calls.iter().position(want)
    }

    /// The EAPOL bodies the station sent, past their LLC.
    fn eapol(&self) -> Vec<Vec<u8>> {
        self.calls
            .iter()
            .filter_map(|c| match c {
                Call::Data { bytes, .. } => eapol(bytes).map(<[u8]>::to_vec),
                _ => None,
            })
            .collect()
    }

    /// The data frames the chip took, with the key each named.
    fn data(&self) -> Vec<(Vec<u8>, FrameKey)> {
        self.calls
            .iter()
            .filter_map(|c| match c {
                Call::Data { bytes, key, .. } => Some((bytes.clone(), *key)),
                _ => None,
            })
            .collect()
    }

    /// The keys installed, as (pairwise, index, CAM entry).
    fn keys(&self) -> Vec<(bool, u8, u8)> {
        self.calls
            .iter()
            .filter_map(|c| match c {
                Call::AddKey {
                    pairwise,
                    index,
                    cam,
                    ..
                } => Some((*pairwise, *index, *cam)),
                _ => None,
            })
            .collect()
    }

    /// The pairwise key's material.
    fn ptk(&self) -> [u8; 16] {
        self.calls
            .iter()
            .find_map(|c| match c {
                Call::AddKey {
                    pairwise: true,
                    material,
                    ..
                } => material[..].try_into().ok(),
                _ => None,
            })
            .unwrap()
    }

    /// The management frames of subtype `fc0` the chip took.
    fn management(&self, fc0: u8) -> Vec<Vec<u8>> {
        self.calls
            .iter()
            .filter_map(|c| match c {
                Call::Management(bytes, _) if bytes[0] == fc0 => Some(bytes.clone()),
                _ => None,
            })
            .collect()
    }

    fn count(&self, want: impl Fn(&Call) -> bool) -> usize {
        self.calls.iter().filter(|c| want(c)).count()
    }
}

#[derive(Clone)]
struct Model(Rc<RefCell<Air>>);

impl Chip for Model {
    fn mac(&self) -> Mac {
        Mac(LOCAL)
    }
    fn poll(&mut self, sink: &mut dyn FnMut(Incoming)) -> Result<(), String> {
        let mut air = self.0.borrow_mut();
        if let Some(channels) = air.scanning.take() {
            let channel = air.channel.unwrap();
            if channels.contains(&channel) {
                let bad = air.bad_fcs;
                let bytes = beacon(&air);
                air.incoming.push_back(Incoming::Frame(Received {
                    bytes,
                    channel,
                    signal_dbm: Some(-40),
                    tsf: 0,
                    decrypted: false,
                    icv_error: false,
                    fcs_error: bad,
                }));
            }
            air.incoming
                .push_back(Incoming::ScanEnded { completed: true });
        }
        air.incoming.drain(..).for_each(sink);
        Ok(())
    }
    fn tune(&mut self, channel: Channel) -> Result<(), String> {
        self.0.borrow_mut().calls.push(Call::Tune(channel));
        Ok(())
    }
    fn begin_scan(&mut self, channels: &[Channel]) -> Result<(), String> {
        let mut air = self.0.borrow_mut();
        air.calls.push(Call::BeginScan(channels.to_vec()));
        air.scanning = Some(channels.to_vec());
        Ok(())
    }
    fn end_scan(&mut self) -> Result<(), String> {
        self.0.borrow_mut().calls.push(Call::EndScan);
        Ok(())
    }
    fn set_bssid(&mut self, bssid: Mac) -> Result<(), String> {
        self.0.borrow_mut().calls.push(Call::SetBssid(bssid));
        Ok(())
    }
    fn authenticated(&mut self) -> Result<(), String> {
        self.0.borrow_mut().calls.push(Call::Authenticated);
        Ok(())
    }
    fn transmit_management(&mut self, frame: &Management<'_>) -> Result<Option<Token>, String> {
        let mut air = self.0.borrow_mut();
        if air.full_management {
            air.refused += 1;
            return Ok(None);
        }
        air.calls
            .push(Call::Management(frame.bytes.to_vec(), frame.basic_rates));
        let token = air.token();
        air.unanswered.push_back((token, frame.bytes.to_vec()));
        Ok(Some(token))
    }
    fn transmit_data(&mut self, frame: &Data<'_>) -> Result<Option<Token>, String> {
        let mut air = self.0.borrow_mut();
        if air.full_data {
            air.refused += 1;
            return Ok(None);
        }
        if let Some((cam, _, _)) = frame.key
            && !air.cam.contains(&cam)
        {
            return Err("a key the CAM does not hold".into());
        }
        air.calls.push(Call::Data {
            bytes: frame.bytes.to_vec(),
            tid: frame.tid,
            eapol: frame.eapol,
            supported: frame.supported_rates,
            key: frame.key,
        });
        let token = air.token();
        air.unanswered.push_back((token, frame.bytes.to_vec()));
        Ok(Some(token))
    }
    fn associate(&mut self, association: &Association) -> Result<(), String> {
        self.0.borrow_mut().calls.push(Call::Associate(Joined {
            bssid: association.bssid,
            aid: association.aid,
            channel: association.channel,
            beacon_interval: association.beacon_interval,
            dtim_period: association.dtim_period,
            legacy: association.legacy,
            ht: association.ht,
            edca: association.edca,
            short_slot: association.short_slot,
        }));
        Ok(())
    }
    /// Leaving deletes what keys are left, as `Live::leave` does.
    fn disassociate(&mut self) -> Result<(), String> {
        let mut air = self.0.borrow_mut();
        air.calls.push(Call::Disassociate);
        air.cam.clear();
        Ok(())
    }
    /// The lowest free CAM entry, unless the firmware never acknowledges.
    fn add_key(&mut self, key: &HardwareKey<'_>) -> Result<u8, String> {
        let mut air = self.0.borrow_mut();
        if air.refuse_keys {
            return Err("the firmware never acknowledged the key".into());
        }
        let cam = (0..).find(|cam| !air.cam.contains(cam)).unwrap();
        air.cam.push(cam);
        air.calls.push(Call::AddKey {
            pairwise: key.pairwise,
            index: key.index,
            material: key.material.to_vec(),
            cam,
        });
        Ok(cam)
    }
    fn remove_key(&mut self, cam: u8) -> Result<(), String> {
        let mut air = self.0.borrow_mut();
        air.calls.push(Call::RemoveKey(cam));
        let at = air
            .cam
            .iter()
            .position(|&held| held == cam)
            .ok_or("no key in that entry")?;
        air.cam.remove(at);
        Ok(())
    }
    fn flush(&mut self, drop: bool) -> Result<(), String> {
        self.0.borrow_mut().calls.push(Call::Flush(drop));
        Ok(())
    }
    fn start_rx_block_ack(&mut self, _: u8, _: u16) -> Result<(), String> {
        Ok(())
    }
    fn stop_rx_block_ack(&mut self, _: u8) -> Result<(), String> {
        Ok(())
    }
    fn diagnostic(&self) -> String {
        "chip".into()
    }
}

/// The AP's rates on 2.4 GHz: 1 Mb/s (basic) and 6 Mb/s.
const RATES: [u8; 4] = [1, 2, 0x82, 0x0c];

/// An AP's management frame to the station.
fn management(subtype: u8, body: &[u8]) -> Vec<u8> {
    let mut frame = vec![0; 24];
    frame[0] = subtype;
    frame[4..10].copy_from_slice(&LOCAL);
    frame[10..16].copy_from_slice(&AP);
    frame[16..22].copy_from_slice(&AP);
    frame.extend(body);
    frame
}

/// The AP's beacon on its channel.
fn beacon(air: &Air) -> Vec<u8> {
    let channel = air.channel.unwrap().number as u8;
    let mut frame = vec![0; 36];
    frame[0] = 0x80;
    frame[4..10].fill(0xff);
    frame[10..16].copy_from_slice(&AP);
    frame[16..22].copy_from_slice(&AP);
    frame[32..34].copy_from_slice(&100u16.to_le_bytes());
    frame[34] = 0x11;
    frame.extend([0, SSID.len() as u8]);
    frame.extend(SSID);
    frame.extend(RATES);
    // DS parameter set, then a TIM with DTIM period 2.
    frame.extend([3, 1, channel, 5, 4, 0, 2, 0, 0]);
    frame.extend(if air.pmf { RSN_PMF } else { RSN });
    frame.extend(HT);
    // HT Operation's primary channel.
    let at = frame.len() - 22;
    frame[at] = channel;
    if !air.no_wmm {
        frame.extend(WMM);
    }
    frame
}

/// A data frame from the AP to `receiver`, without protection.
fn data(receiver: [u8; 6], sequence: u16, kind: [u8; 2], body: &[u8]) -> Vec<u8> {
    let mut frame = vec![0; 26];
    frame[0] = 0x88;
    frame[1] = 0x02;
    frame[4..10].copy_from_slice(&receiver);
    frame[10..16].copy_from_slice(&AP);
    frame[16..22].copy_from_slice(&AP);
    frame[22..24].copy_from_slice(&(sequence << 4).to_le_bytes());
    frame.extend([0xaa, 0xaa, 3, 0, 0, 0]);
    frame.extend(kind);
    frame.extend(body);
    frame
}

/// [`data`] protected under key ID `keyid` at packet number `pn`, as the
/// hardware hands it on decrypted: its CCMP header after the MAC header,
/// the plaintext, then the MIC (zeros here).
fn protected(receiver: [u8; 6], sequence: u16, keyid: u8, pn: u64, body: &[u8]) -> Vec<u8> {
    let mut frame = data(receiver, sequence, [8, 0], body);
    frame[1] |= 0x40;
    let [p0, p1, p2, p3, p4, p5, _, _] = pn.to_le_bytes();
    let header = [p0, p1, 0, 0x20 | keyid << 6, p2, p3, p4, p5];
    frame.splice(26..26, header);
    frame.extend([0; 8]);
    frame
}

/// An IPv4 frame from the station to the AP, its body `fill`.
fn ip(fill: u8) -> Vec<u8> {
    let mut ethernet = Vec::from(AP);
    ethernet.extend(LOCAL);
    ethernet.extend([8, 0, 0x45, 0]);
    ethernet.extend([fill; 44]);
    ethernet
}

/// What the station holds for the network, as each frame's first payload
/// byte past its IPv4 type.
fn delivered(wifi: &mut Wifi<Model>) -> Vec<u8> {
    let mut values = Vec::new();
    while let Some(ethernet) = wifi.receive().unwrap() {
        values.push(ethernet[14]);
    }
    values
}

/// What the station sent past `frame`'s header and LLC when it is EAPOL.
fn eapol(frame: &[u8]) -> Option<&[u8]> {
    if frame[0] & 0x0c != 8 {
        return None;
    }
    let header = if frame[0] & 0x80 != 0 { 26 } else { 24 };
    (frame.get(header + 6..header + 8)? == [0x88, 0x8e]).then(|| &frame[header + 8..])
}

fn channel(number: u16) -> Channel {
    let (band, frequency_mhz) = if number <= 14 {
        (Band::Ghz2, 2407 + 5 * number)
    } else {
        (Band::Ghz5, 5000 + 5 * number)
    };
    Channel {
        band,
        number,
        frequency_mhz,
    }
}

/// The radio kept by a keeper over a model whose AP sits on 2.4 GHz
/// channel 1, under [`US`].
fn wifi() -> (Wifi<Model>, Model) {
    wifi_with(|_| {})
}

/// [`wifi`], with the model set up by `set` first.
fn wifi_with(set: impl FnOnce(&mut Air)) -> (Wifi<Model>, Model) {
    core::mem::forget(crate::test_clock::virtualize());
    let database = regdb(&[(*b"US", &US[..])]);
    let mut air = Air {
        channel: Some(channel(1)),
        ..Air::default()
    };
    set(&mut air);
    let model = Model(Rc::new(RefCell::new(air)));
    let country = database.select(*b"US").unwrap();
    let network = Network {
        ssid: SSID.to_vec(),
        secret: Secret::Psk(Zeroizing::new([0; 32])),
        country: None,
    };
    let radio = Rtl8852c::new(model.clone(), Mac(LOCAL), country, CLOCK).unwrap();
    (
        radio.keep(network, Arc::new(FixedEntropy::default())),
        model,
    )
}

/// Serve the air until `done`, with five milliseconds passing each round.
fn run(
    wifi: &mut Wifi<Model>,
    model: &Model,
    mut done: impl FnMut(&mut Wifi<Model>, &Air) -> bool,
) -> bool {
    for _ in 0..4000 {
        wifi.carrier();
        model.0.borrow_mut().serve();
        if done(wifi, &model.0.borrow()) {
            return true;
        }
        crate::test_clock::advance(CLOCK, 5_000);
    }
    false
}

/// The radio's progress line.
fn progress(wifi: &mut Wifi<Model>) -> String {
    wifi.diagnostic().unwrap()
}

/// Join the model's AP up to the association, its configuration done.
fn associated() -> (Wifi<Model>, Model) {
    let (mut wifi, model) = wifi();
    assert!(run(&mut wifi, &model, |wifi, _| {
        wifi.radio().station().phase() == Peering::Associated
    }));
    (wifi, model)
}

/// The AP's half of the 4-way handshake.
fn authenticator() -> Authenticator {
    authenticator_for(&RSN)
}

/// The AP's half of the 4-way handshake under `rsn`.
fn authenticator_for(rsn: &[u8]) -> Authenticator {
    let security = Security::select(rsn, true).unwrap();
    Authenticator::between(
        security.suites(),
        Zeroizing::new([0; 32]),
        Mac(AP),
        Mac(LOCAL),
        security.element(),
    )
}

/// [`associated`], then messages 1 to 4 of the 4-way handshake, m4's
/// release report included.
fn handshaken() -> (Wifi<Model>, Model) {
    let (mut wifi, model) = associated();
    shake(&mut wifi, &model, &RSN, &[]);
    (wifi, model)
}

/// Messages 1 to 4 of the 4-way handshake under `rsn`, message 3 carrying
/// `extra` KDEs, m4's release report included.
fn shake(wifi: &mut Wifi<Model>, model: &Model, rsn: &[u8], extra: &[u8]) {
    let mut ap = authenticator_for(rsn);
    model
        .0
        .borrow_mut()
        .hear(data(LOCAL, 0, [0x88, 0x8e], &ap.message_1()));
    assert!(run(wifi, model, |_, air| !air.eapol().is_empty()));
    ap.message_2(&model.0.borrow().eapol()[0]);
    model
        .0
        .borrow_mut()
        .hear(data(LOCAL, 1, [0x88, 0x8e], &ap.message_3_with(rsn, extra)));
    assert!(run(wifi, model, |wifi, air| {
        air.eapol().len() > 1 && progress(wifi).contains("acked 2")
    }));
}

/// [`handshaken`], the port open.
fn connected() -> (Wifi<Model>, Model) {
    let (mut wifi, model) = handshaken();
    assert!(wifi.carrier(), "{}", progress(&mut wifi));
    (wifi, model)
}

/// A join to an AP protecting management frames, through its handshake
/// with an IGTK to an open port.
fn connected_with_pmf() -> (Wifi<Model>, Model) {
    let (mut wifi, model) = wifi_with(|air| air.pmf = true);
    assert!(run(&mut wifi, &model, |wifi, _| {
        wifi.radio().station().phase() == Peering::Associated
    }));
    assert!(wifi.radio().station().requires_protected_management());
    shake(&mut wifi, &model, &RSN_PMF, &IGTK_KDE);
    assert!(wifi.carrier(), "{}", progress(&mut wifi));
    (wifi, model)
}

/// Whether `text` holds six `:`-separated pairs of hex digits.
fn names_an_address(text: &str) -> bool {
    let bytes = text.as_bytes();
    bytes.windows(17).any(|w| {
        w.chunks(3).enumerate().all(|(i, part)| {
            part[..2].iter().all(u8::is_ascii_hexdigit) && (i == 5 || part[2] == b':')
        })
    })
}

#[test]
fn a_scan_is_one_passive_offload_of_the_countrys_channels() {
    let (mut wifi, model) = wifi();
    assert!(run(&mut wifi, &model, |_, air| {
        air.position(|c| matches!(c, Call::SetBssid(_))).is_some()
    }));
    let air = model.0.borrow();
    let scans: Vec<&Vec<Channel>> = air
        .calls
        .iter()
        .filter_map(|c| match c {
            Call::BeginScan(channels) => Some(channels),
            _ => None,
        })
        .collect();
    // One offload of every channel the country leaves, 2.4 GHz first.
    let mut want: Vec<u16> = (1..=11).collect();
    want.extend((36..=64).step_by(4));
    want.extend((100..=144).step_by(4));
    want.extend((149..=165).step_by(4));
    let want: Vec<Channel> = want.into_iter().map(channel).collect();
    assert_eq!(scans, [&want]);
    // Each passive: no probe templates (spec S1.3).
    assert!(want.iter().all(|&c| !wifi.radio().probes(c)));
    // Its end restored the chip before the keeper retired it and joined.
    let begun = air.position(|c| matches!(c, Call::BeginScan(_))).unwrap();
    let ended = air.position(|c| *c == Call::EndScan).unwrap();
    let joined = air.position(|c| matches!(c, Call::Tune(_))).unwrap();
    assert!(begun < ended && ended < joined);
    assert_eq!(air.count(|c| *c == Call::EndScan), 1);
    assert_ne!(wifi.radio().stage(), Stage::Scanned);
}

#[test]
fn the_join_tunes_sets_the_bssid_and_authenticates() {
    let (mut wifi, model) = wifi();
    assert!(run(&mut wifi, &model, |wifi, _| {
        wifi.radio().station().phase() == Peering::Associating
    }));
    let air = model.0.borrow();
    let tune = air.position(|c| *c == Call::Tune(channel(1))).unwrap();
    let bssid = air.position(|c| *c == Call::SetBssid(Mac(AP))).unwrap();
    let auth = air
        .position(|c| matches!(c, Call::Management(b, _) if b[0] == 0xb0))
        .unwrap();
    assert!(tune < bssid && bssid < auth);
    // Open system, transaction 1, to the AP, at the BSS's basic rates
    // (1 Mb/s, bit 0).
    let Call::Management(frame, basic) = &air.calls[auth] else {
        unreachable!()
    };
    assert_eq!(frame[4..10], AP);
    assert_eq!(frame[24..28], [0, 0, 1, 0]);
    assert_eq!(*basic, 0x001);
    // Its token's ack and the AP's answer moved the station on: the chip
    // heard of the AP's station authenticated, then the association
    // request left.
    let authenticated = air.position(|c| *c == Call::Authenticated).unwrap();
    let request = air
        .position(|c| matches!(c, Call::Management(b, _) if b[0] == 0x00))
        .unwrap();
    assert!(auth < authenticated && authenticated < request);
    drop(air);
    assert!(progress(&mut wifi).contains("auth 1/1 status 0"));
}

#[test]
fn association_configures_the_chip_with_the_negotiated_peer() {
    let (_wifi, model) = associated();
    let air = model.0.borrow();
    let at = air.position(|c| matches!(c, Call::Associate(_))).unwrap();
    let Call::Associate(joined) = &air.calls[at] else {
        unreachable!()
    };
    let ac = |aci, aifsn, ecw_min, ecw_max, txop_32us| AccessCategory {
        aci,
        admission_control: false,
        aifsn,
        ecw_min,
        ecw_max,
        txop_32us,
    };
    assert_eq!(
        *joined,
        Joined {
            bssid: Mac(AP),
            aid: 1,
            channel: channel(1),
            beacon_interval: 100,
            dtim_period: 2,
            // 1 and 6 Mb/s.
            legacy: 0x011,
            // The AP's one stream, its SGI 20 and RX STBC, under ours.
            ht: Some(HtPeer {
                rx_mask: [0xff, 0, 0, 0],
                ldpc: false,
                rx_stbc: true,
                sgi20: true,
            }),
            // The AP's WMM parameters, in ACI order.
            edca: Some([
                ac(0, 3, 4, 10, 0),
                ac(1, 7, 4, 10, 0),
                ac(2, 2, 3, 4, 94),
                ac(3, 2, 2, 3, 47),
            ]),
            // The response's capability has no short slot on 2.4 GHz.
            short_slot: false,
        }
    );
    // The join's switch stands: nothing tuned between the response and
    // the association (Chip::associate switches only off its channel).
    let response = air
        .position(|c| matches!(c, Call::Management(b, _) if b[0] == 0x00))
        .unwrap();
    assert!(
        !air.calls[response..at]
            .iter()
            .any(|c| matches!(c, Call::Tune(_)))
    );
}

#[test]
fn an_ap_without_wmm_leaves_its_edca_to_the_chip() {
    // `Negotiated::edca` would hand the chip the no-QoS defaults as the
    // AP's own; the chip programs mac80211's QoS defaults for None.
    let (mut wifi, model) = wifi_with(|air| air.no_wmm = true);
    assert!(run(&mut wifi, &model, |_, air| {
        air.position(|c| matches!(c, Call::Associate(_))).is_some()
    }));
    let air = model.0.borrow();
    let joined = air
        .calls
        .iter()
        .find_map(|c| match c {
            Call::Associate(joined) => Some(joined),
            _ => None,
        })
        .unwrap();
    assert_eq!(joined.edca, None);
}

#[test]
fn eapol_flows_both_ways_before_any_key() {
    let (mut wifi, model) = handshaken();
    let air = model.0.borrow();
    // m2 and m4 went as EAPOL on TID 7, without a key, at the peer's rates:
    // m4 was handed to the chip before wpa_supplicant installs the PTK.
    let sent: Vec<(u8, bool, u16, FrameKey)> = air
        .calls
        .iter()
        .filter_map(|c| match c {
            Call::Data {
                bytes,
                tid,
                eapol: is_eapol,
                supported,
                key,
            } if eapol(bytes).is_some() => Some((*tid, *is_eapol, *supported, *key)),
            _ => None,
        })
        .collect();
    assert_eq!(sent, [(7, true, 0x011, None), (7, true, 0x011, None)]);
    let data: Vec<usize> = (0..air.calls.len())
        .filter(|&at| matches!(air.calls[at], Call::Data { .. }))
        .collect();
    let m4 = data[1];
    let ptk = air.position(|c| matches!(c, Call::AddKey { .. })).unwrap();
    assert!(m4 < ptk);
    drop(air);
    let line = progress(&mut wifi);
    assert!(line.contains("eapol rx 2 tx 2 acked 2"), "{line}");
}

#[test]
fn the_handshake_installs_ptk_then_gtk_and_opens_the_port() {
    let (mut wifi, model) = connected();
    // `rtw89_ops_set_key` for the PTK (entry 0), then the GTK of key ID 1
    // (entry 1), each with its own material (spec S12).
    let air = model.0.borrow();
    assert_eq!(air.keys(), [(true, 0, 0), (false, 1, 1)]);
    let gtk: Vec<u8> = (0x50..0x60).collect();
    assert!(air.calls.iter().any(|c| matches!(
        c,
        Call::AddKey { pairwise: false, material, .. } if *material == gtk
    )));
    drop(air);
    assert!(wifi.carrier());
    let line = progress(&mut wifi);
    assert!(line.contains("keys 2, port open"), "{line}");
    assert!(!line.contains("last"), "{line}");
    assert_eq!(wifi.receive(), Ok(None));
}

#[test]
fn unacked_authentication_is_retried_and_reported() {
    let (mut wifi, model) = wifi_with(|air| air.deaf_to_auth = true);
    let mut lines = Vec::new();
    let mut gave_up = None;
    let mut joins = Vec::new();
    assert!(run(&mut wifi, &model, |wifi, air| {
        lines.push(progress(wifi));
        let set = air.count(|c| matches!(c, Call::SetBssid(_)));
        if joins.len() < set {
            joins.push(CLOCK.now_us());
        }
        if gave_up.is_none() && air.count(|c| *c == Call::Disassociate) == 1 {
            gave_up = Some(CLOCK.now_us());
        }
        set == 2 && air.management(0xb0).len() == 4
    }));
    // Sent and never acknowledged: auth 1/0, then mac80211's retries at once.
    assert!(lines.iter().any(|l| l.contains("auth 1/0 status -")));
    assert!(lines.iter().any(|l| l.contains("auth 3/0 status -")));
    // The third ended the join; the keeper waited wpa_supplicant's 10 s and
    // joined again.
    assert!(joins[1] - gave_up.unwrap() >= 10_000_000);
    let air = model.0.borrow();
    let first = air.position(|c| matches!(c, Call::SetBssid(_))).unwrap();
    let left = air.position(|c| *c == Call::Disassociate).unwrap();
    let auths = air.calls[first..left]
        .iter()
        .filter(|c| matches!(c, Call::Management(b, _) if b[0] == 0xb0))
        .count();
    assert_eq!(auths, 3);
    // The AP's station never authenticated: the chip was not told it was.
    assert_eq!(air.count(|c| *c == Call::Authenticated), 0);
    drop(air);
    assert!(progress(&mut wifi).contains("auth 4/0 status -"));
    let Err(LinkError::Adapter(failure)) = wifi.receive() else {
        panic!("no failure")
    };
    assert!(failure.contains("authentication or association failed"));
}

#[test]
fn progress_reports_each_stage() {
    let (mut wifi, model) = wifi();
    let chip = "; chip";
    assert_eq!(
        progress(&mut wifi),
        alloc::format!(
            "scans 0 (0 networks, target -), auth -, assoc -, eapol -, keys -, port closed{chip}"
        )
    );
    // The scan found the target, and the join began.
    assert!(run(&mut wifi, &model, |_, air| {
        air.position(|c| matches!(c, Call::SetBssid(_))).is_some()
    }));
    assert_eq!(
        progress(&mut wifi),
        alloc::format!(
            "scans 1 (1 networks, target found ch 1 -40 dBm), auth 0/0 status -, assoc -, \
             eapol -, keys -, port closed{chip}"
        )
    );
    // Authenticated; the association request is out.
    assert!(run(&mut wifi, &model, |_, air| !air
        .management(0x00)
        .is_empty()));
    assert_eq!(
        progress(&mut wifi),
        alloc::format!(
            "scans 1 (1 networks, target found ch 1 -40 dBm), auth 1/1 status 0, \
             assoc 1/0 status - aid -, eapol -, keys -, port closed{chip}"
        )
    );
    // Associated.
    assert!(run(&mut wifi, &model, |wifi, _| {
        wifi.radio().station().phase() == Peering::Associated
    }));
    assert_eq!(
        progress(&mut wifi),
        alloc::format!(
            "scans 1 (1 networks, target found ch 1 -40 dBm), auth 1/1 status 0, \
             assoc 1/1 status 0 aid 1, eapol rx 0 tx 0 acked 0, keys -, port closed{chip}"
        )
    );
    // The handshake, its two keys and the port (M3b's pass).
    drop((wifi, model));
    let (mut wifi, _model) = connected();
    assert_eq!(
        progress(&mut wifi),
        alloc::format!(
            "scans 1 (1 networks, target found ch 1 -40 dBm), auth 1/1 status 0, \
             assoc 1/1 status 0 aid 1, eapol rx 2 tx 2 acked 2, keys 2, port open{chip}"
        )
    );
}

#[test]
fn diagnostic_names_no_address() {
    let (mut wifi, model) = associated();
    let mut lines = vec![progress(&mut wifi)];
    let mut ap = authenticator();
    model
        .0
        .borrow_mut()
        .hear(data(LOCAL, 0, [0x88, 0x8e], &ap.message_1()));
    assert!(run(&mut wifi, &model, |wifi, air| {
        lines.push(progress(wifi));
        !air.eapol().is_empty()
    }));
    ap.message_2(&model.0.borrow().eapol()[0]);
    model
        .0
        .borrow_mut()
        .hear(data(LOCAL, 1, [0x88, 0x8e], &ap.message_3(&RSN)));
    assert!(run(&mut wifi, &model, |wifi, _| {
        let line = progress(wifi);
        lines.push(line.clone());
        line.contains("port open")
    }));
    // Then the station leaves.
    wifi.radio_mut().deauthenticate(3).unwrap();
    assert!(run(&mut wifi, &model, |wifi, air| {
        lines.push(progress(wifi));
        air.count(|c| *c == Call::Disassociate) == 1
    }));
    assert!(names_an_address("ab:cd:ef:01:23:45"));
    for line in &lines {
        assert!(!names_an_address(line), "{line}");
        assert!(!line.contains("test"), "{line}");
    }
}

#[test]
fn a_deauthentication_sends_and_disassociates() {
    let (mut wifi, model) = associated();
    wifi.radio_mut().deauthenticate(3).unwrap();
    assert!(run(&mut wifi, &model, |_, air| {
        air.position(|c| *c == Call::Disassociate).is_some()
    }));
    let air = model.0.borrow();
    let deauth = air
        .position(|c| matches!(c, Call::Management(b, _) if b[0] == 0xc0))
        .unwrap();
    let left = air.position(|c| *c == Call::Disassociate).unwrap();
    assert!(deauth < left);
    let Call::Management(frame, _) = &air.calls[deauth] else {
        unreachable!()
    };
    assert_eq!((&frame[4..10], &frame[24..]), (&AP[..], &[3, 0][..]));
}

#[test]
fn a_join_that_authenticated_but_never_associated_leaves_through_the_chip() {
    // mac80211 moves the AP's station AUTH, then, at the association's
    // timeout, AUTH to NONE: the chip's disconnect H2Cs (spec S11).
    let (mut wifi, model) = wifi_with(|air| air.mute_on_assoc = true);
    assert!(run(&mut wifi, &model, |_, air| {
        air.position(|c| *c == Call::Disassociate).is_some()
    }));
    let air = model.0.borrow();
    let authenticated = air.position(|c| *c == Call::Authenticated).unwrap();
    let left = air.position(|c| *c == Call::Disassociate).unwrap();
    assert!(authenticated < left);
    assert_eq!(air.management(0x00).len(), 3);
    assert_eq!(air.count(|c| matches!(c, Call::Associate(_))), 0);
}

#[test]
fn a_full_chip_holds_frames_in_order_without_loss() {
    // Review Focus 2 on the radio's side: management frames wait in the
    // outbox, data in `held` (at most 256, the oldest dropped).
    let (mut wifi, model) = wifi_with(|air| air.full_management = true);
    assert!(run(&mut wifi, &model, |_, air| air.refused >= 10));
    assert!(model.0.borrow().management(0xb0).is_empty());
    model.0.borrow_mut().full_management = false;
    assert!(run(&mut wifi, &model, |wifi, _| {
        wifi.radio().station().phase() == Peering::Associated
    }));
    // The authentication waited and went once.
    assert_eq!(model.0.borrow().management(0xb0).len(), 1);
    // Data waits while the queues are full, the oldest dropped past 256.
    model.0.borrow_mut().full_data = true;
    for n in 0..300u16 {
        let mut frame = Vec::from(AP);
        frame.extend(LOCAL);
        frame.extend([0x88, 0x8e]);
        frame.extend(n.to_be_bytes());
        wifi.radio_mut().send_control(&frame, None).unwrap();
    }
    for _ in 0..3 {
        run(&mut wifi, &model, |_, _| true);
    }
    assert_eq!(model.0.borrow().eapol().len(), 0);
    model.0.borrow_mut().full_data = false;
    assert!(run(&mut wifi, &model, |_, air| air.eapol().len() == 256));
    let sent: Vec<u16> = model
        .0
        .borrow()
        .eapol()
        .iter()
        .map(|body| u16::from_be_bytes([body[0], body[1]]))
        .collect();
    assert_eq!(sent, (44..300).collect::<Vec<u16>>());
    // Each sequence number once, in order.
    let air = model.0.borrow();
    let sequences: Vec<u16> = air
        .calls
        .iter()
        .filter_map(|c| match c {
            Call::Data { bytes, .. } => Some(u16::from_le_bytes([bytes[22], bytes[23]]) >> 4),
            _ => None,
        })
        .collect();
    assert!(sequences.windows(2).all(|w| w[1] == (w[0] + 1) & 0xfff));
}

#[test]
fn a_deauthentication_drops_the_data_still_held() {
    // ieee80211_set_disassoc's flush with drop: what the chip has not
    // taken never leaves after the decision to deauthenticate.
    let (mut wifi, model) = associated();
    model.0.borrow_mut().full_data = true;
    let mut frame = Vec::from(AP);
    frame.extend(LOCAL);
    frame.extend([0x88, 0x8e, 1, 2, 3]);
    wifi.radio_mut().send_control(&frame, None).unwrap();
    wifi.radio_mut().deauthenticate(3).unwrap();
    model.0.borrow_mut().full_data = false;
    assert!(run(&mut wifi, &model, |_, air| {
        air.position(|c| *c == Call::Disassociate).is_some()
    }));
    let air = model.0.borrow();
    assert_eq!(air.count(|c| matches!(c, Call::Data { .. })), 0);
    assert_eq!(air.management(0xc0).len(), 1);
}

#[test]
fn a_beacon_failing_its_fcs_finds_no_network() {
    let (mut wifi, model) = wifi_with(|air| air.bad_fcs = true);
    assert!(run(&mut wifi, &model, |wifi, _| {
        progress(wifi).starts_with("scans 1 ")
    }));
    assert!(progress(&mut wifi).starts_with("scans 1 (0 networks, target missing)"));
    assert_eq!(
        model.0.borrow().count(|c| matches!(c, Call::SetBssid(_))),
        0
    );
}

#[test]
fn the_radio_waits_30_s_for_carrier_and_polls() {
    let (wifi, _model) = wifi();
    assert_eq!(wifi.carrier_wait_us(), WIFI_CARRIER_WAIT_US);
    assert_eq!(WIFI_CARRIER_WAIT_US, 30_000_000);
    assert!(!wifi.signals());
    assert_eq!(wifi.due_us(), None);
}

#[test]
fn the_advertisement_is_ht20_with_two_streams() {
    let (wifi, _model) = wifi();
    for band in [Band::Ghz2, Band::Ghz5] {
        let advertisement = wifi.radio().advertisement(band).unwrap();
        let elements = &advertisement.elements;
        let ht = elements.windows(2).position(|w| w == [45, 26]).unwrap();
        // HT capability 0x09AD, A-MPDU 0x03, MCS 0-15, 144 Mb/s, TX defined.
        assert_eq!(elements[ht + 2..ht + 5], [0xAD, 0x09, 0x03]);
        assert_eq!(
            elements[ht + 5..ht + 15],
            [0xff, 0xff, 0, 0, 0, 0, 0, 0, 0, 0]
        );
        assert_eq!(elements[ht + 15..ht + 18], [144, 0, 1]);
        // No VHT or HE (spec S1.7).
        assert!(!elements.windows(2).any(|w| w == [191, 12]));
        assert!(advertisement.capabilities.vht().is_none());
    }
}

#[test]
fn a_frame_the_station_refuses_is_reported_but_is_no_refusal() {
    // The AP's association response again, once associated: the station
    // refuses it, the keeper hears of it once, and the progress line's
    // `last` stays the radio's own refusals.
    let (mut wifi, model) = associated();
    let mut body = vec![0x11, 0, 0, 0, 1, 0xc0];
    body.extend(RATES);
    model.0.borrow_mut().hear(management(0x10, &body));
    assert!(run(&mut wifi, &model, |_, _| true));
    assert!(!progress(&mut wifi).contains("last"));
    let Err(LinkError::Adapter(failure)) = wifi.receive() else {
        panic!("no failure")
    };
    assert_eq!(
        failure,
        "rtw89 unsupported: a management frame the station refuses"
    );
}

#[test]
fn no_frame_leaves_encrypted_before_the_key_is_acked() {
    // Review Focus 3: the firmware never acknowledges the PTK. The key is
    // never completed, nothing goes under it, and the keeper fails the
    // handshake and joins again.
    let (mut wifi, model) = wifi_with(|air| air.refuse_keys = true);
    assert!(run(&mut wifi, &model, |wifi, _| {
        wifi.radio().station().phase() == Peering::Associated
    }));
    shake(&mut wifi, &model, &RSN, &[]);
    assert!(run(&mut wifi, &model, |_, air| {
        air.count(|c| matches!(c, Call::SetBssid(_))) == 2
    }));
    let Err(LinkError::Adapter(failure)) = wifi.receive() else {
        panic!("no failure")
    };
    assert!(failure.contains("never acknowledged"), "{failure}");
    let air = model.0.borrow();
    assert!(air.keys().is_empty());
    assert!(air.data().iter().all(|(_, key)| key.is_none()));
    assert!(air.count(|c| *c == Call::Disassociate) >= 1);
    drop(air);
    assert!(!wifi.carrier());
    assert!(progress(&mut wifi).contains("keys 0, port closed"));
}

#[test]
fn data_goes_with_its_pn_and_without_iv() {
    let (mut wifi, model) = connected();
    let before = model.0.borrow().data().len();
    wifi.transmit(&ip(1)).unwrap();
    wifi.transmit(&ip(2)).unwrap();
    let air = model.0.borrow();
    let sent = &air.data()[before..];
    // The PTK's CAM entry and key ID 0, under PNs 1 and 2: its
    // `TransmitSeed::host(0)`, as `atomic64_inc_return` counts from 0.
    let keys: Vec<FrameKey> = sent.iter().map(|(_, key)| *key).collect();
    assert_eq!(keys, [Some((0, 0, 1)), Some((0, 0, 2))]);
    for (bytes, _) in sent {
        // Protected QoS data whose 26-byte header is followed by the LLC:
        // the CCMP header is the hardware's to insert (spec S5.4).
        assert_eq!(bytes[..2], [0x88, 0x41]);
        assert_eq!(bytes[26..34], [0xaa, 0xaa, 3, 0, 0, 0, 8, 0]);
        assert_eq!(bytes.len(), 26 + 8 + 46);
    }
}

#[test]
fn pns_strictly_increase_across_a_full_ring() {
    // Review Focus 4: a frame the chip has no room for waits in `held` with
    // the PN reserved for it; one dropped from `held` still spent its PN.
    let (mut wifi, model) = connected();
    let before = model.0.borrow().data().len();
    let fill = |bytes: &[u8]| bytes[36];
    wifi.transmit(&ip(1)).unwrap();
    model.0.borrow_mut().full_data = true;
    wifi.transmit(&ip(2)).unwrap();
    wifi.transmit(&ip(3)).unwrap();
    assert_eq!(model.0.borrow().data().len(), before + 1);
    model.0.borrow_mut().full_data = false;
    assert!(run(&mut wifi, &model, |_, air| air.data().len() == before + 3));
    let sent: Vec<(u8, u64)> = model.0.borrow().data()[before..]
        .iter()
        .map(|(bytes, key)| (fill(bytes), key.unwrap().2))
        .collect();
    assert_eq!(sent, [(1, 1), (2, 2), (3, 3)]);
    // 257 frames wait: the oldest (PN 4) is dropped, the rest leave in
    // order, and the next frame takes PN 261, never a PN given before.
    model.0.borrow_mut().full_data = true;
    for n in 0..257u16 {
        wifi.transmit(&ip(n as u8)).unwrap();
    }
    model.0.borrow_mut().full_data = false;
    assert!(run(&mut wifi, &model, |_, air| air.data().len() == before + 259));
    wifi.transmit(&ip(9)).unwrap();
    let pns: Vec<u64> = model.0.borrow().data()[before + 3..]
        .iter()
        .map(|(_, key)| key.unwrap().2)
        .collect();
    assert_eq!(pns, (5..=261).collect::<Vec<u64>>());
}

#[test]
fn received_data_passes_replay_and_reaches_the_link() {
    let (mut wifi, model) = connected();
    let hear = |frame: Vec<u8>| model.0.borrow_mut().hear_as(frame, true, false);
    hear(protected(LOCAL, 10, 0, 1, &[0x45; 46]));
    wifi.carrier();
    assert_eq!(delivered(&mut wifi), [0x45]);
    // Replayed, then the next packet number.
    hear(protected(LOCAL, 10, 0, 1, &[0x45; 46]));
    hear(protected(LOCAL, 11, 0, 2, &[0x46; 46]));
    wifi.carrier();
    assert_eq!(delivered(&mut wifi), [0x46]);
    // A group frame under the GTK of key ID 1, past its RSC of 7.
    hear(protected([0xff; 6], 12, 1, 8, &[0x47; 46]));
    wifi.carrier();
    assert_eq!(delivered(&mut wifi), [0x47]);
    // Unprotected data is refused once the port is controlled.
    model
        .0
        .borrow_mut()
        .hear(data(LOCAL, 13, [8, 0], &[0x48; 46]));
    wifi.carrier();
    assert_eq!(delivered(&mut wifi), []);
}

#[test]
fn an_undecrypted_protected_frame_is_dropped() {
    let (mut wifi, model) = connected();
    let body = [0x45; 46];
    let mut air = model.0.borrow_mut();
    // Not decrypted by the hardware, failing its ICV, and under a key ID
    // no key has (spec S13.3).
    air.hear_as(protected(LOCAL, 10, 0, 1, &body), false, false);
    air.hear_as(protected(LOCAL, 11, 0, 2, &body), true, true);
    air.hear_as(protected(LOCAL, 12, 2, 3, &body), true, false);
    drop(air);
    wifi.carrier();
    assert_eq!(delivered(&mut wifi), []);
    // None of them spent a packet number.
    model
        .0
        .borrow_mut()
        .hear_as(protected(LOCAL, 13, 0, 1, &body), true, false);
    wifi.carrier();
    assert_eq!(delivered(&mut wifi), [0x45]);
}

#[test]
fn a_robust_management_frame_is_sealed_on_the_host() {
    // Spec S1.2: sealed under the PTK with `realtek::ccmp`, handed to the
    // chip as any management frame (no hardware key).
    let (mut wifi, model) = connected_with_pmf();
    let ptk = model.0.borrow().ptk();
    wifi.radio_mut().deauthenticate(3).unwrap();
    let air = model.0.borrow();
    let deauth = air.management(0xc0);
    let [frame] = &deauth[..] else {
        panic!("{} deauthentications", deauth.len())
    };
    // Protected, under the PTK's next PN (nothing went under it before),
    // its reason sealed and its MIC appended.
    assert_eq!(frame[1] & 0x40, 0x40);
    assert_eq!(frame[24..32], [1, 0, 0, 0x20, 0, 0, 0, 0]);
    assert_eq!(frame.len(), 24 + 8 + 2 + 8);
    let opened = ccmp::open(&ptk, frame, PacketNumber::new(1).unwrap()).unwrap();
    assert_eq!(opened[32..34], [3, 0]);
}

#[test]
fn under_pmf_an_unprotected_departure_is_ignored() {
    // `ieee80211_drop_unencrypted_mgmt`: neither a unicast nor a group
    // deauthentication without its protection ends a PMF association.
    let (mut wifi, model) = connected_with_pmf();
    let mut group = management(0xc0, &[7, 0]);
    group[4..10].fill(0xff);
    model.0.borrow_mut().hear(management(0xc0, &[7, 0]));
    model.0.borrow_mut().hear(group);
    for _ in 0..3 {
        run(&mut wifi, &model, |_, _| true);
    }
    assert_eq!(model.0.borrow().count(|c| *c == Call::Disassociate), 0);
    assert!(wifi.carrier());
}

#[test]
fn a_tkip_or_ccmp256_key_is_unsupported() {
    let (mut wifi, model) = associated();
    for (pairwise, index, length) in [(true, 0, 32), (false, 1, 32), (false, 1, 13)] {
        let got = wifi
            .radio_mut()
            .install_key(pairwise, index, &vec![0; length], ReplaySeed::ZERO);
        assert!(
            matches!(
                got,
                Err(Error::Unsupported(
                    "a key other than CCMP-128 on an RTL8852C"
                ))
            ),
            "{got:?}"
        );
    }
    assert!(model.0.borrow().keys().is_empty());
    let line = progress(&mut wifi);
    assert!(
        line.contains("last \"a key other than CCMP-128 on an RTL8852C\""),
        "{line}"
    );
}

#[test]
fn an_authentication_reported_late_still_counts_its_ack() {
    // The M3a boot read 'auth 1/0' though the chip saw every frame
    // acknowledged: the authentication's report came after the association
    // request had replaced it as the frame the station awaits.
    let (mut wifi, model) = wifi_with(|air| air.late_auth_report = true);
    assert!(run(&mut wifi, &model, |wifi, _| {
        wifi.radio().station().phase() == Peering::Associated
    }));
    let line = progress(&mut wifi);
    assert!(
        line.contains("auth 1/1 status 0, assoc 1/1 status 0"),
        "{line}"
    );
}

/// The flushes, management frames, key calls and the leaving among the
/// chip's calls from `from` to the first disassociation.
fn leaving(air: &Air, from: usize) -> Vec<String> {
    let end = from
        + air.calls[from..]
            .iter()
            .position(|c| *c == Call::Disassociate)
            .unwrap();
    air.calls[from..=end]
        .iter()
        .filter_map(|c| match c {
            Call::Flush(drop) => Some(alloc::format!("flush {drop}")),
            Call::Management(bytes, _) => Some(alloc::format!("mgmt {:02x}", bytes[0])),
            Call::RemoveKey(cam) => Some(alloc::format!("remove {cam}")),
            Call::Disassociate => Some(String::from("leave")),
            Call::AddKey { .. } => Some(String::from("add")),
            _ => None,
        })
        .collect()
}

#[test]
fn a_deauthentication_flushes_and_deletes_the_keys_before_leaving() {
    // ieee80211_set_disassoc: a flush with drop, the deauthentication, a
    // flush without; then sta_info_flush deletes the PTK (DISABLE_KEY:
    // rtw89_ops_set_key's flushes, then the CAM delete), the GTK after it
    // (spec S14.5), and the AP's station leaves.
    let (mut wifi, model) = connected();
    let from = model.0.borrow().calls.len();
    wifi.radio_mut().deauthenticate(3).unwrap();
    assert!(run(&mut wifi, &model, |_, air| {
        air.position(|c| *c == Call::Disassociate).is_some()
    }));
    assert_eq!(
        leaving(&model.0.borrow(), from),
        [
            "flush true",
            "mgmt c0",
            "flush false",
            "flush false",
            "remove 0",
            "flush false",
            "remove 1",
            "leave"
        ]
    );
    assert!(!wifi.carrier());
}

#[test]
fn a_deauthentication_from_the_ap_drops_what_is_queued_and_deletes_the_keys() {
    // ieee80211_set_disassoc without a frame of its own: the flush with
    // drop alone, then the keys and the leaving.
    let (mut wifi, model) = connected();
    let from = model.0.borrow().calls.len();
    model.0.borrow_mut().hear(management(0xc0, &[7, 0]));
    assert!(run(&mut wifi, &model, |_, air| {
        air.position(|c| *c == Call::Disassociate).is_some()
    }));
    assert_eq!(
        leaving(&model.0.borrow(), from),
        [
            "flush true",
            "flush false",
            "remove 0",
            "flush false",
            "remove 1",
            "leave"
        ]
    );
}

#[test]
fn a_group_rekey_replaces_the_key_of_its_index() {
    // ieee80211_key_replace: the old key of the same index goes first
    // (DISABLE_KEY), then the new one; a key of another index is added.
    // The port stays open on the PTK and the GTK last installed.
    let (mut wifi, model) = connected();
    let hear = |frame: Vec<u8>| model.0.borrow_mut().hear_as(frame, true, false);
    let from = model.0.borrow().calls.len();
    wifi.radio_mut()
        .install_key(false, 1, &[0x66; 16], ReplaySeed::ZERO)
        .unwrap();
    {
        let air = model.0.borrow();
        let calls: Vec<&Call> = air.calls[from..].iter().collect();
        assert_eq!(calls.len(), 3, "{calls:?}");
        assert_eq!(*calls[0], Call::Flush(false));
        assert_eq!(*calls[1], Call::RemoveKey(1));
        assert!(matches!(
            calls[2],
            Call::AddKey {
                pairwise: false,
                index: 1,
                cam: 1,
                ..
            }
        ));
    }
    hear(protected([0xff; 6], 20, 1, 1, &[0x51; 46]));
    hear(protected(LOCAL, 21, 0, 1, &[0x52; 46]));
    wifi.carrier();
    assert_eq!(delivered(&mut wifi), [0x51, 0x52]);
    let from = model.0.borrow().calls.len();
    wifi.radio_mut()
        .install_key(false, 2, &[0x67; 16], ReplaySeed::ZERO)
        .unwrap();
    {
        let air = model.0.borrow();
        let calls: Vec<&Call> = air.calls[from..].iter().collect();
        assert_eq!(calls.len(), 1, "{calls:?}");
        assert!(matches!(
            calls[0],
            Call::AddKey {
                pairwise: false,
                index: 2,
                cam: 2,
                ..
            }
        ));
    }
    hear(protected([0xff; 6], 22, 2, 1, &[0x53; 46]));
    hear(protected(LOCAL, 23, 0, 2, &[0x54; 46]));
    wifi.carrier();
    assert_eq!(delivered(&mut wifi), [0x53, 0x54]);
    assert!(wifi.carrier());
    // The same key again is no new install (KRACK, ieee80211_key_identical).
    let from = model.0.borrow().calls.len();
    wifi.radio_mut()
        .install_key(false, 2, &[0x67; 16], ReplaySeed::ZERO)
        .unwrap();
    assert_eq!(model.0.borrow().calls.len(), from);
    assert!(progress(&mut wifi).contains("keys 4"));
}

/// Fragment `number` of frame `sequence` under the PTK at packet number
/// `pn`, with `more` to follow.
fn fragment(sequence: u16, number: u8, more: bool, pn: u64, payload: &[u8]) -> Vec<u8> {
    let mut frame = protected(LOCAL, sequence, 0, pn, &[]);
    frame.truncate(34);
    frame.extend_from_slice(payload);
    frame.extend([0; 8]);
    frame[1] |= u8::from(more) << 2;
    frame[22] |= number;
    frame
}

#[test]
fn fragments_under_consecutive_numbers_make_one_frame() {
    let (mut wifi, model) = connected();
    let first = [&[0xaa, 0xaa, 3, 0, 0, 0, 8, 0][..], &[0x45; 4]].concat();
    let hear = |frame: Vec<u8>| model.0.borrow_mut().hear_as(frame, true, false);
    hear(fragment(30, 0, true, 1, &first));
    hear(fragment(30, 1, false, 2, &[0x46; 4]));
    wifi.carrier();
    let ethernet = wifi.receive().unwrap().unwrap();
    assert_eq!(
        ethernet[14..],
        [0x45, 0x45, 0x45, 0x45, 0x46, 0x46, 0x46, 0x46]
    );
    // Numbers that skip one are dropped.
    hear(fragment(31, 0, true, 5, &first));
    hear(fragment(31, 1, false, 7, &[0x46; 4]));
    wifi.carrier();
    assert_eq!(delivered(&mut wifi), []);
}

#[test]
fn an_a_msdu_gives_up_each_of_its_frames() {
    let (mut wifi, model) = connected();
    let mut frame = protected(LOCAL, 20, 0, 1, &[]);
    frame.truncate(34);
    frame[24] = 0x80;
    for value in [0x31, 0x32] {
        let padding = (4 - frame[34..].len() % 4) % 4;
        frame.extend(vec![0; padding]);
        frame.extend(LOCAL);
        frame.extend(AP);
        frame.extend(54u16.to_be_bytes());
        frame.extend([0xaa, 0xaa, 3, 0, 0, 0, 8, 0]);
        frame.extend([value; 46]);
    }
    frame.extend([0; 8]);
    model.0.borrow_mut().hear_as(frame, true, false);
    wifi.carrier();
    assert_eq!(delivered(&mut wifi), [0x31, 0x32]);
}

/// An SA Query request at packet number `number` as the hardware leaves
/// it decrypted, its MIC in place.
fn sa_query(number: u8) -> Vec<u8> {
    let mut frame = management(0xd0, &[number, 0, 0, 0x20, 0, 0, 0, 0, 8, 0, 1, 2]);
    frame[1] = 0x40;
    frame.extend([0; 8]);
    frame
}

#[test]
fn a_protected_management_frame_is_taken_once_under_its_packet_number() {
    let (mut wifi, model) = connected();
    let answers = |air: &Air| air.management(0xd0);
    model.0.borrow_mut().hear_as(sa_query(5), true, false);
    assert!(run(&mut wifi, &model, |_, air| answers(air).len() == 1));
    model.0.borrow_mut().hear_as(sa_query(5), true, false);
    assert!(!run(&mut wifi, &model, |_, air| answers(air).len() > 1));
    model.0.borrow_mut().hear_as(sa_query(6), true, false);
    assert!(run(&mut wifi, &model, |_, air| answers(air).len() == 2));
    // Each answer is sealed under the PTK and answers its request's ID.
    let air = model.0.borrow();
    let ptk = air.ptk();
    for (n, answer) in answers(&air).iter().enumerate() {
        let pn = PacketNumber::new(n as u64 + 1).unwrap();
        let opened = ccmp::open(&ptk, answer, pn).unwrap();
        assert_eq!(opened[32..36], [8, 1, 1, 2]);
    }
}

#[test]
fn a_pairwise_rekey_drops_what_waits_under_the_old_key() {
    // The new PTK takes the CAM entry the old one left: a frame held under
    // the old key and its PN must not leave under the new one.
    let (mut wifi, model) = connected();
    let before = model.0.borrow().data().len();
    model.0.borrow_mut().full_data = true;
    wifi.transmit(&ip(1)).unwrap();
    wifi.radio_mut()
        .install_key(true, 0, &[0x99; 16], ReplaySeed::ZERO)
        .unwrap();
    assert_eq!(model.0.borrow().keys().last(), Some(&(true, 0, 0)));
    model.0.borrow_mut().full_data = false;
    wifi.transmit(&ip(2)).unwrap();
    assert!(run(&mut wifi, &model, |_, air| air.data().len() > before));
    for _ in 0..3 {
        run(&mut wifi, &model, |_, _| true);
    }
    let sent: Vec<(u8, FrameKey)> = model.0.borrow().data()[before..]
        .iter()
        .map(|(bytes, key)| (bytes[36], *key))
        .collect();
    assert_eq!(sent, [(2, Some((0, 0, 1)))]);
}
