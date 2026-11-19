use super::*;

const S8_PACKET: [u8; 32] = [
    0x10, 0x00, 0x00, 0x0D, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x25, 0x14, 0x00, 0x00, 0x10, 0xC0,
    0x00, 0x00, 0x09, 0x00, 0x00, 0x00, 0x5E, 0x00, 0x00, 0x00,
];

#[test]
fn ofld_cfg_packet_is_byte_exact() {
    assert_eq!(H2c::after_download().ofld_cfg(), (0, S8_PACKET.to_vec()));
}

#[test]
fn rec_ack_forced_every_fourth_seq() {
    let mut h = H2c::after_download();
    for want in 0u8..5 {
        let (seq, p) = h.packet(1, 9, 0x14, false, &[]);
        assert_eq!(seq, want);
        assert_eq!(p.len(), 24);
        let hdr0 = u32::from_le_bytes(p[16..20].try_into().unwrap());
        let hdr1 = u32::from_le_bytes(p[20..24].try_into().unwrap());
        assert_eq!(hdr0 >> 24, u32::from(want));
        assert_eq!(hdr1 & 0x3FFF, 8);
        assert_eq!(hdr1 & (1 << 14) != 0, want % 4 == 0, "seq {want}");
        assert_eq!(hdr1 & (1 << 15), 0);
    }
    // 5 calls made so far; 251 more reach 256 and wrap to 0.
    for _ in 0..251 {
        h.packet(1, 9, 0x14, false, &[]);
    }
    assert_eq!(h.packet(1, 9, 0x14, false, &[]).0, 0);
}

#[test]
fn explicit_rec_ack_is_never_cleared_and_payload_is_carried() {
    let mut h = H2c::after_download();
    h.packet(0, 0, 0, false, &[]);
    let (seq, p) = h.packet(1, 9, 0x14, true, &[1, 2, 3]);
    assert_eq!(seq, 1);
    assert_eq!(&p[24..], &[1, 2, 3]);
    let hdr1 = u32::from_le_bytes(p[20..24].try_into().unwrap());
    assert_eq!(hdr1, 11 | 1 << 15);
    // descriptor pkt_size = 8 + payload
    assert_eq!(u32::from_le_bytes(p[..4].try_into().unwrap()), 0x0D00_000B);
}
