extern crate std;
use super::*;

use alloc::{vec, vec::Vec};
fn buf(info: u32, d0: u32, total: usize) -> Vec<u8> {
    let mut b = vec![0u8; total];
    b[..4].copy_from_slice(&info.to_le_bytes());
    b[4..8].copy_from_slice(&d0.to_le_bytes());
    b
}

const FS_LS: u32 = 0xC000;

#[test]
fn rx_parse_short_descriptor_c2h() {
    let mut b = buf(FS_LS | 32, 0x0A00_000C, 32);
    for (i, x) in b[20..32].iter_mut().enumerate() {
        *x = i as u8 + 1;
    }
    let want = b[20..32].to_vec();
    assert_eq!(
        parse(&b),
        Ok(RxBuffer::Frame(RxFrame {
            pkt_type: 10,
            payload: &want
        }))
    );
}

#[test]
fn rx_parse_long_descriptor_with_drv_info() {
    // LONG b31, DRV_INFO 2 (bits 30:28), type 10, RPKT_LEN 6.
    let d0 = 1 << 31 | 2 << 28 | 10 << 24 | 6;
    let mut b = buf(FS_LS | 60, d0, 60);
    b[52..58].copy_from_slice(&[9, 8, 7, 6, 5, 4]);
    assert_eq!(
        parse(&b),
        Ok(RxBuffer::Frame(RxFrame {
            pkt_type: 10,
            payload: &[9, 8, 7, 6, 5, 4]
        }))
    );
}

#[test]
fn rx_parse_caps_payload_at_rpkt_len() {
    let b = buf(FS_LS | 40, 0x0A00_000C, 40);
    let Ok(RxBuffer::Frame(f)) = parse(&b) else {
        panic!()
    };
    assert_eq!(f.payload.len(), 12);
    assert_eq!(f.payload.as_ptr(), b[20..].as_ptr());
}

#[test]
fn rx_parse_reports_segment() {
    let b = buf(0x4000 | 32, 0x0A00_000C, 32);
    assert_eq!(parse(&b), Ok(RxBuffer::Segment));
    let b = buf(0x8000 | 32, 0x0A00_000C, 32);
    assert_eq!(parse(&b), Ok(RxBuffer::Segment));
}

#[test]
fn rx_buffer_shorter_than_its_descriptor_is_refused() {
    let err = Err(AxError::Rx("RX buffer shorter than its descriptor"));
    assert_eq!(parse(&buf(FS_LS | 10, 0x0A00_000C, 32)), err);
    // WRITE_SIZE claims 32 but the slice is shorter.
    assert_eq!(parse(&buf(FS_LS | 32, 0x0A00_000C, 32)[..30]), err);
    assert_eq!(parse(&[0; 3]), err);
}
