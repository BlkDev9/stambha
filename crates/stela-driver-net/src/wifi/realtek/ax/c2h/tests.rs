extern crate std;
use super::*;

use alloc::vec::Vec;
fn c2h(w0: u32, w2: u32) -> Vec<u8> {
    let mut v = Vec::new();
    v.extend_from_slice(&w0.to_le_bytes());
    v.extend_from_slice(&12u32.to_le_bytes());
    v.extend_from_slice(&w2.to_le_bytes());
    v
}

#[test]
fn c2h_parses_rec_and_done_acks() {
    assert_eq!(
        parse(&c2h(0x0000_0001, 0x1425)),
        Some(C2h::RecAck {
            cat: 1,
            class: 9,
            func: 0x14,
            seq: 0
        })
    );
    assert_eq!(
        parse(&c2h(0x0000_0101, 0x1425)),
        Some(C2h::DoneAck {
            cat: 1,
            class: 9,
            func: 0x14,
            ret: 0,
            seq: 0
        })
    );
    assert_eq!(
        parse(&c2h(0x0000_0101, 0x0705_1425)),
        Some(C2h::DoneAck {
            cat: 1,
            class: 9,
            func: 0x14,
            ret: 5,
            seq: 7
        })
    );
    // REC_ACK seq is [23:16].
    assert_eq!(
        parse(&c2h(0x0000_0001, 0x0003_1425)),
        Some(C2h::RecAck {
            cat: 1,
            class: 9,
            func: 0x14,
            seq: 3
        })
    );
    // An ack shorter than 12 bytes is skipped.
    assert_eq!(parse(&c2h(0x0000_0001, 0)[..11]), None);
}

#[test]
fn c2h_log_plain_and_formatted() {
    let mut p = c2h(0x0000_0201, 0)[..8].to_vec();
    p.extend_from_slice(b"hello");
    assert_eq!(parse(&p), Some(C2h::Log(b"hello")));
    let mut p = c2h(0x0000_0201, 0)[..8].to_vec();
    p.extend_from_slice(&[0xA5, 0xA5]);
    p.extend_from_slice(&[0; 9]);
    assert_eq!(parse(&p), Some(C2h::FormattedLog));
    // 10 body bytes starting A5 A5 is still plain.
    let mut p = c2h(0x0000_0201, 0)[..8].to_vec();
    p.extend_from_slice(&[0xA5, 0xA5, 0, 0, 0, 0, 0, 0, 0, 0]);
    assert!(matches!(parse(&p), Some(C2h::Log(_))));
}

#[test]
fn c2h_other_and_short() {
    assert_eq!(
        parse(&c2h(0x0000_0102 | 0x4, 0)),
        Some(C2h::Other {
            cat: 2,
            class: 1,
            func: 1
        })
    );
    assert_eq!(parse(&[0; 7]), None);
}
