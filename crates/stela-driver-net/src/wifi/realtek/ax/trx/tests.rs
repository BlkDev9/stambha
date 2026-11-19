extern crate std;
use super::*;
use crate::wifi::realtek::ax::tests::model::{Op, RegisterModel};
use alloc::{boxed::Box, vec::Vec};

/// One IMR register write step: a clear, a set, or a masked field write.
#[derive(Clone, Copy)]
enum Step {
    Clr(u32),
    Set(u32),
    Field(u32, u32),
}

use Step::{Clr, Field, Set};

/// `rtw89_imr_info` for an RTL8852C, in `enable_imr_ax`'s order (`mac.c:3936`),
/// then the CMAC half, then `set_host_rpr_ax` (`mac.c:4009`) and the 8852C
/// tail (`mac.c:4075`). `0x8520` and `0xC160` are plain writes, listed
/// apart.
fn table() -> Vec<(u32, Vec<Step>)> {
    alloc::vec![
        // DMAC
        (0x9430, alloc::vec![Clr(0x3337), Set(0x3337)]),
        (0x9D2C, alloc::vec![Set(0x3)]),
        (0x9BF4, alloc::vec![Clr(0x3E), Clr(0x3C0), Set(0x36E)]),
        (0x9CF4, alloc::vec![Clr(0xB), Set(0x8)]),
        (0x9EF0, alloc::vec![Clr(0x7), Set(0x7)]),
        (0x9F78, alloc::vec![Clr(0x0035_0F0F), Set(0x0035_0B03)]),
        (0x9FB8, alloc::vec![Clr(0x0035_0F0F), Set(0x0035_0F03)]),
        (0x8C38, alloc::vec![Clr(0x3F0F_F3FF), Set(0x3F0F_F3FF)]),
        (0x9038, alloc::vec![Clr(0x3F0F_F3FF), Set(0x3F0F_F3FF)]),
        (0x9A20, alloc::vec![Set(0x1)]),
        (0x8850, alloc::vec![Clr(0xFFD7_FFFF), Set(0x0C10_0060)]),
        (0x8854, alloc::vec![Clr(0x7F7B_FFFD), Set(0x3600_0060)]),
        (0x8858, alloc::vec![Clr(0xFFFF_DFDF), Set(0x303C_C000)]),
        (0x9840, alloc::vec![Clr(0x1111), Set(0x1111)]),
        (0x9608, alloc::vec![Set(0x1)]),
        (0x9628, alloc::vec![Clr(0xFF), Set(0xFF)]),
        (0x9638, alloc::vec![Set(0x1)]),
        (0x966C, alloc::vec![Set(0x1)]),
        // CMAC band 0
        (0xC3E8, alloc::vec![Clr(0x3), Set(0x1)]),
        (0xC6C0, alloc::vec![Clr(0x3), Set(0x3)]),
        (0xC804, alloc::vec![Clr(0x7FFF_FF06), Set(0x7FFF_FF06)]),
        (0xC870, alloc::vec![Clr(0xFF00_C000), Set(0xFF00_C000)]),
        (0xCCF8, alloc::vec![Clr(0x3F), Set(0x3F)]),
        (0xCEF8, alloc::vec![Clr(0x3FF), Set(0x3FF)]),
        (0xCCBC, alloc::vec![Clr(0x1FF), Set(0x17F)]),
    ]
}

/// The tail after the two IMR tables, with its plain writes first.
fn tail() -> Vec<(u32, Vec<Step>)> {
    alloc::vec![
        (0x9408, alloc::vec![Field(0x3, 0)]),
        (0x9410, alloc::vec![Set(0x0F00_0000)]),
        (0x9414, alloc::vec![Field(0xFF, 30)]),
        (0x9414, alloc::vec![Field(0x00FF_0000, 255)]),
        (0xCC00, alloc::vec![Clr(0x4000_0000)]),
    ]
}

fn apply(prev: u32, step: Step) -> u32 {
    match step {
        Clr(m) => prev & !m,
        Set(m) => prev | m,
        Field(m, v) => (prev & !m) | ((v << m.trailing_zeros()) & m),
    }
}

fn expect(
    want: &mut Vec<Op>,
    state: &mut std::collections::BTreeMap<u32, u32>,
    reg: u32,
    step: Step,
) {
    let prev = *state.get(&reg).unwrap_or(&0xFFFF_FFFF);
    let next = apply(prev, step);
    state.insert(reg, next);
    want.push(Op::W32(reg, next));
}

#[test]
fn imr_tail_matches_linux_sequence() {
    let mut m = RegisterModel::new();
    let regs: Vec<u32> = table()
        .iter()
        .chain(tail().iter())
        .map(|(r, _)| *r)
        .chain([0x8520, 0xC160, 0xE160])
        .collect();
    for r in regs {
        m.set32(r, 0xFFFF_FFFF);
    }
    m.set32(0x8400, 0x6000_0000);
    m.set32(0xC000, 0x4000_0000);

    imr_tail(&mut m).unwrap();

    let mut state = std::collections::BTreeMap::new();
    let mut want = Vec::new();
    for (reg, steps) in table() {
        for s in steps {
            expect(&mut want, &mut state, reg, s);
        }
    }
    want.push(Op::W32(0x8520, 0xFFFF_FFFF));
    want.push(Op::W32(0xC160, 0xFFFF_FFFF));
    for (reg, steps) in tail() {
        for s in steps {
            expect(&mut want, &mut state, reg, s);
        }
    }
    assert_eq!(m.writes(), want);
    // err_imr_ctrl_ax skips the C1 error IMR: band 1 is not initialized.
    assert!(
        !m.writes()
            .iter()
            .any(|op| matches!(op, Op::W32(reg, _) if *reg >= 0xE000))
    );
}

#[test]
fn imr_tail_refuses_disabled_macs_before_writing() {
    let mut m = RegisterModel::new();
    m.set32(0xC000, 0x4000_0000);
    assert_eq!(
        imr_tail(&mut m),
        Err(AxError::Dma("DMAC function is not enabled"))
    );
    assert!(m.writes().is_empty());

    let mut m = RegisterModel::new();
    m.set32(0x8400, 0x6000_0000);
    assert_eq!(
        imr_tail(&mut m),
        Err(AxError::Mac("CMAC function is not enabled"))
    );
    // The DMAC half ran, as in Linux, before the CMAC check failed.
    assert!(m.trace.contains(&Op::W32(0x9430, 0x3337)));
    assert!(!m.writes().iter().any(|op| matches!(op, Op::W32(0xC3E8, _))));
}

/// Both MACs up and every wait reporting done, so `dmac::init` and
/// `cmac::init` both finish.
fn up() -> RegisterModel {
    let mut m = RegisterModel::new();
    m.on_read(0x8D00, Box::new(|_: &mut RegisterModel| 3));
    m.on_read(0x9100, Box::new(|_: &mut RegisterModel| 3));
    m.set32(0x8400, 0x6000_0000);
    m.set32(0xC000, 0x4000_0000);
    m.set32(0x1700, 0x0000_0C39);
    m.set32(0x179C, 0x0C9C);
    m.on_read(
        0x9E10,
        Box::new(|m: &mut RegisterModel| m.get32(0x9E10) | 0x8000_0000),
    );
    m.on_read(
        0xCE34,
        Box::new(|m: &mut RegisterModel| m.get32(0xCE34) & !0x100),
    );
    m.on_read(
        0xCE3C,
        Box::new(|m: &mut RegisterModel| m.get32(0xCE3C) & !0x3),
    );
    m
}

#[test]
fn trx_init_names_its_failing_step() {
    let mut m = up();
    m.on_read(0x9E10, Box::new(|m: &mut RegisterModel| m.get32(0x9E10)));
    let err = init(&mut m).unwrap_err();
    assert_eq!(err.step, "dmac init");
    assert!(matches!(
        err.error,
        AxError::Timeout {
            register: 0x9E10,
            ..
        }
    ));

    // The last DMAC register read clears the CMAC enable behind dmac::init.
    let mut m = up();
    m.on_read(
        0x9D1C,
        Box::new(|m: &mut RegisterModel| {
            m.set32(0xC000, 0);
            m.get32(0x9D1C)
        }),
    );
    let err = init(&mut m).unwrap_err();
    assert_eq!(err.step, "cmac init");
    assert_eq!(err.error, AxError::Mac("CMAC function is not enabled"));
    assert!(!m.writes().iter().any(|op| matches!(op, Op::W32(0x9430, _))));
}

#[test]
fn trx_init_runs_dmac_cmac_then_imr() {
    let mut m = up();
    init(&mut m).unwrap();
    let pos = |want: fn(&Op) -> bool| m.trace.iter().position(want).unwrap();
    let dle = pos(|op| matches!(op, Op::W32(0x8C40, _)));
    let cmac = pos(|op| matches!(op, Op::W32(0xC33C, _)));
    let wdrls = pos(|op| matches!(op, Op::W32(0x9430, _)));
    let err = pos(|op| matches!(op, Op::W32(0x8520, _)));
    let rpr = pos(|op| matches!(op, Op::W32(0x9408, _)));
    assert!(dle < cmac && cmac < wdrls && wdrls < err && err < rpr);
}
