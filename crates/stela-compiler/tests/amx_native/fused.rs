//! `fma32`, `fms32`, `fma64` and `fms64` in every tier.
use crate::harness::{self, Case, Form, GENERATIONS, Paths, differential, native};
use stela_compiler::amx::{AmxOperation, Generation, native::Tier};

const VECTOR: u64 = 1 << 63;
const X_HALF: u64 = 1 << 61;
const Y_HALF: u64 = 1 << 60;

/// Where the fused paths leave an operand to the model: the model tier,
/// which completes only copies and constants; z ± x and z ± y (bits 29:27 of
/// 2 and 4); copies of f16-widened halves, whose NaN payloads fms32 negates;
/// and fma32's widening on the VEX tier without F16C.
fn may_refuse(paths: &Paths, operation: AmxOperation, _: Generation, operand: u64) -> bool {
    let mode = (operand >> 27) & 7;
    let single = matches!(operation, AmxOperation::Fma32 | AmxOperation::Fms32);
    let widened = single && operand & (X_HALF | Y_HALF) != 0;
    let copies_half =
        single && (mode == 3 && operand & X_HALF != 0 || mode == 5 && operand & Y_HALF != 0);
    paths.tier() == Tier::Model
        || matches!(mode, 2 | 4)
        || copies_half
        || widened && paths.tier() == Tier::Fma && !paths.f16c
}

/// The unit test's seed, operands and floor (more than 5000 of 16000 on
/// M2), with its 16000 operands spread over the four generations, and a
/// floor for each shape and mode class: about half of what each ran.
const FUSED: Case = Case {
    name: "fused",
    seed: 0x9e37_79b9_7f4a_7c15,
    operations: &[
        AmxOperation::Fma32,
        AmxOperation::Fms32,
        AmxOperation::Fma64,
        AmxOperation::Fms64,
    ],
    generations: &GENERATIONS,
    iterations: 1000,
    fill: harness::fill,
    operand: harness::operand,
    runs_on: |_| true,
    may_refuse,
    forms: &[
        Form {
            name: "any operand",
            of: |_, _| true,
            floor: |paths| with_fma(paths, 1251),
        },
        Form {
            name: "matrix z ± x*y or x*y",
            of: |_, operand| operand & VECTOR == 0 && arithmetic(operand),
            floor: |paths| with_fma(paths, 400),
        },
        Form {
            name: "vector z ± x*y or x*y",
            of: |_, operand| operand & VECTOR != 0 && arithmetic(operand),
            floor: |paths| with_fma(paths, 400),
        },
        Form {
            name: "f16-widened fma32/fms32",
            of: |operation, operand| {
                matches!(operation, AmxOperation::Fma32 | AmxOperation::Fms32)
                    && operand & (X_HALF | Y_HALF) != 0
                    && arithmetic(operand)
            },
            floor: |paths| {
                if paths.tier() == Tier::Fma && !paths.f16c {
                    0
                } else {
                    with_fma(paths, 450)
                }
            },
        },
        Form {
            name: "copies and constants",
            of: |_, operand| matches!((operand >> 27) & 7, 3 | 5 | 6 | 7),
            floor: |_| 400,
        },
    ],
};

/// z ± x*y or x*y: bits 29:27 of 0 or 1.
fn arithmetic(operand: u64) -> bool {
    (operand >> 27) & 7 <= 1
}

/// `floor` on the tiers that multiply-add, none on the model's.
fn with_fma(paths: &Paths, floor: usize) -> usize {
    if paths.tier() == Tier::Model {
        0
    } else {
        floor
    }
}

#[test]
fn every_native_tier_matches_the_model_bit_for_bit() {
    differential(&FUSED, &native);
}
