//! `amx::native` against `amx::model`, its definition, through the public
//! interface alone: every host path the tests cover runs, each operand's
//! native result equals the model's byte for byte under a hostile caller
//! MXCSR the path must leave as it found it, a refusal changes nothing and
//! is one the test admits, and each named form ran natively at least its
//! floor's number of times ([`harness::differential`]).
//!
//! A host that lacks a tier these tests cover fails them, unless
//! `STELA_AMX_OPTIONAL` names that tier's feature (`fma`, `avx512f` or
//! `f16c`, comma separated), which leaves its paths out with a note
//! ([`harness::tiers`]).
//!
//! Run: `cargo test -p stela-compiler --test amx_native`.
#![cfg(target_arch = "x86_64")]

mod checks;
mod fused;
mod half;
mod harness;
