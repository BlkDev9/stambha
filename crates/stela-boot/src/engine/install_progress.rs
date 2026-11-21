//! An install's progress in the boot log and, while macOS draws nothing new,
//! on the screen ([`stela_boot_core::install_progress`]).
//!
//! The restore host's service, on whichever PE serves it, leaves how much
//! of the system image it has sent ([`image`]); the boot PE reads the
//! installer's steps from the guest's console ([`step`]) and, at its slices'
//! ends, once a second, writes the log's lines and gives the screen to
//! Stela's console or back ([`slice_end`]). Only an install's run turns it
//! on ([`begin`]): in any other boot each slice's end reads one flag.
use super::display;
use crate::output;
use core::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use stela_boot_core::install_progress::{Action, IDLE_SECONDS, Progress, Screen};
use stela_boot_core::screen::Exclusive;

/// This run installs macOS.
static ACTIVE: AtomicBool = AtomicBool::new(false);
/// The system image's bytes sent and its size as the restore host last
/// left them; a size of 0 before any image.
static SENT: AtomicU64 = AtomicU64::new(0);
static SIZE: AtomicU64 = AtomicU64::new(0);

/// The boot PE's own: the install's progress, the screen's, and when it
/// last looked.
struct State {
    progress: Progress,
    screen: Screen,
    looked: u64,
}

static STATE: Exclusive<Option<State>> = Exclusive::new(None);

/// This run installs macOS: its progress is logged, and shown while macOS
/// draws nothing new.
pub(super) fn begin() {
    ACTIVE.store(true, Ordering::Relaxed);
}

pub(super) fn active() -> bool {
    ACTIVE.load(Ordering::Relaxed)
}

/// The restore host has sent `sent` of the system image's `size` bytes.
pub(super) fn image(sent: u64, size: u64) {
    SIZE.store(size, Ordering::Relaxed);
    SENT.store(sent, Ordering::Relaxed);
}

/// The installer began `step` (`installer: ` taken off) at the boot PE's
/// counter `now`, which runs at `hz`: a new step is logged.
pub(super) fn step(step: &str, now: u64, hz: u64) {
    with(now, hz, |state| {
        if let Some(line) = state.progress.step(step, now) {
            output::arguments(format_args!("{line}\n"));
        }
    });
}

/// At the end of the boot PE's slice, at its counter `now`: once a second,
/// the image's progress, the minute's `INSTALL PROGRESS` line, and the
/// screen. The console takes the screen back only while the boot PE is
/// `quiet`, holding no lock another PE spins on, as it draws all of it.
pub(super) fn slice_end(now: u64, hz: u64, quiet: bool) {
    with(now, hz, |state| {
        if now.wrapping_sub(state.looked) < hz {
            return;
        }
        state.looked = now;
        let size = SIZE.load(Ordering::Relaxed);
        state
            .progress
            .image(SENT.load(Ordering::Relaxed), size, now);
        if let Some(line) = state.progress.log_due(now) {
            output::arguments(format_args!("{line}\n"));
        }
        let frames = display::presents();
        match state
            .screen
            .observe(frames, display::frames_shown(), now, hz)
        {
            Action::Take if quiet && display::show_progress() => {
                state.screen.taken(now);
                output::arguments(format_args!(
                    "INSTALL SCREEN shows the install's progress: macOS drew no new frame for \
                     {IDLE_SECONDS} s (frames={frames})\n"
                ));
            }
            Action::Returned { seconds } => {
                crate::status::panel(None);
                output::arguments(format_args!(
                    "INSTALL SCREEN gives the screen back to macOS's new frame after {seconds} s \
                     (frames={frames})\n"
                ));
            }
            Action::Take | Action::Keep => {}
        }
        if state.screen.shown() {
            crate::status::panel(Some(state.progress.snapshot(now).panel()));
        }
    });
}

/// Run `f` on the boot PE's state, made at the first call of an install.
fn with(now: u64, hz: u64, f: impl FnOnce(&mut State)) {
    if !active() {
        return;
    }
    STATE.try_with(|state| {
        f(state.get_or_insert_with(|| State {
            progress: Progress::new(now, hz),
            screen: Screen::new(now),
            looked: now,
        }))
    });
}
