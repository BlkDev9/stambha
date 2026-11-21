//! An install's progress, for the boot log and the screen: the restore step
//! the guest's installer is on (its `CHECKPOINT BEGIN` lines), how much of
//! the system image the restore host has sent it (the ASR stream's offset
//! and size), and since when.
//!
//! macOS's own progress bar can stand still for an hour while the image
//! streams (upstream issues #127 and #168). So an install's boot log carries
//! an `INSTALL STAGE` line at each new step and an `INSTALL PROGRESS` line a
//! minute, each whole by itself: a log ring that kept only an install's
//! last minutes still says where it got to. And while macOS draws nothing
//! new, Stela's console takes the screen back to show the same ([`Screen`]).
use alloc::{format, string::String, vec, vec::Vec};

/// Seconds between two `INSTALL PROGRESS` lines.
pub const LOG_SECONDS: u64 = 60;
/// Seconds with no new frame from macOS before Stela's console shows the
/// install's progress in place of its last one.
pub const IDLE_SECONDS: u64 = 60;
/// Seconds with no new byte of the image before its stream counts as
/// waiting, with no rate and no time left.
pub const STALL_SECONDS: u64 = 30;
/// Seconds between two samples of the image's rate.
pub const SAMPLE_SECONDS: u64 = 5;
/// The rate's smoothing: each sample weighs its own length against this.
pub const SMOOTHING_SECONDS: u64 = 30;

/// Where an install is, as far as the system image says.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Phase {
    /// No system image yet: the installer prepares the disk and firmware.
    Preparing,
    /// The system image streams.
    Image,
    /// The whole image was sent: the installer finishes.
    Finishing,
}

/// How fast the image streams.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Rate {
    /// Not measured yet: no image, or no sample of it.
    Unknown,
    /// No new byte for `seconds`: the stream waits, on the guest's writes,
    /// the disk or the network.
    Waiting { seconds: u64 },
    /// Bytes a second, smoothed.
    BytesPerSecond(u64),
}

/// The install's progress at one time ([`Progress::snapshot`]).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Snapshot<'a> {
    pub phase: Phase,
    /// The installer's step, as the screen names it.
    pub step: Option<&'a str>,
    /// The image's bytes sent and its size.
    pub image: Option<(u64, u64)>,
    pub percent: Option<u8>,
    pub rate: Rate,
    /// Seconds until the whole image is sent, at the rate.
    pub eta: Option<u64>,
    /// Seconds since the install started.
    pub elapsed: u64,
}

/// An install's step and image over time.
pub struct Progress {
    hz: u64,
    started: u64,
    step: Option<String>,
    image: Option<Image>,
    /// When the next `INSTALL PROGRESS` line is due.
    next_log: u64,
}

/// The system image as the restore host sends it.
struct Image {
    done: u64,
    total: u64,
    /// When the rate was last sampled, and the bytes sent then.
    sampled_at: u64,
    sampled: u64,
    /// When the last new byte was seen.
    advanced_at: u64,
    /// Bytes a second, smoothed; none before the first sample.
    rate: Option<u64>,
}

impl Image {
    fn new(done: u64, total: u64, now: u64) -> Self {
        Self {
            done,
            total,
            sampled_at: now,
            sampled: done,
            advanced_at: now,
            rate: None,
        }
    }
}

impl Progress {
    /// An install that started at counter `now`, which runs at `hz`.
    pub fn new(now: u64, hz: u64) -> Self {
        let hz = hz.max(1);
        Self {
            hz,
            started: now,
            step: None,
            image: None,
            next_log: now.saturating_add(LOG_SECONDS * hz),
        }
    }

    /// The installer began `step` at `now`: the `INSTALL STAGE` line when
    /// it is not the step it was on.
    pub fn step(&mut self, step: &str, now: u64) -> Option<String> {
        let step = step.trim();
        if step.is_empty() || self.step.as_deref() == Some(step) {
            return None;
        }
        self.step = Some(String::from(step));
        Some(format!(
            "INSTALL STAGE step={} elapsed={} s image={}",
            token(step),
            self.seconds(now),
            image_field(self.image.as_ref().map(|image| (image.done, image.total)))
        ))
    }

    /// The restore host had sent `done` of the image's `total` bytes at
    /// `now`. A size of 0 is no image; fewer bytes than before, or another
    /// size, an image sent anew, whose rate is measured from the start.
    pub fn image(&mut self, done: u64, total: u64, now: u64) {
        if total == 0 {
            return;
        }
        let done = done.min(total);
        let hz = self.hz;
        let same = |image: &Image| image.total == total && done >= image.done;
        let Some(image) = self.image.as_mut().filter(|image| same(image)) else {
            self.image = Some(Image::new(done, total, now));
            return;
        };
        if done > image.done {
            image.done = done;
            image.advanced_at = now;
        }
        let length = now.saturating_sub(image.sampled_at);
        if length < SAMPLE_SECONDS * hz {
            return;
        }
        let sample = u128::from(done - image.sampled) * u128::from(hz) / u128::from(length);
        // Each sample weighs its length against the smoothing's.
        let smoothing = u128::from(SMOOTHING_SECONDS * hz);
        let rate = match image.rate {
            None => sample,
            Some(rate) => {
                (u128::from(rate) * smoothing + sample * u128::from(length))
                    / (smoothing + u128::from(length))
            }
        };
        image.rate = Some(u64::try_from(rate).unwrap_or(u64::MAX));
        image.sampled_at = now;
        image.sampled = done;
    }

    /// Where the install is at `now`.
    pub fn snapshot(&self, now: u64) -> Snapshot<'_> {
        let mut snapshot = Snapshot {
            phase: Phase::Preparing,
            step: self.step.as_deref(),
            image: None,
            percent: None,
            rate: Rate::Unknown,
            eta: None,
            elapsed: self.seconds(now),
        };
        let Some(image) = &self.image else {
            return snapshot;
        };
        snapshot.image = Some((image.done, image.total));
        snapshot.percent = crate::screen::percentage(image.done, image.total);
        if image.done == image.total {
            snapshot.phase = Phase::Finishing;
            snapshot.rate = image.rate.map_or(Rate::Unknown, Rate::BytesPerSecond);
            snapshot.eta = Some(0);
            return snapshot;
        }
        snapshot.phase = Phase::Image;
        let still = now.saturating_sub(image.advanced_at) / self.hz;
        snapshot.rate = match image.rate {
            _ if still >= STALL_SECONDS => Rate::Waiting { seconds: still },
            None => Rate::Unknown,
            Some(0) => Rate::Waiting { seconds: still },
            Some(rate) => Rate::BytesPerSecond(rate),
        };
        if let Rate::BytesPerSecond(rate) = snapshot.rate {
            snapshot.eta = Some((image.total - image.done).div_ceil(rate));
        }
        snapshot
    }

    /// The `INSTALL PROGRESS` line, once each [`LOG_SECONDS`]: after a gap,
    /// one line and the next a minute on.
    pub fn log_due(&mut self, now: u64) -> Option<String> {
        if now < self.next_log {
            return None;
        }
        let minute = LOG_SECONDS * self.hz;
        self.next_log = self.next_log.saturating_add(minute);
        if self.next_log <= now {
            self.next_log = now.saturating_add(minute);
        }
        Some(self.snapshot(now).log_line())
    }

    fn seconds(&self, now: u64) -> u64 {
        now.saturating_sub(self.started) / self.hz
    }
}

const MIB: u64 = 1 << 20;

/// A step's name as one word of a log line.
fn token(step: &str) -> String {
    step.split_whitespace().collect::<Vec<_>>().join("_")
}

/// `done/total MiB (percent%)`, or `unknown` with no image.
fn image_field(image: Option<(u64, u64)>) -> String {
    match image {
        None => String::from("unknown"),
        Some((done, total)) => match crate::screen::percentage(done, total) {
            Some(percent) => format!("{}/{} MiB ({percent}%)", done / MIB, total / MIB),
            None => format!("{}/{} MiB", done / MIB, total / MIB),
        },
    }
}

/// Bytes a second as MiB/s to a tenth.
fn mib_per_second(rate: u64) -> String {
    let tenths = (u128::from(rate) * 10 + u128::from(MIB / 2)) / u128::from(MIB);
    format!("{}.{} MiB/s", tenths / 10, tenths % 10)
}

/// Whole minutes and hours, as people read a time.
fn minutes(minutes: u64) -> String {
    match minutes {
        0..60 => format!("{minutes} min"),
        _ => format!("{} h {} min", minutes / 60, minutes % 60),
    }
}

impl Snapshot<'_> {
    /// The `INSTALL PROGRESS` line: one word a field, so a log keeps each
    /// whole.
    pub fn log_line(&self) -> String {
        let rate = match self.rate {
            Rate::BytesPerSecond(rate) => mib_per_second(rate),
            Rate::Waiting { .. } => String::from("0.0 MiB/s"),
            Rate::Unknown => String::from("unknown"),
        };
        let eta = match (self.eta, self.rate) {
            (Some(seconds), _) => format!("{} min", seconds.div_ceil(60)),
            (None, Rate::Waiting { .. }) => String::from("waiting"),
            (None, _) => String::from("unknown"),
        };
        format!(
            "INSTALL PROGRESS step={} image={} rate={rate} eta={eta} elapsed={} min",
            self.step.map_or_else(|| String::from("none"), token),
            image_field(self.image),
            self.elapsed / 60
        )
    }

    /// The lines Stela's console shows under its running step while macOS
    /// draws nothing new: plain ASCII, as its font has no more.
    pub fn panel(&self) -> Vec<String> {
        let image = match (self.phase, self.image) {
            (Phase::Finishing, Some(_)) => format!("{}, all sent", self.image_of()),
            (_, Some(_)) => self.image_of(),
            (_, None) => String::from("not started yet"),
        };
        let speed = match (self.phase, self.rate) {
            (Phase::Preparing, _) => String::from("not measured yet"),
            (_, Rate::BytesPerSecond(rate)) => mib_per_second(rate),
            (Phase::Finishing, _) => String::from("not measured"),
            (_, Rate::Waiting { seconds }) => format!(
                "waiting for data (none for {})",
                match seconds {
                    0..120 => format!("{seconds} s"),
                    _ => minutes(seconds / 60),
                }
            ),
            (_, Rate::Unknown) => String::from("measuring"),
        };
        let left = match (self.phase, self.rate, self.eta) {
            (Phase::Preparing, ..) => String::from("unknown until the image starts"),
            (Phase::Finishing, ..) => String::from("the installer is finishing"),
            (_, _, Some(seconds)) => format!("about {}", minutes(seconds.div_ceil(60))),
            (_, Rate::Waiting { .. }, None) => String::from("unknown while it waits"),
            (..) => String::from("not known yet"),
        };
        let elapsed = match self.elapsed {
            0..60 => String::from("under 1 min"),
            seconds => minutes(seconds / 60),
        };
        let field = |label: &str, value: &str| format!("{label:<15}{value}");
        vec![
            String::from("macOS is being installed. This is normal; installs can take 1-2 hours."),
            String::new(),
            field("Step:", self.step.unwrap_or("starting")),
            field("System image:", &image),
            field("Speed:", &speed),
            field("Time left:", &left),
            field("Elapsed:", &elapsed),
            String::new(),
            String::from(
                "macOS has drawn nothing new for a minute; its screen comes back when it does.",
            ),
        ]
    }

    /// `done of total MiB (percent%)`.
    fn image_of(&self) -> String {
        let (done, total) = self.image.unwrap_or_default();
        match self.percent {
            Some(percent) => format!("{} of {} MiB ({percent}%)", done / MIB, total / MIB),
            None => format!("{} of {} MiB", done / MIB, total / MIB),
        }
    }
}

/// What the screen should do ([`Screen::observe`]).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Action {
    /// Nothing changes.
    Keep,
    /// Take the screen back for Stela's console: macOS has drawn no new
    /// frame for [`IDLE_SECONDS`]. Once it has, [`Screen::taken`].
    Take,
    /// macOS's frames have the screen again, after the console showed the
    /// progress for `seconds`.
    Returned { seconds: u64 },
}

/// When Stela's console shows an install's progress in place of macOS's
/// last frame: once macOS has drawn nothing new for [`IDLE_SECONDS`], until
/// its next frame takes the screen back.
pub struct Screen {
    /// The frames presented when last seen, and since when.
    frames: u64,
    changed_at: u64,
    /// Since when the console shows the progress.
    shown: Option<u64>,
}

impl Screen {
    pub fn new(now: u64) -> Self {
        Self {
            frames: 0,
            changed_at: now,
            shown: None,
        }
    }

    /// At `now`, macOS has presented `frames` so far, and its frames hold
    /// the screen (`frames_hold`) or Stela's console does: before macOS's
    /// first frame, the console shows the boot's steps, and nothing is
    /// taken.
    pub fn observe(&mut self, frames: u64, frames_hold: bool, now: u64, hz: u64) -> Action {
        if frames != self.frames {
            self.frames = frames;
            self.changed_at = now;
        }
        let hz = hz.max(1);
        match self.shown {
            Some(since) if frames_hold => {
                self.shown = None;
                Action::Returned {
                    seconds: now.saturating_sub(since) / hz,
                }
            }
            Some(_) => Action::Keep,
            None if frames_hold && now.saturating_sub(self.changed_at) >= IDLE_SECONDS * hz => {
                Action::Take
            }
            None => Action::Keep,
        }
    }

    /// The console took the screen at `now`.
    pub fn taken(&mut self, now: u64) {
        self.shown = Some(now);
    }

    /// Whether the console shows the progress.
    pub fn shown(&self) -> bool {
        self.shown.is_some()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const HZ: u64 = 1000;
    const MIB: u64 = 1 << 20;

    fn at(seconds: u64) -> u64 {
        seconds * HZ
    }

    #[test]
    fn an_install_with_no_image_yet_is_preparing_and_measures_nothing() {
        let progress = Progress::new(at(10), HZ);
        let snapshot = progress.snapshot(at(135));
        assert_eq!(
            snapshot,
            Snapshot {
                phase: Phase::Preparing,
                step: None,
                image: None,
                percent: None,
                rate: Rate::Unknown,
                eta: None,
                elapsed: 125,
            }
        );
        assert_eq!(
            snapshot.log_line(),
            "INSTALL PROGRESS step=none image=unknown rate=unknown eta=unknown elapsed=2 min"
        );
    }

    #[test]
    fn each_new_step_is_logged_once_with_the_time_and_image() {
        let mut progress = Progress::new(0, HZ);
        assert_eq!(
            progress.step("fdr create", at(3)).as_deref(),
            Some("INSTALL STAGE step=fdr_create elapsed=3 s image=unknown")
        );
        // The guest begins a step again, as `load sep os` repeats: no line.
        assert_eq!(progress.step("fdr create", at(4)), None);
        assert_eq!(progress.step("  ", at(4)), None);
        progress.image(512 * MIB, 2048 * MIB, at(70));
        assert_eq!(
            progress.step("load sep os", at(75)).as_deref(),
            Some("INSTALL STAGE step=load_sep_os elapsed=75 s image=512/2048 MiB (25%)")
        );
        assert_eq!(progress.snapshot(at(76)).step, Some("load sep os"));
        // Back to an earlier step is a change too.
        assert!(progress.step("fdr create", at(80)).is_some());
    }

    #[test]
    fn the_rate_is_a_smoothed_average_that_gives_the_time_left() {
        let mut progress = Progress::new(0, HZ);
        progress.step("restore image", at(0));
        progress.image(0, 1000 * MIB, at(0));
        // Closer than a sample's length, nothing is measured yet.
        progress.image(20 * MIB, 1000 * MIB, at(2));
        assert_eq!(progress.snapshot(at(2)).rate, Rate::Unknown);
        assert_eq!(progress.snapshot(at(2)).eta, None);
        assert_eq!(progress.snapshot(at(2)).phase, Phase::Image);
        // The first sample is the rate: 50 MiB in 5 s.
        progress.image(50 * MIB, 1000 * MIB, at(5));
        let first = progress.snapshot(at(5));
        assert_eq!(first.rate, Rate::BytesPerSecond(10 * MIB));
        assert_eq!(first.percent, Some(5));
        assert_eq!(first.image, Some((50 * MIB, 1000 * MIB)));
        assert_eq!(first.eta, Some(95));
        assert_eq!(
            first.log_line(),
            "INSTALL PROGRESS step=restore_image image=50/1000 MiB (5%) rate=10.0 MiB/s \
             eta=2 min elapsed=0 min"
        );
        // A slow 5 s moves the average a sixth of the way: 5 s against 30.
        progress.image(60 * MIB, 1000 * MIB, at(10));
        let second = progress.snapshot(at(10));
        let expected = (u128::from(10 * MIB) * 30 + u128::from(2 * MIB) * 5) / 35;
        assert_eq!(second.rate, Rate::BytesPerSecond(expected as u64));
        assert!(second.log_line().contains(" rate=8.9 MiB/s "));
        let left = (940 * MIB).div_ceil(expected as u64);
        assert_eq!(second.eta, Some(left));
    }

    #[test]
    fn a_stalled_image_is_waiting_with_no_time_left_until_bytes_come_again() {
        let mut progress = Progress::new(0, HZ);
        progress.image(0, 100 * MIB, at(0));
        progress.image(10 * MIB, 100 * MIB, at(5));
        for second in (10..=34).step_by(5) {
            progress.image(10 * MIB, 100 * MIB, at(second));
        }
        // 29 s without a byte: slower, still a rate.
        assert!(matches!(
            progress.snapshot(at(34)).rate,
            Rate::BytesPerSecond(rate) if rate > 0 && rate < 2 * MIB
        ));
        progress.image(10 * MIB, 100 * MIB, at(35));
        let stalled = progress.snapshot(at(35));
        assert_eq!(stalled.rate, Rate::Waiting { seconds: 30 });
        assert_eq!(stalled.eta, None);
        assert_eq!(stalled.phase, Phase::Image);
        assert!(
            stalled
                .log_line()
                .contains(" image=10/100 MiB (10%) rate=0.0 MiB/s eta=waiting ")
        );
        // Bytes again: a rate again.
        progress.image(12 * MIB, 100 * MIB, at(40));
        assert!(matches!(
            progress.snapshot(at(40)).rate,
            Rate::BytesPerSecond(_)
        ));
    }

    #[test]
    fn an_empty_total_is_no_image_and_a_full_one_is_finishing() {
        let mut progress = Progress::new(0, HZ);
        progress.image(5, 0, at(1));
        assert_eq!(progress.snapshot(at(1)).phase, Phase::Preparing);
        progress.image(0, 100 * MIB, at(1));
        progress.image(40 * MIB, 100 * MIB, at(6));
        // More than the size counts as all of it.
        progress.image(300 * MIB, 100 * MIB, at(11));
        let done = progress.snapshot(at(11));
        assert_eq!(done.phase, Phase::Finishing);
        assert_eq!(done.percent, Some(100));
        assert_eq!(done.image, Some((100 * MIB, 100 * MIB)));
        assert_eq!(done.eta, Some(0));
        assert!(done.log_line().contains(" image=100/100 MiB (100%) "));
        assert!(done.log_line().contains(" eta=0 min "));
        // Finished, it is never waiting, however long the installer takes.
        assert_ne!(
            progress.snapshot(at(4000)).rate,
            Rate::Waiting { seconds: 3989 }
        );
        assert_eq!(progress.snapshot(at(4000)).eta, Some(0));
    }

    #[test]
    fn an_image_sent_again_or_another_image_measures_its_rate_anew() {
        let mut progress = Progress::new(0, HZ);
        progress.image(0, 100 * MIB, at(0));
        progress.image(50 * MIB, 100 * MIB, at(5));
        assert!(matches!(
            progress.snapshot(at(5)).rate,
            Rate::BytesPerSecond(_)
        ));
        // Fewer bytes than before: the stream started over.
        progress.image(MIB, 100 * MIB, at(6));
        assert_eq!(progress.snapshot(at(6)).rate, Rate::Unknown);
        assert_eq!(progress.snapshot(at(6)).percent, Some(1));
        progress.image(11 * MIB, 100 * MIB, at(11));
        assert_eq!(
            progress.snapshot(at(11)).rate,
            Rate::BytesPerSecond(2 * MIB)
        );
        // Another size: another image.
        progress.image(11 * MIB, 300 * MIB, at(12));
        assert_eq!(progress.snapshot(at(12)).rate, Rate::Unknown);
        assert_eq!(progress.snapshot(at(12)).image, Some((11 * MIB, 300 * MIB)));
    }

    #[test]
    fn the_progress_line_comes_once_a_minute_and_catches_up_once() {
        let mut progress = Progress::new(at(100), HZ);
        assert_eq!(progress.log_due(at(159)), None);
        let line = progress.log_due(at(160)).unwrap();
        assert!(line.starts_with("INSTALL PROGRESS step=none "));
        assert!(line.ends_with(" elapsed=1 min"));
        assert_eq!(progress.log_due(at(161)), None);
        assert!(progress.log_due(at(220)).is_some());
        // A long gap gives one line, and the next a minute after it.
        assert!(progress.log_due(at(500)).is_some());
        assert_eq!(progress.log_due(at(501)), None);
        assert_eq!(progress.log_due(at(559)), None);
        assert!(progress.log_due(at(560)).is_some());
    }

    #[test]
    fn the_panel_names_the_step_the_image_the_rate_and_that_this_is_normal() {
        let mut progress = Progress::new(0, HZ);
        progress.step("load sep os", at(0));
        progress.image(0, 1000 * MIB, at(8150));
        progress.image(50 * MIB, 1000 * MIB, at(8155));
        let panel = progress.snapshot(at(8160)).panel();
        assert!(panel.iter().all(|line| line.is_ascii()));
        assert!(
            panel[0].contains("This is normal; installs can take 1-2 hours."),
            "{panel:?}"
        );
        for expected in [
            "Step:          load sep os",
            "System image:  50 of 1000 MiB (5%)",
            "Speed:         10.0 MiB/s",
            "Time left:     about 2 min",
            "Elapsed:       2 h 16 min",
        ] {
            assert!(
                panel.iter().any(|line| line == expected),
                "{expected:?} in {panel:?}"
            );
        }
        assert!(panel.last().unwrap().contains("macOS"));
    }

    #[test]
    fn the_panel_says_when_the_image_waits_is_not_started_or_is_sent() {
        let mut progress = Progress::new(0, HZ);
        let preparing = progress.snapshot(at(30)).panel();
        for expected in [
            "Step:          starting",
            "System image:  not started yet",
            "Speed:         not measured yet",
            "Time left:     unknown until the image starts",
            "Elapsed:       under 1 min",
        ] {
            assert!(
                preparing.iter().any(|line| line == expected),
                "{expected:?} in {preparing:?}"
            );
        }
        progress.image(0, 100 * MIB, at(60));
        progress.image(10 * MIB, 100 * MIB, at(65));
        let waiting = progress.snapshot(at(245)).panel();
        for expected in [
            "Speed:         waiting for data (none for 3 min)",
            "Time left:     unknown while it waits",
            "Elapsed:       4 min",
        ] {
            assert!(
                waiting.iter().any(|line| line == expected),
                "{expected:?} in {waiting:?}"
            );
        }
        progress.image(100 * MIB, 100 * MIB, at(250));
        let sent = progress.snapshot(at(250)).panel();
        for expected in [
            "System image:  100 of 100 MiB (100%), all sent",
            "Time left:     the installer is finishing",
        ] {
            assert!(
                sent.iter().any(|line| line == expected),
                "{expected:?} in {sent:?}"
            );
        }
    }

    #[test]
    fn the_console_shows_the_progress_only_after_a_minute_with_no_new_frame() {
        let mut screen = Screen::new(at(0));
        // Before macOS's first frame the console has the screen anyway.
        assert_eq!(screen.observe(0, false, at(500), HZ), Action::Keep);
        assert_eq!(screen.observe(1, true, at(510), HZ), Action::Keep);
        assert_eq!(screen.observe(1, true, at(569), HZ), Action::Keep);
        assert_eq!(screen.observe(1, true, at(570), HZ), Action::Take);
        // Not taken (the screen was busy): asked again.
        assert_eq!(screen.observe(1, true, at(571), HZ), Action::Take);
        assert!(!screen.shown());
        screen.taken(at(571));
        assert!(screen.shown());
        assert_eq!(screen.observe(1, false, at(600), HZ), Action::Keep);
        // macOS's next frame takes the screen back.
        assert_eq!(
            screen.observe(2, true, at(631), HZ),
            Action::Returned { seconds: 60 }
        );
        assert!(!screen.shown());
        // And a minute must pass from that frame before the next take.
        assert_eq!(screen.observe(2, true, at(690), HZ), Action::Keep);
        assert_eq!(screen.observe(2, true, at(691), HZ), Action::Take);
    }

    #[test]
    fn frames_that_keep_coming_keep_the_screen_on_macos() {
        let mut screen = Screen::new(at(0));
        for second in 1..600 {
            assert_eq!(
                screen.observe(second / 30, true, at(second), HZ),
                Action::Keep
            );
        }
    }
}
