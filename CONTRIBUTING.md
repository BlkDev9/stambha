# Contributing to Stambha

Thanks for helping! There are three ways in, and all of them matter:

- **Test and report.** Run Stambha on your hardware and post a
  [hardware report](https://github.com/garvitsurana271/stambha/issues/new/choose), working or not.
- **Help others** in the [Discord](https://discord.gg/M66z9fPJy5) #help forum.
- **Send code.** Read on.

## Building

See "Build from source" in the [README](README.md). Host tests run with `cargo test -p <crate>`; the boot image builds
with `cargo xtask dist`.

## How the code is written

- **Explain the why.** Items carry doc comments that say what they do, why, and which Linux or Apple behavior they
  follow (cite the file and line when porting).
- **Tests are sentences.** Name a test after the behavior it proves, e.g.
  `a_saved_line_reads_back_as_its_network`.
- **Commit titles** follow `type(area): what it does, where it fixes something` (`fix(wifi): ...`, `feat(boot): ...`).
- **New behavior is opt-in.** A new feature gets a boot option and stays off until it is proven on hardware. Bug fixes
  apply by default.
- Run `rustfmt` on what you change.

## The safety rules

Some code can hurt a user's data or silently give wrong answers, so it has a higher bar:

- **Disks and the installer.** Stambha only ever writes into unallocated space, checks its plan against both GPT copies,
  reads back every write before the next one, and never trims. A change here must keep all of that, with tests.
- **The translator and AMX.** A faster path must produce exactly the same results as the path it replaces: prove it
  with the differential tests against the reference model, including NaNs, infinities, subnormals and rounding ties.
- **Privacy.** Logs, tests and screenshots never carry serial numbers, MAC addresses, Wi-Fi names, IP addresses or
  other personal data.

## Pull requests

Keep each PR to one change, describe how you tested it, and fill in the checklist. Hardware changes are tested on real
hardware before they ship; say which machine you used.

## Credits

Your commits keep your name. Contributors get the Contributor role on Discord.
