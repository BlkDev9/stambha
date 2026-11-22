<h1 align="center">Stambha</h1>

<p align="center"><i>स्तम्भ (stambha): "pillar", as in the pillars of Ashoka, India's ancient inscribed stone pillars.</i></p>

Run Apple-silicon macOS on ordinary x86-64 PCs, desktops and laptops alike. Stambha boots from
a USB stick, translates macOS's arm64 code to x86-64 as it runs, and models the Apple hardware
macOS expects.

The goal: macOS that works like it belongs on your machine. That means **speed**, the hardware
you actually have (**Wi-Fi, Bluetooth, sound**), and on laptops the things a laptop needs
(battery, touchpad gestures, the lid, brightness and keyboard keys).

> [!WARNING]
> Experimental. It may not boot on your hardware, and it is slower than a real Mac.
> Back up anything you care about before installing.

## Status

| | |
|---|---|
| **Wi-Fi**: Realtek RTL8852CE (Wi-Fi 6E, found in many recent laptops): scan, connect, DHCP, HTTPS | Working on real hardware with WPA2 (tested before macOS starts); WPA3 implemented but not yet hardware-tested; the in-macOS test is in progress |
| **Install safety**: installs only into empty (unallocated) space, reads back every partition-table write, refuses unexpected layouts, and offers a "Check only" dry run that writes nothing | Done |
| **Install progress**: a progress line every minute (stage, percent, speed), so a long install never looks frozen | Done |
| Faster installs, Bluetooth (Realtek), sound, battery and charging in macOS, a Magic-Trackpad-style touchpad, power button / lid / brightness / keyboard backlight, a guest profiler and speed-ups | In development on branches; each lands in `main` after review and a hardware test |
| More Realtek Wi-Fi chips (RTL8852BE, RTL8851BE, RTL8852AE, RTL8922AE) and full Wi-Fi speed (aggregation, 80/160 MHz) | In development |
| AMD Radeon and Intel graphics | In development |

## Hardware requirements

| Part | Requirement |
|---|---|
| CPU | Intel or AMD x86-64-v2 or newer. AVX2 and AVX-512 are used when present. |
| Memory | 8 GB or more (16 GB recommended). |
| Firmware | 64-bit UEFI with Secure Boot off. |
| GPU | NVIDIA GeForce GTX 16 or RTX 20/30/40/50 series. On laptops with a MUX switch, use dGPU-only mode. |
| Disk | NVMe, or SATA in AHCI mode, with 60 GB or more of **unallocated** space (shrink a partition first and leave the space unallocated). |
| Wi-Fi | Realtek RTL8852CE (more coming). |
| Ethernet | Intel, Realtek, Broadcom, Aquantia, Qualcomm Atheros, and common USB adapters. |

## Getting it

Releases are coming soon. Until then, build from source:

```sh
git clone https://github.com/garvitsurana271/stambha
cd stambha
cargo xtask dist
```

You need [Rust](https://rustup.rs), git, Python 3.10 or newer, and LLVM 21-23 with Clang, LLD and
libclang. The first build fetches and compiles Linux, Mesa and LLVM sources and takes a long time.
It writes `target/release/stela`: the command keeps its original name until the rename to
`stambha` lands.

## Maintainer

**Garvit Surana** ([@garvitsurana271](https://github.com/garvitsurana271)).

## Credits

- Stambha is originally based on [Stela](https://github.com/tylertms/Stela) by Tyler Smith
  (tylertms): the translator, the Metal driver and the Apple silicon hardware model come from his
  work. His branches are kept here as `tyler/*`.
- The Wi-Fi driver is a Rust port of Linux's `rtw89` by Realtek and the Linux wireless developers.
  Its Linux sources are listed in [NOTICE](NOTICE).
- Mesa, Linux and LLVM: see [NOTICE](NOTICE).

## Legal

Stambha contains no Apple software. You are responsible for following Apple's software license
agreement and terms of use.

Stambha's code is GPL-2.0-or-later. Files whose SPDX line says otherwise keep their own terms. The
boot program contains Linux code and is distributed as a whole under GPL-2.0-only; NOTICE gives
the terms for each program. See [LICENSE](LICENSE) and [NOTICE](NOTICE).
