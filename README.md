<p align="center">
  <img src="assets/stambha-icon.svg" width="140" alt="Stambha">
</p>

<h1 align="center">Stambha</h1>

<p align="center">
  <b>Run Apple-silicon macOS on your PC.</b><br>
  Desktops and laptops. NVIDIA graphics. Installs over Ethernet or Wi-Fi.
</p>

<p align="center">
  <a href="https://discord.gg/M66z9fPJy5"><img src="https://img.shields.io/discord/1574111611348590602?label=Discord&logo=discord&logoColor=white&color=5865F2" alt="Discord"></a>
  <a href="LICENSE"><img src="https://img.shields.io/badge/license-GPL--2.0--or--later-2B3A8C" alt="License: GPL-2.0-or-later"></a>
  <a href="https://github.com/garvitsurana271/stambha/stargazers"><img src="https://img.shields.io/github/stars/garvitsurana271/stambha?color=F07A28" alt="Stars"></a>
</p>
<p align="center">
  <img src="https://img.shields.io/badge/runs-macOS%2027-111111?logo=apple&logoColor=white" alt="Runs macOS 27">
  <img src="https://img.shields.io/badge/on-x86--64%20UEFI%20PCs-2B3A8C" alt="On x86-64 UEFI PCs">
  <img src="https://img.shields.io/badge/GPU-NVIDIA%20GTX%2016%20%7C%20RTX%2020%E2%80%9350-76B900?logo=nvidia&logoColor=white" alt="NVIDIA GTX 16, RTX 20-50">
  <img src="https://img.shields.io/badge/written%20in-Rust-B7410E?logo=rust&logoColor=white" alt="Written in Rust">
  <a href="https://github.com/garvitsurana271/stambha/commits/main"><img src="https://img.shields.io/github/last-commit/garvitsurana271/stambha?color=FFD27A" alt="Last commit"></a>
  <a href="https://github.com/garvitsurana271/stambha/issues"><img src="https://img.shields.io/github/issues/garvitsurana271/stambha?color=5FE3C8" alt="Open issues"></a>
</p>

<p align="center"><i>स्तम्भ (stambha): "pillar", as in the Ashoka pillars.</i></p>

---

Stambha boots from a USB stick and runs the **Apple-silicon (arm64) version of macOS** on an ordinary x86-64 PC. It
translates macOS's Arm code to x86 as it runs, drives your NVIDIA GPU through its own Metal driver, and recreates the
Apple hardware macOS expects. No Mac, no Hackintosh kexts, no virtual machine.

> [!WARNING]
> Experimental. It may not boot on your hardware, and it is slower than a real Mac.
> Back up anything you care about before installing.

## ✨ What works

| | |
|---|---|
| 🖥️ **macOS on your PC** | Apple-silicon macOS boots, with NVIDIA GPU acceleration (GTX 16, RTX 20/30/40/50) |
| 📶 **Install over Wi-Fi** | No Ethernet cable needed with a Realtek RTL8852CE (Wi-Fi 6E, in many recent laptops) |
| 🛡️ **A safe installer** | Only ever uses empty space on your disk; see [below](#-safe-for-the-system-you-already-have) |
| 📈 **Install progress** | A progress line every minute (stage, percent, speed), so a long install never looks frozen |

## 🔧 Coming in the first release

- ⚡ **Speed:** Apple's matrix unit (AMX) running natively on your CPU, faster translator paths, faster installs
- 📶 **Wi-Fi inside macOS**, and a Wi-Fi picker right in the boot menu with saved networks
- 💻 **Laptops:** battery level and charging, a Magic-Trackpad-style touchpad, brightness and keyboard-backlight keys,
  power button and lid, audio fixes
- 🟦 **Bluetooth** (Realtek)
- 🧩 **Fixes for known issues** and the best community contributions

Next after that: AMD and Intel graphics, more Wi-Fi chips and full Wi-Fi speed, and laptops presented as a MacBook.

## 🛡️ Safe for the system you already have

Installing macOS next to Windows or Linux should never risk them. Stambha's installer:

- uses **only unallocated space**: it never resizes, moves or deletes a partition;
- checks the partition table it is about to write against **both** GPT copies, and refuses an unexpected layout;
- **reads back every write** before making the next one;
- offers **Check only**: a full dry run of the install plan that writes nothing at all.

## 📋 Requirements

| Part | Requirement |
|---|---|
| CPU | Intel or AMD x86-64-v2 or newer (Intel Nehalem, AMD Bulldozer). AVX2 and AVX-512 are used when present. |
| Memory | 8 GB or more (16 GB recommended) |
| Firmware | 64-bit UEFI with Secure Boot off |
| GPU | NVIDIA GeForce GTX 16 or RTX 20/30/40/50 series. Laptops with a MUX switch: use dGPU-only mode. |
| Disk | NVMe, or SATA in AHCI mode, with **60 GB or more of unallocated space** (shrink a partition, leave the space empty) |
| Network | Ethernet (Intel, Realtek, Broadcom, Aquantia, Qualcomm Atheros, common USB adapters) or Realtek RTL8852CE Wi-Fi |
| USB stick | 8 GB or more |

## 🚀 Getting it

The first release is on its way. **[Join the Discord](https://discord.gg/M66z9fPJy5)** to get it the day it lands, to
test early builds, and to get help.

<details>
<summary>Build from source</summary>

```sh
git clone https://github.com/garvitsurana271/stambha
cd stambha
cargo xtask dist
```

You need [Rust](https://rustup.rs), git, Python 3.10 or newer, and LLVM 21-23 with Clang, LLD and libclang. The first
build fetches and compiles Linux, Mesa and LLVM sources and takes a long time. It writes `target/release/stela`: the
command keeps its original name until the rename to `stambha` lands.
</details>

## ❓ Questions

Answers to the common ones (AMD/Intel graphics, hybrid-graphics laptops, Wi-Fi chips, virtual machines, how long an
install takes) are in the **#faq** channel on [Discord](https://discord.gg/M66z9fPJy5). Bugs and hardware reports:
[open an issue](https://github.com/garvitsurana271/stambha/issues/new/choose).

## 🤝 Contributing

Pull requests are welcome: see [CONTRIBUTING.md](CONTRIBUTING.md). Testing on your hardware and posting a hardware
report helps just as much.

## Maintainer

**Garvit Surana** ([@garvitsurana271](https://github.com/garvitsurana271))

## Credits

- Stambha is built on [Stela](https://github.com/tylertms/Stela) by Tyler Smith (tylertms): the translator, the Metal
  driver and the Apple-silicon hardware model come from his work. Thank you, Tyler.
- The Wi-Fi driver is a Rust port of Linux's `rtw89` by Realtek and the Linux wireless developers.
- Mesa, Linux and LLVM: see [NOTICE](NOTICE).
- Everyone who contributes code, tests and reports.

## Legal

Stambha contains no Apple software. You are responsible for following Apple's software license agreement and terms of
use.

Stambha's code is GPL-2.0-or-later. Files whose SPDX line says otherwise keep their own terms. The boot program contains
Linux code and is distributed as a whole under GPL-2.0-only; NOTICE gives the terms for each program. See
[LICENSE](LICENSE) and [NOTICE](NOTICE).
