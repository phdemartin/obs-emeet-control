# OBS EMEET CamControl (Linux C++ Filter Plugin)

Native **OBS Studio** video filter plugin (`obs-emeet-control.so`) for **EMEET SmartCam S800** and UVC webcams on Linux.

Control your webcam hardware and apply **OpenCV face auto-framing**, **software HDR**, **3D LUTs**, and **mirror/flip** directly as an **OBS Filter** — **without needing a second virtual camera (`v4l2loopback`)**, so you can keep using OBS's built-in Virtual Camera for Zoom, Meet, or Teams!

---

## Features

- **Hardware V4L2/UVC Controls (0% CPU, live in OBS)**:
  - Auto-detects EMEET cameras (`/dev/video*`)
  - Hardware Zoom / FOV (`100` = 73° wide → `400` = 40° close-up)
  - Continuous Autofocus (PDAF+ToF) & Manual Focus
  - Exposure Mode (Auto / Manual / Shutter Priority) & Exposure Time
  - Auto White Balance & Manual Color Temperature (`2300K–7500K`)
  - Brightness, Contrast, Saturation, Hue, Sharpness
  - Anti-Flicker (`Disabled`, `50 Hz`, `60 Hz`) & Reset button
- **Software Effects (Inside OBS, no `v4l2loopback`)**:
  - **OpenCV Face Tracking (Auto-Framing)** with adaptive `2.8×` headroom (never cuts off the top/bottom of your head) and smooth EMA anti-jitter tracking
  - **Software HDR** (*Balanced* / *Strong* shadow lift & local contrast)
  - **3D LUTs** (supports both OBS 512×512 `.png` LUTs and `.cube` files)
  - **Flip Horizontal / Vertical**
  - **0% CPU Bypass**: when only hardware V4L2 controls are active, frames pass through untouched

---

## Installation

### Option 1: Debian / Ubuntu `.deb` Package (Recommended)
Download `obs-emeet-control_1.0.0_amd64.deb` from [Releases](https://github.com/phdemartin/obs-emeet-control/releases):
```bash
sudo apt install ./obs-emeet-control_1.0.0_amd64.deb
```

### Option 2: Portable OBS Plugin Archive (`.tar.gz`)
Extract `obs-emeet-control-1.0.0-linux-x86_64.tar.gz` into `~/.config/obs-studio/plugins/`:
```bash
mkdir -p ~/.config/obs-studio/plugins
tar -xzf obs-emeet-control-1.0.0-linux-x86_64.tar.gz -C ~/.config/obs-studio/plugins/
```

### Option 3: Build from Source
```bash
sudo apt install cmake g++ pkg-config libobs-dev libopencv-dev opencv-data
./scripts/install_local.sh
```

---

## Usage in OBS Studio

1. Restart **OBS Studio**.
2. Right-click your **Video Capture Device (V4L2)** source → **Filters**.
3. Under **Audio/Video Filters** (or Effect Filters), click **+** → **EMEET CamControl (V4L2 + Auto-Framing + HDR + LUT)**.
4. Adjust hardware controls and/or enable Face Tracking, Software HDR, or a 3D LUT.

---

## Companion Standalone App

Looking for a standalone Linux desktop app with System Tray, MP4 recording, and RTMP streaming? See [emeet-control](https://github.com/phdemartin/emeet-control).
