# Scanner Companion — Project Handoff

Date: 2025-09-22
Version: 1.0

## 1) Purpose and Scope
Scanner Companion is a Raspberry Pi–based control app for a 3D photogrammetry scanner. It coordinates two stepper motors via an Arduino (turntable rotation and linear focus rail) and a Pi Camera, providing:
- Manual motor controls with live preview
- A cardinal-point focus calibration flow (0°, 90°, 180°, 270°)
- Automated 360° capture with configurable focus slices per angle
- Camera settings control (exposure, brightness, white balance, resolution)
- Organized output with metadata and session folders

This document onboards non-code and code contributors with project context, current state, and where to help.

## 2) High-Level Architecture
- Raspberry Pi runs a Python/Tkinter GUI (`scanner_control.py`)
- Arduino (over USB serial) executes motion commands for two motors
  - Motor 1: Turntable rotation (degrees)
  - Motor 2: Linear focus rail (millimeters)
- Pi Camera (picamera2) provides preview and still capture

Data flow overview:
- GUI → Arduino: textual serial protocol (e.g., `ROTATE 1 15.0 CW`, `MOVE 2 5.0 FORWARD`, `ZERO 1`, `GET_POS 2`)
- Arduino → GUI: `OK` after motion, numeric strings for position queries, `ERR` on errors
- GUI → Filesystem: session folders, `metadata.json`, saved calibration data, and images

## 3) Repository Map (key paths)
- `scanner_control.py` — main Tkinter GUI application
- `arduino/scanner_controller.ino` — Arduino firmware handling move/rotate/position commands
- `docs/` — user-facing docs
  - `QUICKSTART.md` — setup and basic usage
  - `hardware_setup.md` — wiring; hardware overview
  - `FEATURES.md` — feature list
  - `handoff.md` — this document
- `examples/sample_calibration.json` — example of calibration data format
- `requirements.txt` — Python dependencies
- `run_scanner.sh` — convenience launcher
- `legacy/` — prior iterations and experiments (v1..v4, utilities)

## 4) How to Run
Prereqs (on Raspberry Pi OS):
- Python 3 with Tkinter
- `picamera2` + its OS-level dependencies
- `pip install -r requirements.txt` (includes Pillow and other Python packages)

Run options:
- `bash run_scanner.sh`
- or `python3 scanner_control.py`

If the camera or picamera2/Pillow aren’t available, the GUI will present status messages but keep running so you can test other parts.

## 5) Serial Protocol (Arduino ↔ GUI)
The GUI uses a simple, line-oriented text protocol. Examples:
- Rotation (Motor 1): `ROTATE 1 <degrees> <CW|CCW>`
- Linear move (Motor 2): `MOVE 2 <millimeters> <FORWARD|BACKWARD>`
- Get position: `GET_POS <motor>` → Arduino replies with a numeric string (position)
- Zero position: `ZERO <motor>`

Arduino replies `OK` after completing motion commands, a number for `GET_POS`, or `ERR` on failure.

Note: The app currently doesn’t require firmware-specific configuration commands; it performs calibration/compensation at the application level (see Section 7).

## 6) GUI Overview (tabs)
- Manual Control
  - Live preview (start/stop)
  - Jog controls for Motor 1 (rotation) and Motor 2 (focus)
  - Position readouts and step sizes
- Calibration
  - Wizard for cardinal angles (0°, 90°, 180°, 270°)
  - Capture “near” and “far” focus positions per angle
  - Save/load calibration JSON
- Capture
  - Specimen name, counts for perspectives (angles around 360°), focus slices/angle
  - Settling delay, image format
  - Rotation Mapping (App): scale and invert (see Section 7)
  - Start/stop full capture; progress bar; capture log
- Camera Settings
  - Shutter speed (or auto), ISO/gain, EV, brightness
  - Auto/manual white balance (WB gains)
  - Contrast, saturation, sharpness
  - Preview and capture resolutions
  - Camera info panel

## 7) Current State and Recent Changes
The project recently switched to a direct-drive stepper for the turntable. To avoid firmware churn, rotation calibration is now handled in the app:

- App-level rotation mapping
  - Scale: how many “firmware degrees” to command for 1° of desired physical rotation
  - Invert: flips CW/CCW if mechanical direction is reversed
- Where it’s applied
  - All rotation paths in `scanner_control.py` (manual jog, calibration, `move_to_angle`) use the mapping
- Persistence
  - Saved to `app_settings.json` under the current output directory
  - Automatically saved on change and on exit

Quick calibration of the scale:
- Command a known angle (e.g., 90°) using the UI
- Measure actual rotation on the turntable (e.g., 72°)
- Compute scale = desired / actual = 90 / 72 = 1.25
- Enter scale in Capture → Rotation Mapping (App)

The firmware retains its basic protocol (`ROTATE`, `MOVE`, `ZERO`, `GET_POS`). Any earlier firmware-side SPD/direction experiments are not required by the current app and can be ignored or removed later if desired.

## 8) Data & Output Layout
- Default base output: `~/Desktop/scanner_captures`
- Test captures: `test_captures/`
- Full sessions: `specimen_name/session_YYYYMMDD_HHMMSS/`
  - `metadata.json` — includes capture parameters and calibration data snapshot
  - `stack_00/`, `stack_01/`, ... — one folder per perspective/angle
  - Filenames: `stack_<perspective>_shot_<index>_angle_<deg>.<ext>`
- Calibration files: saved/loaded via Calibration tab (format similar to `examples/sample_calibration.json`)
- App settings: `app_settings.json` (rotation scale/invert), created next to the chosen output folder

## 9) Threading and Responsiveness
- The long-running capture sequence executes in a background thread
- GUI updates use `self.after(...)` to remain thread-safe
- A lock guards camera access during preview vs. capture
- Preview updates are temporarily paused during actual captures for performance

## 10) Known Limitations / Considerations
- Camera stack installation varies by Pi OS version; ensure `picamera2` works on your image
- Serial port default is `/dev/ttyACM0`; adjust if your Arduino enumerates differently
- No auto-detection for the serial port yet
- Rotation mapping is purely app-side; firmware EEPROM/config isn’t used for SPD
- Error handling is pragmatic (status bar, logs, dialogs) but not exhaustive

## 11) Where a Non-Code Contributor Can Help
- Documentation
  - Walkthroughs for direct-drive calibration (screenshots, step-by-step)
  - Troubleshooting guides (serial connection, camera setup)
  - A user-facing quick reference for Rotation Mapping (with examples)
- Process & Ops
  - Create a lightweight test plan (manual steps) for each release
  - Build a checklist for hardware setup and teardown
  - Record and maintain a “common pitfalls” FAQ
- Content & Assets
  - Annotated photos of the rig with labels matching `hardware_setup.md`
  - Short demo videos for calibration and capture workflows
  - Example datasets (public domain) and a gallery of expected results

## 12) Next Steps (Backlog Starters)
- App: Add a one-click “90° test” that suggests a rotation scale from measured outcome
- App: Serial port chooser UI + auto-detect common ports
- App: Option to home turntable on startup to re-sync logical position
- App: More robust error surface (status panel with levels, log export)
- Firmware: Confirm the minimal protocol set and prune any unused experimental commands
- CI/Release: Package a known-good Pi image recipe and a basic regression checklist

## 13) Contributing, License, and Contacts
- Contributing: see `CONTRIBUTING.md`
- License: see `LICENSE`
- For quick starts: see `docs/QUICKSTART.md` and `docs/hardware_setup.md`

If you’re picking up non-code tasks, start by running the app without hardware to learn the UI, then shadow one full calibration + capture with the hardware to document the process end to end.
