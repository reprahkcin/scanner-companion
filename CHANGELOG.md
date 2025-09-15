# Changelog

All notable changes to this project will be documented in this file.

## [1.0.0] - 2025-09-14

### Added
- Angle-step indicator in Capture settings showing `360 / perspectives`.
- Robust calibration load: accept JSON with string angle keys (e.g., "0","90"), normalize to integers.
- Calibration Reset / Start Over button and workflow.

### Changed
- Capture semantics clarified: 
  - Perspectives (angles) = number of angles around 360°.
  - Focus slices per angle = number of focus steps captured at each angle.
- Capture loop order switched to perspective-major: move to angle, then capture all focus slices.
- UI labels updated to reflect perspectives/slices terminology.
- Live preview color corrected (optional BGR→RGB swap) without altering saved images.

### Fixed
- Serial command handling now blocks until Arduino returns `OK`, ensuring sequential motion.
- Preview throttling and camera access locking to prevent UI freezes.
- Calibration file loading failures due to string keys.

### Notes
- Metadata includes both legacy keys (`stacks`, `shots_per_stack`) and clarified fields (`perspectives`, `focus_slices_per_perspective`, `angle_step_degrees`).
- Folder naming remains `stack_XX` to preserve compatibility, representing perspective folders.


## [1.0.1] - 2025-09-14

### Added
- Camera Settings tab extended with controls for:
  - Auto Exposure toggle, ISO presets, Analogue Gain, EV compensation
  - Auto White Balance toggle, manual White Balance Red/Blue gains
  - Brightness, Contrast, Saturation, Sharpness
- Resolution presets for Preview and Capture with automatic preview reconfigure.

### Changed
- Capture now uses still-mode configuration per selected capture resolution, then returns to preview mode.
- Camera info panel shows camera properties and current metadata when available.

### Fixed
- Structural/indentation issues in the Camera Settings UI code resolved; UI renders reliably.

### Maintenance
- Removed an empty/failing GitHub Actions workflow (`.github/workflows/code-quality.yml`); retained a minimal basic syntax check on push/PR.


## [1.0.2] - 2025-09-15

### Maintenance
- Temporarily removed all GitHub Actions workflows to stop failure emails while we stabilize the project CI on Raspberry Pi-specific dependencies.
- We'll reintroduce a minimal CI later (e.g., pure syntax check or docs-only jobs) that doesn't depend on Pi hardware.

