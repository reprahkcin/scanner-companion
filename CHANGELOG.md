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

