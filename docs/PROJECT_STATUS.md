# Project Status

Last updated: 2026-05-31

## Current (Functional and Supported)

- Primary application: `scanner_control.py`
- Primary firmware: `arduino/scanner_controller/scanner_controller.ino`
- Active hardware profile: `profiles/raspberry_pi_3axis.json`
- Documentation set: `docs/current/`

This is the only workflow considered functional right now.

## Future (Planned, Not Yet Fully Implemented)

- Sherline/LinuxCNC/MKII direction
- Future profile: `profiles/sherline_6axis.json`
- Planning docs in `docs/future/`

Important: Future docs may describe target architecture and proposed files that do not yet exist.

## Archive (Historical, Read-Only Context)

- Old handoff/status docs in `docs/archive/`
- Prior code generations in `legacy/`

Archive content is intentionally kept for context and should not be treated as authoritative for current implementation.

## Working Rule

When adding or editing docs:

1. Put active scanner behavior in `docs/current/`.
2. Put roadmaps/proposals in `docs/future/`.
3. Move obsolete snapshots into `docs/archive/` with date-based names.
