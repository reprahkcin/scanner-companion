# Tests

This directory contains test scripts for the Scanner Companion project.

## Test Files

- `test_xmp_feature.py` - Tests the XMP feature implementation
- `test_xmp_consolidation.py` - Tests XMP file consolidation workflow  
- `test_portable_session.py` - Tests portable session creation with helper files

## Running Tests

```bash
# Run individual tests
python tests/test_xmp_feature.py
python tests/test_xmp_consolidation.py
python tests/test_portable_session.py

# Or run from the tests directory
cd tests
python test_xmp_feature.py
```

## Test Requirements

Tests should be run from the project root directory and require the main `scanner_control.py` module to be available.