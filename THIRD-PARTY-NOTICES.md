# Third-party notices

This repository is licensed under the MIT License (see `LICENSE`). The following components are third-party works, bundled or depended upon, and remain under their own licenses.

## Bundled in this repository

### Three.js

- Path: `tools/imu_cube_viewer/web/vendor/three.module.js`
- Copyright 2010-2024 Three.js Authors
- License: MIT (`SPDX-License-Identifier: MIT`, declared in the file header)
- Used by the browser-based orientation viewer for 3D rendering. Vendored rather than fetched at runtime so the viewer works offline.

Three.js is MIT-licensed, same as this repository; its own license header is retained unmodified.

## Runtime dependencies (not bundled — installed via pip)

Installed from PyPI at their own licenses; none are redistributed here.

| Package | License |
|---|---|
| numpy | BSD-3-Clause |
| scipy | BSD-3-Clause |
| pandas | BSD-3-Clause |
| scikit-learn | BSD-3-Clause |
| matplotlib | PSF-based (matplotlib license) |
| joblib | BSD-3-Clause |
| threadpoolctl | BSD-3-Clause |
| pyserial | BSD-3-Clause |
| vqf | MIT |

## Firmware toolchain (not bundled)

The firmware builds against the STM32duino core (`STMicroelectronics:stm32`), which carries its own licenses (predominantly BSD-3-Clause for STM32 HAL/LL drivers and CMSIS). The core is installed through the Arduino board manager and is not redistributed in this repository. Firmware sources here call into it; no ST-provided source files are copied in.
