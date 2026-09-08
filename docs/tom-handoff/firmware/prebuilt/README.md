# Optional prebuilt firmware

The beginner path is in the package BUILD guide. These binaries are supplied for a lab member already familiar with STM32CubeProgrammer; they do not replace the wiring and acceptance checks.

- Target: NUCLEO-U385RG-Q only; STM32 core 2.12.0; ST-Link SWD upload configuration; USART1 through ST-Link virtual COM port; native USB disabled.
- `connectivity.bin`: diagnostic sketch, 115200 baud. TA0 pins D2, D4, D6, D8, D9 for IMU0 through IMU4.
- `streaming.bin`: raw/SFLP acquisition, 921600 baud, same TA0 mapping.
- Compiled on macOS on 8 September 2026. Compilation passed; physical flashing and garment operation have not been performed for this handoff revision.
- Raw binary load address for this build is 0x08000000. Verify the exact board and binary before any manual programming. Follow BUILD.md for the standard source-based route.

Checksums and source hashes are in `build-manifest.json`. Firmware sources are the sibling sketch directories. Do not substitute firmware from the earlier ZIP while using this package's wiring table.
