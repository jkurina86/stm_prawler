# stm-prawler

STM32L476 firmware generated from STM32CubeMX and built with CMake.

## Linux build

Install the build tools and ARM embedded GCC toolchain:

```sh
sudo apt install cmake ninja-build gcc-arm-none-eabi
```

Configure and build:

```sh
cmake --preset Debug
cmake --build --preset Debug
```

The ELF is written to `build/linux/Debug/stm-prawler.elf`.

Release builds use the matching preset:

```sh
cmake --preset Release
cmake --build --preset Release
```

To flash with the provided `flash` target, install STM32CubeProgrammer and make
sure `STM32_Programmer_CLI` is on `PATH`, then run:

```sh
cmake --build --preset Debug --target flash
```

The legacy `build/Debug` directory may contain a stale CMake cache generated on
Windows. The Linux presets intentionally use `build/linux/...` so that caches
from different operating systems do not conflict.
