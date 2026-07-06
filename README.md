# stm-prawler

STM32L476 firmware generated from STM32CubeMX and built with CMake.

## Build

Download and install the STM32CubeCLT (command-line tools)
https://www.st.com/en/development-tools/stm32cubeclt.html

Configure and build:

```sh
cmake --preset Debug
cmake --build --preset Debug
```

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