# stm-prawler

STM32L476 firmware generated from STM32CubeMX and built with CMake.

This branch provides a UART shell on USART1 and LI-COR instrument commands on
USART2 (`huart2`, 9600 baud, 8N1). SD and RTC commands remain available; recorder,
legacy sensor and WiFi services are excluded. See [LI-COR shell commands](Docs/LI8x0.md).

To start communicating with a powered instrument, run `licor-discover`,
`licor-configure`, then `licor`.

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