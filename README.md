# UAC Control

## Introduction

This repository is a module for the [LoRa Wireless Control Drone](https://github.com/Lorraine0666/LoRa-Wireless-Control-Drone), designed to control the attitude and communication of the drone via LoRa technology.

## Structure

The code is built using `Platform IO`. All related header files are located in the `include` folder, and the source code files can be found in the `src` folder.

## Prerequisites

Before compiling and running the code, make sure you have the following installed:

- **VS Code**: The integrated development environment (IDE).
- **Platform IO Extension**: This is the plugin used for building and uploading the code to your STM32 device.
- **STM32CubeProgrammer**: A utility to flash your STM32 microcontroller.

## Compile & Run

1. **Install the required software**:
   - Download and install **VS Code** from [here](https://code.visualstudio.com/).
   - Install the **Platform IO** extension from the VS Code marketplace.
   - Download and install **STM32CubeProgrammer** from [STMicroelectronics](https://www.st.com/en/development-tools/stm32cubeprog.html).

2. **Set up the environment**:
   - Open VS Code and make sure the **Platform IO** extension is installed.
   - Clone or download this repository to your local machine.

3. **Build the code**:
   - Open the repository folder in VS Code.
   - Click the **tick icon** (checkmark) on the top right of the VS Code window to build the project.

4. **Connect the STM32 board**:
   - Connect your STM32 development board to your computer via USB or another supported connection.

5. **Flash the firmware**:
   - Download the corresponding `.elf` file (compiled output).
   - Use **STM32CubeProgrammer** to flash the firmware onto your STM32 board, or use the Platform IO extension directly to upload the code to the board.

## Troubleshooting

- If you encounter issues with uploading the code, ensure that the STM32 board is properly connected and the correct port is selected in Platform IO.
- If the build fails, check the `platformio.ini` file for the correct board configuration and dependencies.