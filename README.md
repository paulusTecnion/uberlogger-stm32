# 🧠 Uberlogger – STM32 Development Instructions

| | |
|---|---|
| ![Uberlogger UL01](https://github.com/user-attachments/assets/0152f741-67ca-4d7e-8d23-f9b1f567e315) | ![cropped_to_2000x1500_477230259-9cfe64ff-2d1a-40d0-abcc-8224dfc5fa30](https://github.com/user-attachments/assets/6eb20049-7d64-4672-af0b-c1cea997191f)  |


Welcome to the repository for the [Uberlogger](https://www.uberlogger.com), a simple yet powerful ESP32-based & STM32 WiFi enabled logging device. The logger can log 8 analog channels and 6 digital channels at maximum 250 Hz at 16 or 12 bits to CSV or raw data file on an SD card and is fully stand-alone. The analog channels can be set as either NTC inputs or normal analog inputs with input ranges varying from -10V to +10V DC or from -60V to + 60V DC. There is a web UI interface to configure the device and see live data and an [API](https://docs.uberlogger.com/api) to retrieve data using your own scripts or devices.

I decided to make this open source, since I have little time to make more features alone by myself, and next to that I notice there are hobbyists and research people who might be interested in using or adjusting the code to their needs. Since I value innovation and education, it made sense to make this open source. The source is what it is, with all the things that are nice and not so nice :-) I'm not at full-time programmer, so I was also learning along the way. What makes it a challenge to debug is that it needs to be stable over long periods over time and it uses 2 chips, from which the documentation was not always completely right. I don't have time to give support for new features you might want to add, but you are free to adjust or contribute and do whatever you want with it.

This repository contains the STM32G030 firmware that handles the analog-to-digital conversion, data processing, and communication with the ESP32 chip. The ESP32 firmware repository can be found [here](https://github.com/paulusTecnion/uberlogger-esp32).

## 💖 Support our cause

When you buy the Uberlogger, you help the children in Guatemala. 50% of the profits go to EDELAC, a school for the poorest children in the city Xela of Guatemala. Ordering can be done at our partner [Eleshop](https://eleshop.eu/uberloggerul01.html) for best and fast international shipments.

## 📄 Licensing and Legal

### ✅ Software License

The code in this repository is licensed under the [MIT License](LICENSE). You are free to use, modify, and redistribute it — even commercially — as long as proper attribution is provided.

### 🚫 Hardware Notice

The **hardware** design, including schematics, PCB layouts, mechanical components, and industrial design of the **Uberlogger**, is **proprietary**. It is **not covered** under the MIT License and may **not** be reproduced, sold, or modified for commercial use without **explicit written permission** from the author.

## 🚀 Prerequisites

1. **STM32CubeIDE**: Download and install the latest version from [STMicroelectronics](https://www.st.com/en/development-tools/stm32cubeide.html)
2. **STM32CubeMX**: Usually comes with STM32CubeIDE, but can be downloaded separately
3. **Git**: For cloning the repository

## 📥 Getting Started

### 1. Clone the Repository

```bash
git clone https://github.com/paulusTecnion/uberlogger-stm32.git
cd uberlogger-stm32
```

### 2. Open in STM32CubeIDE

1. Launch **STM32CubeIDE**
2. Go to **File** → **Import**
3. Select **General** → **Existing Projects into Workspace** (or use the Git extension to import this repo!)
4. Click **Next**
5. Browse to the cloned repository folder
6. Ensure the project is checked and click **Finish**

### 3. Project Structure

The project contains the following key components:

- **Core/Inc/**: Header files for all modules
- **Core/Src/**: Source files for all modules
- **Drivers/**: STM32 HAL drivers and CMSIS files
- **STM32G030C6TX_FLASH.ld**: Linker script
- **stm32g030c6.ioc**: STM32CubeMX configuration file

### 4. Build the Project

1. Right-click on the project in the Project Explorer
2. Select **Build Project** or press **Ctrl+B**

## ⚠️ Known Build Issue

You may encounter this error during the build process:

```bash
mv stm32g030c6.bin ../ota_support.bin
make[1]: *** [makefile:93: post-build] Error -1
make: *** [makefile:61: all] Error 2
"make -j20 all" terminated with exit code 2. Build might be incomplete.
```

**This error is harmless and can be ignored on Windows.** It occurs during the post-build step when trying to create a release file (`ota_support.bin`) for over-the-air updates. The main firmware compilation is successful, and this error only affects the release file creation.

## 🔧 Configuration

### STM32CubeMX Configuration

The project includes a pre-configured `.ioc` file. To modify the configuration:

1. Double-click on `stm32g030c6.ioc` in the Project Explorer
2. STM32CubeMX will open with the current configuration
3. Make your changes
4. Save and generate code when prompted

### Key Configuration Details

- **MCU**: STM32G030C6TX
- **Clock**: 16 MHz internal oscillator
- **ADC**: 8 channels with DMA
- **SPI**: Communication with ESP32
- **Timers**: ADC sampling and SPI communication timing
- **GPIO**: Digital inputs and status LEDs

## 📡 Communication with ESP32

The STM32 communicates with the ESP32 via SPI, handling:

- **Configuration commands**: ADC settings, sample rates, trigger modes
- **Data transmission**: ADC readings and digital input states
- **Status updates**: Device state and error reporting

## 🐞 Debugging

### Debug Output

The firmware includes debug capabilities through:

- **GPIO pins**: Status for different states
- **SPI communication**: Debug messages sent to ESP32
- **Internal timers**: Performance monitoring
- **ST-Link**: you can debug the STM32 using the ST-link! Read further below

### Common Issues

1. **Build errors**: Ensure all STM32 HAL drivers are properly included
2. **Communication issues**: Check SPI configuration and timing
3. **ADC accuracy**: Verify calibration and reference voltage settings

### Connecting the ST-Link for programming
Below you see the connector shown on the UL01B with the pinout the connector to you ST-link. The same holds for the UL01 version, but you'll have to remove the top cover first to reach the connector.

<img width="970" height="681" alt="image" src="https://github.com/user-attachments/assets/de1addae-8e7b-433e-af4d-598e8b9505dc" />

## 🔄 Development Workflow

1. **Modify code** in STM32CubeIDE
2. **Build** the project (Ctrl+B)
3. **Flash** to device using ST-Link or other programmer
4. **Test** functionality
5. **Debug** if needed using breakpoints and watch variables

## 📚 Additional Resources

- [STM32G030 Reference Manual](https://www.st.com/resource/en/reference_manual/dm00371828-stm32g0x0-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)
- [STM32 HAL Documentation](https://www.st.com/en/embedded-software/stm32cube-mcu-mpu-packages.html)
- [Uberlogger ESP32 Repository](https://github.com/paulusTecnion/uberlogger-esp32)
- [Uberlogger Documentation](https://docs.uberlogger.com)


## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

---

Happy Hacking! 🛠️

_Remember: The hardware design remains proprietary, but the software is open source for your learning and development needs._
