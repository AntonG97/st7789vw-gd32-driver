## Driver for ST7789VW LCD on GD32VF103

Driver for SKU22632 LCD Breakout board on a 1.3" 240x240 IPS LCD (ST7789VW driver) using a GD32VF103 RISC-V microcontroller and SPI.

The user can freely configure which SPI peripheral and GPIO pins are used for communication with the display.

![image](https://github.com/user-attachments/assets/1bd5ee32-b214-4a39-9df6-e109f845d37e)


## 📌 Features

- Supports 1-bit monochrome bitmaps (240x240)
- User-configurable SPI interface and GPIOs
- Clear and easy to use API
- Non-blocking 
- Uses hardware SPI and optimized screen writing

## 🖥️ Hardware

| Component           | Description                            |
|---------------------|----------------------------------------|
| **MCU**             | GD32VF103 (RISC-V core)                |
| **LCD Module**      | 1.3" 240x240 IPS LCD (SKU: 22632)      |
| **Display Driver**  | ST7789VW                               |
| **Interface**       | SPI                                    |
| **Voltage**         | 3.3V or 5V compatible                  |

### 🔌 LCD Pinout & Connection

Below is a description of the LCD pinout for the SKU22632 1.3" 240x240 IPS display with the ST7789VW driver:

| LCD Pin | Name         | Function                                                  | Configurable? |
|---------|--------------|-----------------------------------------------------------|----------------|
| `BL`    | Backlight    | LCD backlight. Connect to 3.3V or control via GPIO.       | ❌ No       |
| `RST`   | Reset        | **Active Low**. Resets the LCD controller.               | ✅ Yes         |
| `DC`    | Data/Command | **High = Data**, **Low = Command** mode selector.         | ✅ Yes         |
| `CS`    | Chip Select  | **Active Low**. Enables SPI communication with the LCD.   | ✅ Yes         |
| `CLK`   | SPI Clock    | SPI clock signal. Connect to MCU SPI SCK pin.             | ✅ Yes         |
| `DIN`   | SPI Data In  | SPI data input. Connect to MCU SPI MOSI pin.              | ✅ Yes         |
| `VCC`   | Power        | Supply voltage. Accepts 3.3V or 5V.                        | ❌ No          |
| `GND`   | Ground       | Ground connection.                                         | ❌ No          |

> 💡 **Note:** All signal pins (`RST`, `DC`, `CS`, `CLK`, `DIN`) are **selectable** when initializing lcd. Please make sure that choosen SPI pheripheral and SPI pins (CLK & DIN) are correctly selected


