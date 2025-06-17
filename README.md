## Driver for ST7789VW LCD on GD32VF103

Driver for SKU22632 LCD Breakout board on a 1.3" 240x240 IPS LCD (ST7789VW driver) using a GD32VF103 RISC-V microcontroller and SPI and DMA.

The user can freely configure which SPI, DMA peripheral and GPIO pins are used for communication with the display.

![image](https://github.com/user-attachments/assets/1bd5ee32-b214-4a39-9df6-e109f845d37e)


## User options
Change resolution on LCD by chaning the following constants in sku22632.h header file

![image](https://github.com/user-attachments/assets/2e9fd384-8f7a-4733-ad70-e98ed928888c)


## 📌 Features

- Supports 1-bit monochrome bitmaps (240x240)
- User-configurable SPI, DMA interface and GPIOs
- Clear and easy to use API
- Non-blocking DMA buffer for high speed applications
- Uses hardware SPI and optimized screen writing

## 🖥️ Hardware

| Component           | Description                            |
|---------------------|----------------------------------------|
| **MCU**             | GD32VF103 (RISC-V core)                |
| **LCD Module**      | 1.3" 240x240 IPS LCD (SKU: 22632)      |
| **Display Driver**  | ST7789VW                               |
| **Interface**       | SPI & DMA                              |
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

> 💡 **Note:** All signal pins (`RST`, `DC`, `CS`, `CLK`, `DIN`) are **selectable** when initializing lcd. Please make sure that chosen SPI, DMA pheripheral has compatible channels and SPI GPIO pins are correctly selected. An usable example is presented in the table below


### ✅ Recommended Configuration for `lcd_init(...)`

Use the example below to quickly get started with a known working setup:

```c
lcd_init(SPI0, DMA0, DMA_CH2, GPIOA, GPIO_PIN_5, GPIO_PIN_7, GPIO_PIN_1, GPIO_PIN_2, GPIO_PIN_3);
```
| Argument       | Value        | Description                           |
| -------------- | ------------ | ------------------------------------- |
| `_spi_perpih`  | `SPI0`       | SPI peripheral used for data transfer |
| `_dma_periph`  | `DMA0`       | DMA controller                        |
| `_channel`     | `DMA_CH2`    | DMA channel mapped to `SPI0_TX`       |
| `_gpio_perpih` | `GPIOA`      | GPIO port used for all signal pins    |
| `_clk`         | `GPIO_PIN_5` | SPI0 clock pin (SCK)                  |
| `_din`         | `GPIO_PIN_7` | SPI0 data out pin (MOSI)              |
| `_rst`         | `GPIO_PIN_1` | LCD Reset pin                         |
| `_cs`          | `GPIO_PIN_2` | LCD Chip Select pin                   |
| `_dc`          | `GPIO_PIN_3` | LCD Data/Command pin                  |





