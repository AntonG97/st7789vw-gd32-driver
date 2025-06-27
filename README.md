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
Sure! Here’s a full, clean English README section you can copy-paste directly into your README file:

---

# LCD Initialization

This module initializes an LCD via SPI using a configuration struct for easy parameter management.

## Struct: `lcd_param_init`

This struct contains all the parameters needed to initialize the LCD.

| Field          | Type               | Description                                    |
| -------------- | ------------------ | ---------------------------------------------- |
| `_spi_periph`  | `uint32_t`         | SPI peripheral, e.g. `SPI0`, `SPI1`, `SPI2`    |
| `_dma_periph`  | `uint32_t`         | DMA controller, e.g. `DMA0`, `DMA1`            |
| `_dma_channel` | `dma_channel_enum` | DMA channel, e.g. `DMA_CH2`                    |
| `_clk`         | `gpio_param_t`     | GPIO port and pin for SPI clock (SCK)          |
| `_din`         | `gpio_param_t`     | GPIO port and pin for SPI data out (MOSI)      |
| `_rst`         | `gpio_param_t`     | GPIO port and pin for LCD reset (active LOW)   |
| `_cs`          | `gpio_param_t`     | GPIO port and pin for chip select (active LOW) |
| `_dc`          | `gpio_param_t`     | GPIO port and pin for data/command select      |

## Example configuration

```c
lcd_param_init lcd_params = {
    ._spi_periph = SPI0,
    ._dma_periph = DMA0,
    ._dma_channel = DMA_CH2,
    ._clk = {GPIOA, GPIO_PIN_5},  // SPI0 SCK
    ._din = {GPIOA, GPIO_PIN_7},  // SPI0 MOSI
    ._rst = {GPIOA, GPIO_PIN_1},  // LCD reset pin
    ._cs = {GPIOA, GPIO_PIN_2},   // LCD chip select pin
    ._dc = {GPIOA, GPIO_PIN_3}    // LCD data/command pin
};
```

## Usage

Initialize the LCD by passing a pointer to this struct:

```c
lcd_init(&lcd_params);
```

---


> **NOTE:** Font for ascii char provided by [Rinky-Dink Electronics](http://www.rinkydinkelectronics.com/r_fonts.php#:~:text=All%20fonts%20on%20this%20page,any%20project%2C%20commercial%20or%20not)
