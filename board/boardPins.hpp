#pragma once

#include <cstdint>
extern "C" {
#include "gd32vf103_gpio.h"
}
struct BoardPins 
{
	struct Lcd
	{
		struct RTS {static constexpr uint32_t periph = GPIOA;static constexpr uint32_t pin = GPIO_PIN_1;};
		struct CS  {static constexpr uint32_t periph = GPIOA;static constexpr uint32_t pin = GPIO_PIN_2;};
		struct DC  {static constexpr uint32_t periph = GPIOA;static constexpr uint32_t pin = GPIO_PIN_3;}; 
	};

	struct Spi0
	{
		struct CLK  {static constexpr uint32_t periph = GPIOA;static constexpr uint32_t pin = GPIO_PIN_5;};
		struct MOSI {static constexpr uint32_t periph = GPIOA;static constexpr uint32_t pin = GPIO_PIN_7;};
	};

};
