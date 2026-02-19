#pragma once 

extern "C"{
#include "gd32vf103_gpio.h"
}

struct GpioMode
{
    static constexpr uint32_t AnalogIn     = GPIO_MODE_AIN;
    static constexpr uint32_t AnalogFloat  = GPIO_MODE_IN_FLOATING;
    static constexpr uint32_t InputPU      = GPIO_MODE_IPU;
    static constexpr uint32_t InputPD      = GPIO_MODE_IPD;
    static constexpr uint32_t OutOD        = GPIO_MODE_OUT_OD;
    static constexpr uint32_t OutPP        = GPIO_MODE_OUT_PP;
    static constexpr uint32_t AltFuncOD    = GPIO_MODE_AF_OD;
    static constexpr uint32_t AltFuncPP    = GPIO_MODE_AF_PP;
};

struct GpioSpeed
{
    static constexpr uint32_t MHz2  = GPIO_OSPEED_2MHZ;
    static constexpr uint32_t MHz10 = GPIO_OSPEED_10MHZ;
    static constexpr uint32_t MHz50 = GPIO_OSPEED_50MHZ;
};

template<typename Pin>
class Gpio
{
	public:
		static inline void conf(uint32_t mode, uint32_t speed)
		{
			activateGpioRCU(Pin::port);
			gpio_init(Pin::periph,mode,speed,Pin::pin);
		}

		static void inline set(void) {

		}


		static void inline confINPT(void)
		{

		}

		static void inline confAF(void)
		{

		}


	private:

		static inline void activateGpioRCU(uint32_t periph) 
		{
			switch(periph)
			{
				case GPIOA: rcu_periph_clock_enable(RCU_GPIOA); break;
				case GPIOB: rcu_periph_clock_enable(RCU_GPIOB); break;
				case GPIOC: rcu_periph_clock_enable(RCU_GPIOC); break;
				case GPIOD: rcu_periph_clock_enable(RCU_GPIOD); break;
				case GPIOE: rcu_periph_clock_enable(RCU_GPIOE); break;
			}
		}
};
