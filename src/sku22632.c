/* 
Replace this file with your code. Put your source files in this directory and any libraries in the lib folder. 
If your main program should be assembly-language replace this file with main.S instead.

Libraries (other than vendor SDK and gcc libraries) must have .h-files in /lib/[library name]/include/ and .c-files in /lib/[library name]/src/ to be included automatically.
*/

#include "gd32vf103.h"

/**
 * SPI base
 */
static uint32_t spi_periph;

/**
 * GPIO base
 */
static uint32_t gpio_perpih;

/**
 * Reset GPIO pin
 */
static uint32_t rst;
/**
 * Chip select GPIO pin. Active LOW. Configured as SPI_NSS (Select slave pin) and controlled by SPI hardware
 */
static uint32_t cs;

/**
 * Data/Command sleection GPIO pin
 */
static uint32_t dc;


int main(void){



	return 0;
}

/**
 * @brief initialises LCD. OBS
 * \param[in]: spi_periph: SPIx(x=0,1,2)
 * \param[in]: gpio_periph: GPIOx(x = A,B,C,D,E). OBS! Once chosen, base all GPIO pins on input param!
 * \param[in]: clk: SPI clock pin
 * \param[in]: din: SPI data out (MOSI)
 * \param[in]: rst: Reset => GPIO_PIN_x(x=0..15) 
 * \param[in]: cs: Chip select => GPIO_PIN_x(x=0..15)
 * \param[in]: dc: Data or cmd => GPIO_PIN_x(x=0..15)
 */
void lcd_init(uint32_t _spi_periph, uint32_t _gpio_perpih, uint32_t clk, uint32_t din, uint32_t _rst, uint32_t _cs, uint32_t _dc){

	spi_periph = _spi_periph;
	gpio_perpih = _gpio_perpih;
	rst = _rst;
	cs = _cs;
	dc = _dc;

	spi_parameter_struct spi_param;
	spi_param.device_mode = SPI_MASTER;
	spi_param.trans_mode = SPI_TRANSMODE_BDTRANSMIT;
	spi_param.frame_size = SPI_FRAMESIZE_8BIT;
	spi_param.nss = SPI_NSS_HARD;
	spi_param.endian = SPI_ENDIAN_LSB;
	spi_param.clock_polarity_phase= SPI_CK_PL_LOW_PH_1EDGE;
	spi_param.prescale = SPI_PSC_2;

	//Start RCU for SPI and GPIO
	switch(spi_periph){
		case SPI0: rcu_periph_clock_enable(RCU_SPI0); break;
		case SPI1: rcu_periph_clock_enable(RCU_SPI1); break;
		case SPI2: rcu_periph_clock_enable(RCU_SPI2); break;
	}

		switch(gpio_perpih){
		case GPIOA: rcu_periph_clock_enable(RCU_GPIOA); break;
		case GPIOB: rcu_periph_clock_enable(RCU_GPIOB); break;
		case GPIOC: rcu_periph_clock_enable(RCU_GPIOC); break;
		case GPIOD: rcu_periph_clock_enable(RCU_GPIOD); break;
		case GPIOE: rcu_periph_clock_enable(RCU_GPIOE); break;
	}


	//gpio_init();

	//spi_init()
}
