/* 
Replace this file with your code. Put your source files in this directory and any libraries in the lib folder. 
If your main program should be assembly-language replace this file with main.S instead.

Libraries (other than vendor SDK and gcc libraries) must have .h-files in /lib/[library name]/include/ and .c-files in /lib/[library name]/src/ to be included automatically.
*/

#include "gd32vf103.h"
#include "../include/sku22632.h"

static void lcd_wr_data(const uint8_t data);
static void lcd_wr_cmd(const uint8_t data);
static void delay_ms(uint8_t ms);
static void lcd_queue_flush_blocking(void);

/**
 * SPI base
 */
static uint32_t spi_perpih;

/**
 * GPIO base
 */
static uint32_t gpio_perpih;

/**
 * Reset GPIO pin. Active LOW.
 */
static uint32_t rst;

/**
 * Chip select GPIO pin. Active LOW. 
 */
static uint32_t cs;

/**
 * Data/Command selection GPIO pin. 
 * LOW => Cmd
 * HIGH => Data
 */
static uint32_t dc;

typedef enum{
	SWRESET = 0x01,	//SW reset
	SLPIN = 0x10,	//LCD power saving mode
	SLPOUT = 0x11, 	//LCD turn off power saving mode
	NORON = 0x13,	//Normal display mode on
	INVOFF = 0x20, 	//Display inversion off
	INVON = 0x21,	//Display inversion on
	GAMSET = 0x26,	//Gamma set
	DISPOFF = 0x28,	//Display OFF
	DISPON = 0x29, 	//Display ON
	CASET = 0x2A,	//Column adress set + 4 bytes of data
	RASET = 0x2B,	//Row adress set + 4 bytes of data
	RAMWR = 0x2C, 	//Memory write. Transfer data from MCU to frame mem. When cmd accepted, col and page set to start pos

	MADCTL = 0x36,	//Memory data access control
	COLMOD = 0x3A,	//Interface pixel format
	WRDISBV = 0x51,	//Write display brightness + 1 byte (0x00 => Lowest 0xFF => Highest)


}cmd;


int main(void){

	
	//DBG
	rcu_periph_clock_enable(RCU_GPIOB);
	gpio_init(GPIOB, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_9);
	gpio_bit_reset(GPIOB, GPIO_PIN_9);
	lcd_init(SPI0, GPIOA, GPIO_PIN_5, GPIO_PIN_7, GPIO_PIN_1, GPIO_PIN_2, GPIO_PIN_3);
	gpio_bit_set(GPIOB, GPIO_PIN_9);
	while(1){

		//gpio_bit_set(GPIOB, GPIO_PIN_9);
		//delay_ms(1);
		//gpio_bit_reset(GPIOB, GPIO_PIN_9);
		//delay_ms(1);
	}


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
void lcd_init(uint32_t _spi_perpih, uint32_t _gpio_perpih, uint32_t _clk, uint32_t _din, uint32_t _rst, uint32_t _cs, uint32_t _dc){

	//DBG
	//rcu_periph_clock_enable(RCU_GPIOB);
	//gpio_init(GPIOB, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_9);
	//gpio_bit_set(GPIOB, GPIO_PIN_9);


	spi_perpih = _spi_perpih;
	gpio_perpih = _gpio_perpih;
	rst = _rst;
	cs = _cs;
	dc = _dc;

	/**
	 * Init RCU for SPI and GPIO
	 */
	switch(gpio_perpih){
		case GPIOA: rcu_periph_clock_enable(RCU_GPIOA); break;
		case GPIOB: rcu_periph_clock_enable(RCU_GPIOB); break;
		case GPIOC: rcu_periph_clock_enable(RCU_GPIOC); break;
		case GPIOD: rcu_periph_clock_enable(RCU_GPIOD); break;
		case GPIOE: rcu_periph_clock_enable(RCU_GPIOE); break;
	}
	switch(spi_perpih){
		case SPI0: rcu_periph_clock_enable(RCU_SPI0); break;
		case SPI1: rcu_periph_clock_enable(RCU_SPI1); break;
		case SPI2: rcu_periph_clock_enable(RCU_SPI2); break;
	}
	
	//rcu_periph_clock_enable(RCU_AF); // Lägg till detta innan du initierar GPIO!
	//rcu_periph_clock_enable(RCU_GPIOA);

	/**
	 * Init GPIO
	 */
	//clk
	gpio_init(gpio_perpih, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, _clk);
	//din (MOSI)
	gpio_init(gpio_perpih, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, _din);
	//rst
	gpio_init(gpio_perpih, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, rst);
	gpio_bit_set(gpio_perpih, rst);
	//cs
	gpio_init(gpio_perpih, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, cs);
	gpio_bit_set(gpio_perpih, cs);
	//dc
	gpio_init(gpio_perpih, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, dc);
	//gpio_bit_set(gpio_perpih, dc);

	
	/**
	 * Init SPI As Master, 8 bit frame size, little endian and f = 27MHz
	 */
	spi_parameter_struct spi_param;

	spi_struct_para_init(&spi_param);
	spi_param.device_mode = SPI_MASTER;
	spi_param.trans_mode = SPI_TRANSMODE_BDTRANSMIT;
	spi_param.frame_size = SPI_FRAMESIZE_8BIT;
	spi_param.nss = SPI_NSS_SOFT;
	spi_param.endian = SPI_ENDIAN_MSB;
	spi_param.clock_polarity_phase= SPI_CK_PL_LOW_PH_1EDGE;
	spi_param.prescale = SPI_PSC_32;	//TODO: Change later to higher!

	//Init spi
	spi_init(spi_perpih, &spi_param);
	
	/**
	 * Enable SPI
	 */
	spi_enable(spi_perpih);

	
	//HW reset (Reset low for 10-20ms)
	gpio_bit_reset(gpio_perpih, rst);
	delay_ms(40);
	gpio_bit_set(gpio_perpih, rst);

	//SW reset (CMD: 0x01, wait 120ms)
	lcd_wr_cmd(SWRESET);
	lcd_queue_flush_blocking();
	delay_ms(130);

	//Sleep out?
	lcd_wr_cmd(SLPOUT);
	lcd_queue_flush_blocking();
	delay_ms(130);

	//Pixel format (0x3A + data(0x55) RGB16bit)
	lcd_wr_cmd(COLMOD);
	lcd_wr_data(0x55);
	//lcd_queue_flush_blocking();

	lcd_wr_cmd(INVON);

	//MADCTL (cmd 0x36 + data)
	lcd_wr_cmd(MADCTL);
	lcd_wr_data(0x00);
	//lcd_queue_flush_blocking();

	//Set col(cmd 0x2A + 4 bytes) & adr (cmd 0x2B + 4 bytes) range
	lcd_wr_cmd(CASET);
	lcd_wr_data(0x00);
	lcd_wr_data(0x00);
	lcd_wr_data(0x00);
	lcd_wr_data(0xEF);

	lcd_wr_cmd(RASET);
	lcd_wr_data(0x00);
	lcd_wr_data(0x00);
	lcd_wr_data(0x01);
	lcd_wr_data(0x3F);
	//RAMWR
	lcd_wr_cmd(RAMWR);

	//lcd_queue_flush_blocking();

	lcd_wr_cmd(DISPON);
	//lcd_queue_flush_blocking();
	//Dislpay on (cmd 0x29)	

	//delay_ms(100);


	lcd_wr_cmd(RAMWR);       // RAMWR (start writing to RAM)
	//lcd_queue_flush_blocking();
	for (int i = 0; i < 240*240; i++) {
		//0xF0 MSB
		//0x00 LSB
		lcd_wr_data(0x07);  // High byte
		lcd_wr_data(0xE0);  // Low byte
	}
	
	lcd_queue_flush_blocking();

	//lcd_wr_cmd(WRDISBV);
	//lcd_wr_data(0xF0);
	//lcd_queue_flush_blocking();
}

static inline void dc_set(void) {
    gpio_bit_set(gpio_perpih, dc);
}

static inline void dc_clr(void) {
    gpio_bit_reset(gpio_perpih, dc);
}

static inline void cs_set(void) {
    gpio_bit_set(gpio_perpih, cs);
}

static inline void cs_clr(void) {
    gpio_bit_reset(gpio_perpih, cs);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//SPI functions begin
#define BUFF_SIZE 512
/**
 * Struct for SPI data
 * @param type: Kind of data. 0 = cmd and 1 = data
 * @param data: Data to be sent
 */
typedef struct{
	//0 = cmd, 1 = data
	uint8_t type;
	uint8_t data;
}spi_data;

/*
 * Spi data buffer
 */
spi_data buff[BUFF_SIZE] = {0};
int tail = 0, head = 0;
//If buffer is full => full = 1
uint8_t full = 0;

/**
 * \brief Queue for LCD. Call first in superloop
 */
void lcd_queue_flush(void){
	//Prev_type indicate if needed to toggle DC 
	static uint8_t prev_type = 0;

	if( head != tail ){												//Buffer is NOT empty!
		//If same type and SPI buffer ready for data
		if( buff[tail].type == prev_type && spi_i2s_flag_get(spi_perpih, SPI_FLAG_TBE) == SET ){
			cs_clr();
			spi_i2s_data_transmit(spi_perpih, buff[tail].data); 	//Send data
			tail = (tail + 1) & (BUFF_SIZE - 1);					//Increment and wrap tail
		//Different types, then wait for SPI to finish tx b.f toggling DC
		}else if( spi_i2s_flag_get(spi_perpih, SPI_FLAG_TRANS) == RESET ){
			cs_clr();
			buff[tail].type ? dc_set() : dc_clr();					//Data or cmd
			prev_type = buff[tail].type;
			spi_i2s_data_transmit(spi_perpih, buff[tail].data); 	//Send data
			tail = (tail + 1) & (BUFF_SIZE - 1);					//Increment and wrap tail
		}
	
	}else{
		if(  spi_i2s_flag_get(spi_perpih, SPI_FLAG_TRANS) == RESET ) cs_set();
	}	
}

/**
 * @brief Function used by initialisation of LCD. Blocking
 */
static void lcd_queue_flush_blocking(void){
    static uint8_t prev_type = 0;

    while(head != tail){
		//If same type and SPI buffer ready for data
        if(buff[tail].type == prev_type && spi_i2s_flag_get(spi_perpih, SPI_FLAG_TBE) == SET){
            cs_clr();
            spi_i2s_data_transmit(spi_perpih, buff[tail].data);
            tail = (tail + 1) & (BUFF_SIZE - 1);
        }
		//Different types, then wait for SPI to finish tx b.f toggling DC
        else if(spi_i2s_flag_get(spi_perpih, SPI_FLAG_TRANS) == RESET){
            cs_clr();
            buff[tail].type ? dc_set() : dc_clr();
            prev_type = buff[tail].type;
            spi_i2s_data_transmit(spi_perpih, buff[tail].data);
            tail = (tail + 1) & (BUFF_SIZE - 1);
        }
        //Wait until spi ready
    }

    
    while(spi_i2s_flag_get(spi_perpih, SPI_FLAG_TRANS) != RESET);
    cs_set();
}



/**
 * @brief Writes data to SPI buffer to be sent
 * @param[in] data: Data to be written by SPI
 */
static void lcd_wr(const spi_data data){
	while(((head + 1) & (BUFF_SIZE - 1)) == tail) lcd_queue_flush(); //Flush buff if FULL
	buff[head] = data;								//Add data to buffer
	head = (head + 1) & (BUFF_SIZE - 1);			//Inrement and wrap head. (BUFF_SIZE - 1 = bitmask of 1's)
}

/**
 * @brief Write LCD command
 * @param[in]: Data to be transmitted
 */
static void lcd_wr_cmd(uint8_t data){
	spi_data _data = {0, data};
	lcd_wr(_data);
}

/**
 * @brief Write LCD data
 * @param[in]: Data to be transmitted
 */
static void lcd_wr_data(uint8_t data){
	spi_data _data = {1, data};
	lcd_wr(_data);
}
//SPI functions begin
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



/**
 * @brief Delays (blocking)
 * @param[in] ms: Amount of ms to delay
 */
static void delay_ms(uint8_t ms){
	volatile long long base = 7200; 	//Base
	base = base*(long long)ms;			//Mult base with ms
	while(--base) __asm__ volatile("nop");
}