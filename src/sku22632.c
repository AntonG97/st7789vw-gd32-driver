/* 
Replace this file with your code. Put your source files in this directory and any libraries in the lib folder. 
If your main program should be assembly-language replace this file with main.S instead.

Libraries (other than vendor SDK and gcc libraries) must have .h-files in /lib/[library name]/include/ and .c-files in /lib/[library name]/src/ to be included automatically.
*/

#include "gd32vf103.h"
#include "../include/sku22632.h"

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

static void lcd_wr_data(const uint8_t data);
static void lcd_wr_cmd(const cmd data);
static void lcd_queue_flush_blocking(void);
static void setWindow(const uint16_t xs, const uint16_t xe, const uint16_t ys, const uint16_t ye);
static void fillWindow(const uint16_t xs, const uint16_t xe, const uint16_t ys, const uint16_t ye, color color);

//Auxillery functions
static void delay_ms(uint8_t ms);

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

int main(void){

	lcd_init(SPI0, GPIOA, GPIO_PIN_5, GPIO_PIN_7, GPIO_PIN_1, GPIO_PIN_2, GPIO_PIN_3);

	//DBG
	//setWindow(50, 150, 50, 150);
	//fillWindow(50, 150, 50, 150, BLUE);
	//setWindow(100, 150, 100, 150);
	//fillWindow(100, 150, 100, 150, RED);

	//lcd_drawLine(1,100,1,100, RED);

	/*
	lcd_drawRec(0,120,0,50,GREEN);
	//lcd_drawRec(0,280,230,280,RED);

	lcd_drawLine(120,120,100,140, BLACK);
	lcd_drawLine(100,140,120,120, BLACK);
	lcd_drawLine(0,240,0,240, BLACK);
	lcd_drawLine(0,240,240,0, BLACK);

	lcd_drawCircle(190,190, 30, BLACK);
	lcd_drawCircle_filled(100,100,20, RED);
	lcd_clear(RED);
	lcd_drawPixel_big(120,120,GREEN);
	*/

	lcd_drawLine(0, 240, 120, 120, BLACK);

	
	
	int max_y = LCD_Y_MAX - 90;
	int x = 0;
	int y = 120;
	int up = 1;

	while(1){
		lcd_queue_flush();
		lcd_clear(RED);
		lcd_clear(WHITE);
		lcd_clear(GREEN);

		/*
		lcd_drawPixel_big(x,y,RED);

		x = (x + 1) % LCD_X_MAX;
		//y = ( y + 1) % LCD_Y_MAX;
		
		if( y == max_y ) up = 0;
		if( y == max_y - 120 ) up = 1;
		//Climb y upper
		if( y >= 120 && y < max_y && up ){
			y++;
		}else{
			y--;
		}
		//Reached upper, go down
		//Reached down, go upp
		
		*/
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
	spi_param.prescale = SPI_PSC_4;	//TODO: Change later to higher!

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

	//Sleep out
	lcd_wr_cmd(SLPOUT);
	lcd_queue_flush_blocking();
	delay_ms(130);

	//Pixel format (0x3A + data(0x55) RGB16bit)
	lcd_wr_cmd(COLMOD);
	lcd_wr_data(0x55);

	lcd_wr_cmd(INVON);

	lcd_wr_cmd(MADCTL);
	lcd_wr_data(0xA0);

	lcd_drawRec_filled(0,240,0,240, WHITE); //LCD screen white
	
	lcd_wr_cmd(DISPON);
	lcd_queue_flush_blocking();
}

void lcd_clear(color color){
	setWindow(0, LCD_X_MAX, 0, LCD_Y_MAX);
	for(int i = 0; i < LCD_X_MAX * LCD_Y_MAX; i++){
	 	lcd_wr_data(((uint16_t)color >> 8) & 0xFF);  	// High byte
		lcd_wr_data((uint16_t)color & 0xFF);  			// Low byte
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//SPI functions begin
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
static void lcd_wr_cmd(const cmd data){
	spi_data _data = {0, (uint8_t)data};
	lcd_wr(_data);
}

/**
 * @brief Write LCD data
 * @param[in]: Data to be transmitted
 */
static void lcd_wr_data(const uint8_t data){
	spi_data _data = {1, data};
	lcd_wr(_data);
}
//SPI functions ends
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//LCD functions begin

/**
 * @brief Sets a new window
 * @param[in] xs: col start coordinate [0 <= xs <= xe]
 * @param[in] xe: col end coordinate [xs <= xe <= 239]
 * @param[in] ys: row start coordinate [0 <= ys <= ye]
 * @param[in] ye: row end coordinate [ys <= ye <= 319]
 */
static void setWindow(const uint16_t xs, const uint16_t xe, const uint16_t ys, const uint16_t ye){
	//Add offset to get correct window Size
	uint8_t X_OFFSET = 80;
	uint8_t Y_OFFSET = 0;
	lcd_wr_cmd(CASET);
	lcd_wr_data(((xs + X_OFFSET) >> 8 ) & 0xFF);
	lcd_wr_data((xs + X_OFFSET) & 0xFF);
	lcd_wr_data(((xe + X_OFFSET) >> 8) & 0xFF);
	lcd_wr_data((xe + X_OFFSET) & 0xFF);

	lcd_wr_cmd(RASET);
	lcd_wr_data(((ys + Y_OFFSET) >> 8) & 0xFF);
	lcd_wr_data((ys + Y_OFFSET) & 0xFF);
	lcd_wr_data(((ye + Y_OFFSET) >> 8) & 0xFF);
	lcd_wr_data((ye + Y_OFFSET) & 0xFF);
	//RAMWR
	lcd_wr_cmd(RAMWR);
}

/**
 * @brief Sets a new window
 * @param[in] xs: col start coordinate [0 <= xs <= xe]
 * @param[in] xe: col end coordinate [xs <= xe <= 239]
 * @param[in] ys: row start coordinate [0 <= ys <= ye]
 * @param[in] ye: row end coordinate [ys <= ye <= 319]
 * @param[in] color: Color to be filled
 */
static void fillWindow(const uint16_t xs, uint16_t xe, const uint16_t ys, uint16_t ye, color color){
	lcd_wr_cmd(RAMWR);       							// RAMWR (start writing to RAM)
	(xe = (xe > 240 ? 240 : xe));						//Make sure end coordinates is within range
	(ye = (ye > 240 ? 240 : ye));
	uint16_t dx = xe - xs + 1, dy = ye - ys + 1;
	for (int i = 0; i < dx*dy; i++) {
		lcd_wr_data(((uint16_t)color >> 8) & 0xFF);  	// High byte
		lcd_wr_data((uint16_t)color & 0xFF);  			// Low byte
	}
}

/**
 * @brief Draw single point
 * @param[in] x: col start coordinate [0 <= xs <= 240]
 * @param[in] y: row start coordinate [0 <= ys <= 240]
 * @param[in] color: Color to be filled
 */
void lcd_drawPixel(const uint16_t x, const uint16_t y, color color){
	setWindow(x, x, y, y);
	lcd_wr_data(((uint16_t)color >> 8) & 0xFF);
	lcd_wr_data((uint16_t)color & 0xFF);
}

/**
 * @brief Draw single point big size
 * @param[in] x: col start coordinate [0 <= xs <= 240]
 * @param[in] y: row start coordinate [0 <= ys <= 240]
 * @param[in] color: Color to be filled
 */
void lcd_drawPixel_big(const uint16_t x, const uint16_t y, color color){
	lcd_drawCircle_filled(x,y,2,color);
}

/**
 * @brief Sets a new window
 * @param[in] xs: col start coordinate 	[0 <= xs <= xe]
 * @param[in] xe: col end coordinate	[xs <= xe <= 240]
 * @param[in] ys: row start coordinate 	[0 <= ys <= ye]
 * @param[in] ye: row end coordinate 	[ys <= ye <= 240]
 */
void lcd_drawLine(uint16_t xs, uint16_t xe, uint16_t ys, uint16_t ye, color color){
		// Specialfall: vertikal linje
	 if (xs == xe) {						//Vertical line
		 if (ys > ye) { uint16_t tmp = ys; ys = ye; ye = tmp; }
		 setWindow(xs, xs, ys, ye);
		 for (uint16_t y = ys; y <= ye; y++) {
			lcd_wr_data(color >> 8);
			lcd_wr_data(color & 0xFF);
		}
		return;
	}

	// Specialfall: horisontell linje
	if (ys == ye) {							//Horizontal line
		if (xs > xe) { uint16_t tmp = xs; xs = xe; xe = tmp; }
		setWindow(xs, xe, ys, ys);
		for (uint16_t x = xs; x <= xe; x++) {
			lcd_wr_data(color >> 8);
			lcd_wr_data(color & 0xFF);
		}
		return;
	}

	//Bresenham's linjealgoritm. Given by ChatGPT
	int16_t dx = (xe > xs ? xe - xs : xs - xe);	//Abs val
	int16_t dy = -(ye > ys ? ye - ys : ys - ye);//(-)Abs val
	int16_t sx = (xs < xe) ? 1 : -1;			//Direction of x
	int16_t sy = (ys < ye) ? 1 : -1;			//Direction of y
	int16_t err = dx + dy;						//Error term

	while (1) {
		lcd_drawPixel(xs, ys, color);			//Draw pixel
		if (xs == xe && ys == ye) break;		//If both coordinates start = end pos then break
		int16_t e2 = err << 1;					//err * 2
		if (e2 >= dy) {							
			err += dy;
			xs += sx;
		}
		if (e2 <= dx) {
			err += dx;
			ys += sy;
		}
	}
}

/**
 * @brief Draw rectangle
 * @param[in] xs: col start coordinate 	[0 <= xs <= xe]
 * @param[in] xe: col end coordinate	[xs <= xe <= 240]
 * @param[in] ys: row start coordinate 	[0 <= ys <= ye]
 * @param[in] ye: row end coordinate 	[ys <= ye <= 240]
 */
void lcd_drawRec(const uint16_t xs, const uint16_t xe, const uint16_t ys, const uint16_t ye, color color){
	lcd_drawLine(xs, xe, ys, ys, color);		//Upper horiz
	lcd_drawLine(xs, xe, ye, ye, color);		//Lower horiz
	lcd_drawLine(xs, xs, ys, ye, color);		//Lower vert
	lcd_drawLine(xe, xe, ys, ye, color);		//Upper vert
}

/**
 * @brief Draw rectangle
 * @param[in] xs: col start coordinate 	[0 <= xs <= xe]
 * @param[in] xe: col end coordinate	[xs <= xe <= 240]
 * @param[in] ys: row start coordinate 	[0 <= ys <= ye]
 * @param[in] ye: row end coordinate 	[ys <= ye <= 240]
 */
void lcd_drawRec_filled(const uint16_t xs, const uint16_t xe, const uint16_t ys, const uint16_t ye, color color){
	setWindow(xs,xe,ys,ye);
	fillWindow(xs,xe,ys,ye, color);
}

/**
 * @brief Draw circle
 * @param[in]: xs: center x coordinate 
 * @param[in]: ys: center y coordinate 
 * @param[in]: r: radie
 */
void lcd_drawCircle(uint16_t xs, uint16_t ys, uint16_t r, color color){

	//Algoritm to draw circle based on Midpoint circle. Given by ChatGPT
	int16_t x = 0;
    int16_t y = r;
    int16_t d = 1 - r; 

    while (x <= y) {
        //Draw symmetric points in all eight octants
        lcd_drawPixel(xs + x, ys + y, color);
        lcd_drawPixel(xs - x, ys + y, color);
        lcd_drawPixel(xs + x, ys - y, color);
        lcd_drawPixel(xs - x, ys - y, color);
        lcd_drawPixel(xs + y, ys + x, color);
        lcd_drawPixel(xs - y, ys + x, color);
        lcd_drawPixel(xs + y, ys - x, color);
        lcd_drawPixel(xs - y, ys - x, color);

        if (d < 0) {
            d += 2 * x + 3;
        } else {
            d += 2 * (x - y) + 5;
            y--;
        }
        x++;
    }
}

/**
 * @brief Draw filled circle
 * @param[in]: xs: center x coordinate 
 * @param[in]: ys: center y coordinate 
 * @param[in]: r: radie
 */
void lcd_drawCircle_filled(uint16_t xs, uint16_t ys, uint16_t r, color color){
    int16_t x = 0;
    int16_t y = r;
    int16_t d = 1 - r;

    while (y >= x) {
        // Ritar horisontella linjer mellan symmetriska punkter i alla oktanter

        lcd_drawLine(xs - x, xs + x, ys - y, ys - y, color); // Övre horisontella linje
        lcd_drawLine(xs - x, xs + x, ys + y, ys + y, color); // Nedre horisontella linje
        lcd_drawLine(xs - y, xs + y, ys - x, ys - x, color); // Övre horisontella linje (andra oktanter)
        lcd_drawLine(xs - y, xs + y, ys + x, ys + x, color); // Nedre horisontella linje (andra oktanter)

        x++;

        if (d < 0) {
            d += (x << 1) + 1;
        } else {
            y--;
            d += ((x - y) << 1)+ 1;
        }
    }
}



//LCD functions ends
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Extras 
/**
 * @brief Delays (blocking)
 * @param[in] ms: Amount of ms to delay
 */
static void delay_ms(uint8_t ms){
	volatile long long base = 7200; 	//Base
	base = base*(long long)ms;			//Mult base with ms
	while(--base) __asm__ volatile("nop");
}