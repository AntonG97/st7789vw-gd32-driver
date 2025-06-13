/* 
Replace this file with your code. Put your source files in this directory and any libraries in the lib folder. 
If your main program should be assembly-language replace this file with main.S instead.

Libraries (other than vendor SDK and gcc libraries) must have .h-files in /lib/[library name]/include/ and .c-files in /lib/[library name]/src/ to be included automatically.
*/

#include "gd32vf103.h"
#include "../include/sku22632.h"
#include "../include/lcd_font.h"

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

/**
 * Variable holds the current background color set during previous call of lcd_clear()
 */
static color curr_backgr;

int main(void){

	lcd_init(SPI0, GPIOA, GPIO_PIN_5, GPIO_PIN_7, GPIO_PIN_1, GPIO_PIN_2, GPIO_PIN_3);

	/*
	lcd_drawLine(0, 240, 120, 120, BLACK);

	lcd_ShowCh(20,20,'2', RED, BIG);
	lcd_ShowCh(40,20,'2', RED, SMALL);

	lcd_showStr(8,100, "Hello World!!!!!!!!!!",RED, BIG);

	lcd_showNum(20, 200, 0, GREEN, SMALL);
	
	lcd_showNum_float(50,200, 222.99, 2, RED, BIG);
	*/
	lcd_showPicture(kub_map_v4);

	while(1){
		lcd_queue_flush();

	
	}
	return 0;
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

	lcd_clear(WHITE);

	lcd_wr_cmd(DISPON);
	//Write data
	lcd_queue_flush_blocking();
}

/**
 * @brief Clears LCD screen
 * @param[in]: color: Specifies the color of the screen to be cleared to
 */
void lcd_clear(color color){
	setWindow(0, LCD_X_MAX, 0, LCD_Y_MAX);
	for(int i = 0; i < LCD_X_MAX * LCD_Y_MAX; i++){
	 	lcd_wr_data(((uint16_t)color >> 8) & 0xFF);  	// High byte
		lcd_wr_data((uint16_t)color & 0xFF);  			// Low byte
	}
	curr_backgr = color;
}

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
	uint8_t X_OFFSET = 0;
	uint8_t Y_OFFSET = 0;
	if( LCD_X_MAX == 240 && LCD_Y_MAX == 240 ){
		X_OFFSET = 80;
		Y_OFFSET = 0;
	}

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
	(xe = (xe > 240 ? LCD_X_MAX : xe));					//Make sure end coordinates is within range
	(ye = (ye > 240 ? LCD_Y_MAX : ye));
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

/**
 * @brief Writes a character on LCD
 * @param[in]: xs: Center x coordinate
 * @param[in]: ys: Center y coordinate
 * @param[in]: ch: Character that is to be written
 * @param[in]: _color: Color of Character
 * @param[in]: size: Size of Character
 */
void lcd_ShowCh(const uint16_t xs, const uint16_t ys, const uint8_t ch, const color _color, const font_size size){
	//Select font
	uint8_t var_y;
	uint8_t *font_type;
	if( size == SMALL){};
	if( size == NORMAL) {var_y = STD_FONT_Y_SIZE; font_type = arial_normal; }
	if( size == BIG) {var_y = BIG_FONT_Y_SIZE; font_type = arial_big;}
	int offset = (var_y << 1) * (ch - ' ');

	//Set window size
	uint8_t x = xs - (STD_FONT_X_SIZE >> 1);				//Calc leftmost corner..-
	uint8_t y = ys - (var_y >> 1);							
	setWindow(x, x + STD_FONT_X_SIZE - 1, y, y + var_y - 1);//... and set window

	//Write char
	for(uint8_t i = 0; i < (var_y << 1); i++){
		for(int8_t j = 7; j >= 0; j--){
			//Check each bit of each element. If 0b1 => Write new col, else keep current background color
			color set_color = ( (((font_type[i + offset] >> j) & 1U ) == 1 ) ? _color : curr_backgr); 
			lcd_wr_data((set_color >> 8) & 0xFF);  			// High byte
			lcd_wr_data(set_color & 0xFF);  				// Low byte
		}
	}
}

/**
 * @brief Writes a string on LCD
 * @param[in]: xs: Center x coordinate
 * @param[in]: ys: Center y coordinate
 * @param[in]: str: String that is to be written
 * @param[in]: _color: Color of Characters
 * @param[in]: size: Size of Characters
 */
void lcd_showStr(const uint16_t xs, const uint16_t ys, const uint8_t *str, const color _color, const font_size size){
	//Select font
	uint8_t var_y;
	uint8_t *font_type;
	if( size == SMALL){};
	if( size == NORMAL) {var_y = STD_FONT_Y_SIZE; font_type = arial_normal;}
	if( size == BIG) {var_y = BIG_FONT_Y_SIZE; font_type = arial_big;}

	uint8_t ind = 0;
	//Loop until NULL
	while( str[ind] != '\0' && ind < STD_FONT_X_SIZE ){		//While char not NULL and ind less than 16 (Max amount of chars on LCD row)
		int offset = (var_y << 1) * (str[ind] - ' ');

		//Set window size
		uint8_t x = xs - (STD_FONT_X_SIZE >> 1) + (STD_FONT_X_SIZE * ind);				//Calc leftmost corner...
		uint8_t y = ys - (var_y >> 1);							
		setWindow(x, x + STD_FONT_X_SIZE - 1, y, y + var_y - 1);						//... and set window

		//Write char
		for(uint8_t i = 0; i < (var_y << 1); i++){
			for(int8_t j = 7; j >= 0; j--){
				//Check each bit of each element. If 0b1 => Write new col, else keep current background color
				color set_color = ( (((font_type[i + offset] >> j) & 1U ) == 1 ) ? _color : curr_backgr); 
				lcd_wr_data((set_color >> 8) & 0xFF);  			// High byte
				lcd_wr_data(set_color & 0xFF);  				// Low byte
			}
		}
		ind++;		//Increment index var
	}
}

/**
 * @brief Writes a number on LCD
 * @param[in]: xs: Center x coordinate
 * @param[in]: ys: Center y coordinate
 * @param[in]: val: val to be written
 * @param[in]: _color: Color of val
 * @param[in]: size: Size of val
 */
void lcd_showNum(const uint16_t xs, const uint16_t ys, int val, const color _color, const font_size size){

	if (val == 0) {
    lcd_ShowCh(xs, ys,'0', _color, size);
    return;
	}

	//Select font
	uint8_t var_y;
	uint8_t *font_type;
	if( size == SMALL){};
	if( size == NORMAL) {var_y = STD_FONT_Y_SIZE; font_type = arial_normal;}
	if( size == BIG) {var_y = BIG_FONT_Y_SIZE; font_type = arial_big;}

	long BASE = 100000000;
	uint8_t ind = 0;
	while( (val / BASE) == 0 && BASE > 1) BASE /= 10;
	
	while( val > 0 && ind < 16 ){
		uint8_t curr_val = val / BASE;		//Get next MSB digit
		val -= (curr_val * BASE);			//Subtract
		BASE /= 10;							//Adjust base

		int offset = (var_y << 1) * (curr_val + '0' - ' ');								//Get correct character
	
		//Set window size
		uint8_t x = xs - (STD_FONT_X_SIZE >> 1) + (STD_FONT_X_SIZE * ind);				//Calc leftmost corner...
		uint8_t y = ys - (var_y >> 1);							
		setWindow(x, x + STD_FONT_X_SIZE - 1, y, y + var_y - 1);						//... and set window
	
		//Write char
		for(uint8_t i = 0; i < (var_y << 1); i++){
			for(int8_t j = 7; j >= 0; j--){
				//Check each bit of each element. If 0b1 => Write new col, else keep current background color
				color set_color = ( (((font_type[i + offset] >> j) & 1U ) == 1 ) ? _color : curr_backgr); 
				lcd_wr_data((set_color >> 8) & 0xFF);  			// High byte
				lcd_wr_data(set_color & 0xFF);  				// Low byte
			}
		}
		ind++;		//Increment index var
	}
}

/**
 * @brief Writes a float number on LCD
 * @param[in]: xs: Center x coordinate
 * @param[in]: ys: Center y coordinate
 * @param[in]: val: float val to be written
 * @param[in]: decim_pt: Amount of decimals
 * @param[in]: _color: Color of val
 * @param[in]: size: Size of val
 */
void lcd_showNum_float(const uint16_t xs, const uint16_t ys, const float val, uint8_t decim_pt, const color _color, const font_size size){

	//int decimal_points = 10 * 100000000;
	//Integral
	//.
	//Decimal
	int integral_val = val;										//Get integral part of val
	int decimal_val = (val - integral_val) * 100000000;			//Get decimal part of val
	
    if (val == 0) {
    lcd_ShowCh(xs, ys,'0', _color, size);
    return;
	}

	//Select font
	uint8_t var_y;
	uint8_t *font_type;
	if( size == SMALL){};
	if( size == NORMAL) {var_y = STD_FONT_Y_SIZE; font_type = arial_normal;}
	if( size == BIG) {var_y = BIG_FONT_Y_SIZE; font_type = arial_big;}

	long BASE = 100000000;
	uint8_t ind = 0;
	while( (integral_val / BASE) == 0 && BASE > 1) BASE /= 10;
	while( integral_val > 0 && ind < 16 ){
		uint8_t curr_val = integral_val / BASE;		//Get next MSB digit
		integral_val -= (curr_val * BASE);			//Subtract
		BASE /= 10;									//Adjust base

		int offset = (var_y << 1) * (curr_val + '0' - ' ');								//Get correct character
	
		//Set window size
		uint8_t x = xs - (STD_FONT_X_SIZE >> 1) + (STD_FONT_X_SIZE * ind);				//Calc leftmost corner...
		uint8_t y = ys - (var_y >> 1);							
		setWindow(x, x + STD_FONT_X_SIZE - 1, y, y + var_y - 1);						//... and set window
	
		//Write char
		for(uint8_t i = 0; i < (var_y << 1); i++){
			for(int8_t j = 7; j >= 0; j--){
				//Check each bit of each element. If 0b1 => Write new col, else keep current background color
				color set_color = ( (((font_type[i + offset] >> j) & 1U ) == 1 ) ? _color : curr_backgr); 
				lcd_wr_data((set_color >> 8) & 0xFF);  			// High byte
				lcd_wr_data(set_color & 0xFF);  				// Low byte
			}
		}
		ind++;		//Increment index var
	}

	//Decimal point
	lcd_ShowCh(xs + (STD_FONT_X_SIZE * ind ), ys, '.', _color, size);
	ind++;


	BASE = 100000000;
	while( (decimal_val / BASE) == 0 && BASE > 1 ) BASE /= 10;
	while( decimal_val > 0 && ind < 16 && decim_pt > 0){

		decim_pt--;

		uint8_t curr_val = decimal_val / BASE;		//Get next MSB digit
		decimal_val -= (curr_val * BASE);			//Subtract
		BASE /= 10;									//Adjust base

		int offset = (var_y << 1) * (curr_val + '0' - ' ');								//Get correct character
	
		//Set window size
		uint8_t x = xs - (STD_FONT_X_SIZE >> 1) + (STD_FONT_X_SIZE * ind);				//Calc leftmost corner...
		uint8_t y = ys - (var_y >> 1);							
		setWindow(x, x + STD_FONT_X_SIZE - 1, y, y + var_y - 1);						//... and set window
	
		//Write char
		for(uint8_t i = 0; i < (var_y << 1); i++){
			for(int8_t j = 7; j >= 0; j--){
				//Check each bit of each element. If 0b1 => Write new col, else keep current background color
				color set_color = ( (((font_type[i + offset] >> j) & 1U ) == 1 ) ? _color : curr_backgr); 
				lcd_wr_data((set_color >> 8) & 0xFF);  			// High byte
				lcd_wr_data(set_color & 0xFF);  				// Low byte
			}
		}
		ind++;		//Increment index var
	}
}

/**
 * @brief Draws picture on LCD.
 * @param[in]: bitMap: Pointer to bitmap of 240x240 / 8 bytes
 */
void lcd_showPicture(uint8_t *bitMap){
    setWindow(0, LCD_X_MAX - 1, 0, LCD_Y_MAX - 1);  

    for (uint16_t y = 0; y < LCD_Y_MAX; y++) {
        for (uint16_t x = 0; x < LCD_X_MAX; x++) {
            uint32_t pixelIndex = y * LCD_X_MAX + x;	//Pixel index
            uint32_t byteIndex = pixelIndex / 8;		//Byte index
            uint8_t bitIndex = 7 - (pixelIndex % 8);  	//MSB first

            uint8_t byte = bitMap[byteIndex];		
            uint8_t bit = (byte >> bitIndex) & 0x01;	

            color set_color = (bit == 1) ? BLACK : WHITE;

            lcd_wr_data((set_color >> 8) & 0xFF);  		// High byte
            lcd_wr_data(set_color & 0xFF);         		// Low byte
        }
    }
}

//LCD functions ends
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Auxillary
/**
 * @brief Delays (blocking)
 * @param[in] ms: Amount of ms to delay
 */
static void delay_ms(uint8_t ms){
	volatile long long base = 7200; 	//Base
	base = base*(long long)ms;			//Mult base with ms
	while(--base) __asm__ volatile("nop");
}