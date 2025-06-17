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

typedef struct{
	uint32_t amount_trans;		//Amount of transfers
	uint8_t type;				//LOW (0) => HIGH (1) => Data			
	uint16_t payload;					
}dma_spi_data_t;

//DMA
static void lcd_dma_init(uint32_t _dma_periph, dma_channel_enum _channel, uint32_t _spi_perpih);
static void dma_lcd_wr(const dma_spi_data_t data);
static void dma_lcd_wr_cmd(const cmd payload);
static void dma_lcd_wr_data(const uint16_t payload, const uint32_t amount);
static void dma_buffer_flush_blocking(void);

//Auxillery functions
static void setWindow(const uint16_t xs, const uint16_t xe, const uint16_t ys, const uint16_t ye);
static void fillWindow(const uint16_t xs, const uint16_t xe, const uint16_t ys, const uint16_t ye, color color);
static void delay_ms(uint16_t ms);
static inline void dc_set(void);
static inline void dc_clr(void);
static inline void cs_set(void);
static inline void cs_clr(void);

/**
 * SPI base
 */
static uint32_t spi_perpih;
/**
 * DMA base
 */
static uint32_t dma_periph;
/**
 * DMA channel
 */
static dma_channel_enum dma_channel;

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

	//TODO: ShowNum_float is bugged when writing 10.xxx

	lcd_init(SPI0, DMA0, DMA_CH2, GPIOA, GPIO_PIN_5, GPIO_PIN_7, GPIO_PIN_1, GPIO_PIN_2, GPIO_PIN_3);

	rcu_periph_clock_enable(RCU_GPIOB);
	gpio_init(GPIOB, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_10);
	gpio_bit_reset(GPIOB, GPIO_PIN_10);

	setWindow(0,120,0,120);
	fillWindow(0,120,0,120, RED);

	lcd_drawPixel(150,150,RED);
	lcd_drawPixel_big(170,170,RED);

	lcd_drawLine(0,240,240,0,GREEN);

	lcd_drawRec(50,100,50,100,BLACK);
	lcd_drawRec_filled(50,100,50,100,GRAY);

	lcd_drawCircle(120,120,20, GRED);
	lcd_drawCircle_filled(120,120,18, MAGENTA);

	lcd_ShowCh(150,150,'A', RED);
	lcd_showStr(10,170,"FUCK", GREEN);

	lcd_showNum(10,10,99, BLACK);

	lcd_showNum_float(160,40,1000.7854,2, BLACK);

	//lcd_showPicture(kub_map_v4);
	




	
	while(1){
		dma_buffer_flush();



	}

	return 0;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//DMA functions begin 
#define DMA_BUFFER_SIZE 256
/**
 * DMA data buffer
 */
static dma_spi_data_t dma_buffer[DMA_BUFFER_SIZE];
/**
 * LCD data buffer. Fill with pixel value and let DMA transmit to SPI
 */
static uint16_t dma_data;
/**
 * Variables used in DMA buffer logic
 */
static int dma_head = 0, dma_tail = 0;
/**
 * @brief Place in beginning of superloop. Handles DMA queue
 */
void dma_buffer_flush(void){
	/**
	 * Variable is used to DMA ISR to correctly set GPIOx pins based on previous and current data to be transmitted
	 */
	static uint8_t prev_type = 0;

	if( dma_head != dma_tail ){						//Buffer is NOT empty!

		//If same type and SPI tx buffer ready for data!
		if( dma_buffer[dma_tail].type == prev_type && spi_i2s_flag_get(spi_perpih, SPI_FLAG_TBE) == SET ){
			cs_clr();
			dma_channel_disable(dma_periph, dma_channel);	//Start DMA transfer
			dma_data = dma_buffer[dma_tail].payload;
			dma_transfer_number_config(dma_periph, dma_channel, dma_buffer[dma_tail].amount_trans);

			dma_channel_enable(dma_periph, dma_channel);	//Start DMA transfer
			dma_tail = (dma_tail + 1) & (DMA_BUFFER_SIZE - 1);

		//Different types and SPI has finished transmitting
		}else if( dma_buffer[dma_tail].type != prev_type && spi_i2s_flag_get(spi_perpih, SPI_FLAG_TRANS) == RESET ){
			cs_clr();
			dma_channel_disable(dma_periph, dma_channel);	//Start DMA transfer
			dma_data = dma_buffer[dma_tail].payload;
			dma_buffer[dma_tail].type ? dc_set() : dc_clr();	//Data or cmd
			prev_type = dma_buffer[dma_tail].type;
			dma_transfer_number_config(dma_periph, dma_channel, dma_buffer[dma_tail].amount_trans);

			dma_channel_enable(dma_periph, dma_channel); 	//Start DMA transfer
			dma_tail = (dma_tail + 1) & (DMA_BUFFER_SIZE - 1);
		}

	}else{
		if( spi_i2s_flag_get(spi_perpih, SPI_FLAG_TRANS) == RESET ){
		cs_set();
		dma_channel_disable(dma_periph, dma_channel);
		}
	}
}

/**
 * @brief Function used by initialisation of LCD. Blocking
 */
static void dma_buffer_flush_blocking(void){
    static uint8_t prev_type = 0;
    while( dma_head != dma_tail ){						//Buffer is NOT empty!

		//If same type and SPI tx buffer ready for data!
		if( dma_buffer[dma_tail].type == prev_type && spi_i2s_flag_get(spi_perpih, SPI_FLAG_TBE) == SET ){
			cs_clr();
			dma_channel_disable(dma_periph, dma_channel);	//Start DMA transfer
			dma_data = dma_buffer[dma_tail].payload;
			dma_transfer_number_config(dma_periph, dma_channel, dma_buffer[dma_tail].amount_trans);

			dma_channel_enable(dma_periph, dma_channel);	//Start DMA transfer
			dma_tail = (dma_tail + 1) & (DMA_BUFFER_SIZE - 1);
		//Different types and SPI has finished transmitting
		}else if( dma_buffer[dma_tail].type != prev_type && spi_i2s_flag_get(spi_perpih, SPI_FLAG_TRANS) == RESET ){
			cs_clr();
			dma_channel_disable(dma_periph, dma_channel);	//Start DMA transfer
			dma_data = dma_buffer[dma_tail].payload;
			dma_buffer[dma_tail].type ? dc_set() : dc_clr();	//Data or cmd
			prev_type = dma_buffer[dma_tail].type;
			dma_transfer_number_config(dma_periph, dma_channel, dma_buffer[dma_tail].amount_trans);

			dma_channel_enable(dma_periph, dma_channel); 	//Start DMA transfer
			dma_tail = (dma_tail + 1) & (DMA_BUFFER_SIZE - 1);
		}
	}

    while(spi_i2s_flag_get(spi_perpih, SPI_FLAG_TRANS) != RESET);
    cs_set();
}

/**
 * @brief Initialises DMA pherip and channel used by LCD
 * @param[in]: _dma_periph: DMAx(x=0,1)
 * @param[in]: _channel: specify which DMA channel is initialized only one parameter can be selected => 
 * 				DMA0: DMA_CHx(x=0..6), DMA1: DMA_CHx(x=0..4)
 * @param[in]: _spi_periph: SPIx(x=0,1,2).
 */
static void lcd_dma_init(uint32_t _dma_periph, dma_channel_enum _channel, uint32_t _spi_perpih){
	
	rcu_periph_clock_enable(RCU_DMA0);

	dma_periph = _dma_periph;
	dma_channel = _channel;
	dma_channel_disable(dma_periph, dma_channel);
	/**
	 * Initilise dma parameters
	 */
	dma_parameter_struct dma_init_struct;
	dma_struct_para_init(&dma_init_struct);

	dma_init_struct.direction = DMA_MEMORY_TO_PERIPHERAL; 	
	dma_init_struct.memory_addr = (uint32_t)&dma_data; 
	dma_init_struct.memory_inc = DMA_MEMORY_INCREASE_DISABLE; 
	dma_init_struct.memory_width = DMA_MEMORY_WIDTH_16BIT; 			//Buffer 16 bits of data / transfer
	//dma_init_struct.number = (LCD_X_MAX * LCD_Y_MAX); 				//2 bytes per pixel. There are 240x240 pixels
	dma_init_struct.number = 0;
	dma_init_struct.periph_addr = (uint32_t)((_spi_perpih) + 0x0CU);
	dma_init_struct.periph_inc = DMA_PERIPH_INCREASE_DISABLE; 		
	dma_init_struct.periph_width = DMA_PERIPHERAL_WIDTH_16BIT; 		//SPIx 16 bit data
	dma_init_struct.priority = DMA_PRIORITY_HIGH;					//Priority
	
	dma_init(dma_periph, dma_channel, &dma_init_struct);
	
	//Interrup config
	//dma_interrupt_flag_clear(dma_periph, dma_channel, DMA_INT_FTF);
	//dma_interrupt_enable(dma_periph, dma_channel, DMA_INT_FTF); 
	//eclic_irq_enable(DMA0_Channel2_IRQn, 1, 1);
	//eclic_priority_group_set(ECLIC_PRIGROUP_LEVEL3_PRIO1); 
}

static void dma_lcd_wr(const dma_spi_data_t data){
	while(((dma_head + 1) & (DMA_BUFFER_SIZE - 1)) == dma_tail) dma_buffer_flush(); //Flush buff if FULL (OBS BLOCKING!)
	dma_buffer[dma_head] = data;								//Add data to buffer
	dma_head = (dma_head + 1) & (DMA_BUFFER_SIZE - 1);
}

/**
 * @brief Write LCD command
 * @param[in]: payload: Data to be transmitted
 */
static void dma_lcd_wr_cmd(const cmd payload){
	dma_spi_data_t _data = {1, 0, (0x00FF & (payload)) };
	dma_lcd_wr(_data);
}

/**
 * @brief Write LCD data
 * @param[in]: payload: Data to be transmitted
 * @param[in]: amount: How many bytes to send (each DMA tx is 2 bytes)
 */
static void dma_lcd_wr_data(const uint16_t payload, const uint32_t amount){
	dma_spi_data_t _data = {amount , 1, payload};
	dma_lcd_wr(_data);
}

//DMA functions end 
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//SPI functions begin

/**
 * @brief initialises LCD. OBS
 * @param[in]: spi_periph: SPIx(x=0,1,2)
 * @param[in]: _dma_periph: DMAx(x=0,1)
 * @param[in]: _channel: specify which DMA channel is initialized only one parameter can be selected => DMA0: DMA_CHx(x=0..6), DMA1: DMA_CHx(x=0..4)
 * @param[in]: gpio_periph: GPIOx(x = A,B,C,D,E). OBS! Once chosen, base all GPIO pins on input param!
 * @param[in]: clk: SPI clock pin
 * @param[in]: din: SPI data out (MOSI)
 * @param[in]: rst: Reset => GPIO_PIN_x(x=0..15) 
 * @param[in]: cs: Chip select => GPIO_PIN_x(x=0..15)
 * @param[in]: dc: Data or cmd => GPIO_PIN_x(x=0..15)
 */
void lcd_init(uint32_t _spi_perpih, uint32_t _dma_periph, dma_channel_enum _channel, uint32_t _gpio_perpih, uint32_t _clk, uint32_t _din, uint32_t _rst, uint32_t _cs, uint32_t _dc){
	spi_perpih = _spi_perpih;
	gpio_perpih = _gpio_perpih;
	rst = _rst;
	cs = _cs;
	dc = _dc;

	lcd_dma_init(_dma_periph, _channel, spi_perpih);

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
	spi_param.frame_size = SPI_FRAMESIZE_16BIT;
	spi_param.nss = SPI_NSS_SOFT;
	spi_param.endian = SPI_ENDIAN_MSB;
	spi_param.clock_polarity_phase= SPI_CK_PL_LOW_PH_1EDGE;
	spi_param.prescale = SPI_PSC_4;	//TODO: Change later to higher!

	//spi_i2s_deinit(spi_perpih);         // Rensar SPI-register
	//spi_disable(spi_perpih);            // Se till att den är helt avslagen


	//Init spi
	spi_init(spi_perpih, &spi_param);
	//Enable DMA data transfers
	spi_dma_enable(spi_perpih, SPI_DMA_TRANSMIT);
	//Enable global interrups
	//eclic_global_interrupt_enable(); 
	
	/**
	 * Enable SPI
	 */
	spi_enable(spi_perpih);

	//delay_ms(40+130+130); //Delay to make work

	
	//HW reset (Reset low for 10-20ms)
	gpio_bit_reset(gpio_perpih, rst);
	delay_ms(40);
	gpio_bit_set(gpio_perpih, rst);

	
	//SW reset (CMD: 0x01, wait 120ms)
	dma_lcd_wr_cmd(SWRESET);
	dma_buffer_flush_blocking();
	delay_ms(130);

	//Sleep out
	dma_lcd_wr_cmd(SLPOUT);
	dma_buffer_flush_blocking();
	delay_ms(130);

	//Pixel format (0x3A + data(0x55) RGB16bit)
	dma_lcd_wr_cmd(COLMOD);
	dma_lcd_wr_data(0x5500,1);

	dma_lcd_wr_cmd(INVON);

	dma_lcd_wr_cmd(MADCTL);
	dma_lcd_wr_data(0xA000,1);

	lcd_clear(WHITE);
	dma_buffer_flush_blocking();

	dma_lcd_wr_cmd(DISPON);

	dma_buffer_flush_blocking();
	
}

//SPI functions ends
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//LCD functions begin

/**
 * @brief Clears LCD screen
 * @param[in]: color: Specifies the color of the screen to be cleared to
 */
void lcd_clear(color color){
	setWindow(0, LCD_X_MAX - 1, 0, LCD_Y_MAX - 1);
	fillWindow(0, LCD_X_MAX - 1, 0, LCD_Y_MAX - 1, color);
	curr_backgr = color;
}

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

	dma_lcd_wr_cmd(CASET);
	dma_lcd_wr_data( xs + X_OFFSET, 1);
	dma_lcd_wr_data( xe + X_OFFSET, 1);

	dma_lcd_wr_cmd(RASET);
	dma_lcd_wr_data( ys + Y_OFFSET, 1);
	dma_lcd_wr_data( ye + Y_OFFSET, 1);
	//RAMWR
	dma_lcd_wr_cmd(RAMWR);
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
	dma_lcd_wr_cmd(RAMWR);       						// RAMWR (start writing to RAM)
	(xe = (xe > 240 ? LCD_X_MAX : xe));					//Make sure end coordinates is within range
	(ye = (ye > 240 ? LCD_Y_MAX : ye));
	uint16_t dx = xe - xs + 1, dy = ye - ys + 1;

	dma_lcd_wr_data(color, dx*dy);
}

/**
 * @brief Draw single point
 * @param[in] x: col start coordinate [0 <= xs <= 240]
 * @param[in] y: row start coordinate [0 <= ys <= 240]
 * @param[in] color: Color to be filled
 */
void lcd_drawPixel(const uint16_t x, const uint16_t y, color color){
	setWindow(x, x, y, y);
	dma_lcd_wr_data(color, 1);
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
			dma_lcd_wr_data(color,1);
		}
		return;
	}

	// Specialfall: horisontell linje
	if (ys == ye) {							//Horizontal line
		if (xs > xe) { uint16_t tmp = xs; xs = xe; xe = tmp; }
		setWindow(xs, xe, ys, ys);
		for (uint16_t x = xs; x <= xe; x++) {
			dma_lcd_wr_data(color,1);
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
	
	if( xs < r ) xs = r;
	if( ys < r) ys = r;


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

	if( xs < r ) xs = r;
	if( ys < r) ys = r;

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
void lcd_ShowCh(const uint16_t xs, const uint16_t ys, const uint8_t ch, const color _color){
	//Select font
	uint8_t var_y = STD_FONT_Y_SIZE;
	uint8_t *font_type = arial;
	//if( size == NORMAL) {var_y = STD_FONT_Y_SIZE; font_type = arial_normal; }
	//if( size == BIG) {var_y = BIG_FONT_Y_SIZE; font_type = arial_big;}
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
			dma_lcd_wr_data(set_color,1);
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
void lcd_showStr(const uint16_t xs, const uint16_t ys, const uint8_t *str, const color _color){
	//Select font
	uint8_t var_y = STD_FONT_Y_SIZE;
	uint8_t *font_type = arial;
	//if( size == SMALL){};
	//if( size == NORMAL) {var_y = STD_FONT_Y_SIZE; font_type = arial_normal;}
	//if( size == BIG) {var_y = BIG_FONT_Y_SIZE; font_type = arial_big;}

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
				//lcd_wr_data((set_color >> 8) & 0xFF);  			// High byte
				//lcd_wr_data(set_color & 0xFF);  				// Low byte
				dma_lcd_wr_data(set_color,1);

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
void lcd_showNum(const uint16_t xs, const uint16_t ys, int val, const color _color){

	if (val == 0) {
    lcd_ShowCh(xs, ys,'0', _color);
    return;
	}

	//Select font
	uint8_t var_y = STD_FONT_Y_SIZE;
	uint8_t *font_type = arial;
	//if( size == SMALL){};
	//if( size == NORMAL) {var_y = STD_FONT_Y_SIZE; font_type = arial_normal;}
	//if( size == BIG) {var_y = BIG_FONT_Y_SIZE; font_type = arial_big;}

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
				//lcd_wr_data((set_color >> 8) & 0xFF);  			// High byte
				//lcd_wr_data(set_color & 0xFF);  				// Low byte
				dma_lcd_wr_data(set_color,1);
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
void lcd_showNum_float(const uint16_t xs, const uint16_t ys, const float val, uint8_t decim_pt, const color _color){

	//int decimal_points = 10 * 100000000;
	//Integral
	//.
	//Decimal
	int integral_val = val;										//Get integral part of val
	int decimal_val = (val - integral_val) * 100000000;			//Get decimal part of val
	
    if (val == 0) {
    lcd_ShowCh(xs, ys,'0', _color);
    return;
	}

	//Select font
	uint8_t var_y = STD_FONT_Y_SIZE;
	uint8_t *font_type = arial;
	//if( size == SMALL){};
	//if( size == NORMAL) {var_y = STD_FONT_Y_SIZE; font_type = arial_normal;}
	//if( size == BIG) {var_y = BIG_FONT_Y_SIZE; font_type = arial_big;}

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
				dma_lcd_wr_data(set_color, 1);
			}
		}
		ind++;		//Increment index var
	}

	//Decimal point
	lcd_ShowCh(xs + (STD_FONT_X_SIZE * ind ), ys, '.', _color);
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
				dma_lcd_wr_data(set_color, 1);
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

           // lcd_wr_data((set_color >> 8) & 0xFF);  		// High byte
            //lcd_wr_data(set_color & 0xFF);         		// Low byte
			dma_lcd_wr_data(set_color, 1);
        }
    }
}

//LCD functions ends
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Auxillary functions
/**
 * @brief Delays (blocking)
 * @param[in] ms: Amount of ms to delay
 */
static void delay_ms(uint16_t ms){
	volatile long long base = 7200; 	//Base
	base = base*(long long)ms;			//Mult base with ms
	while(--base) __asm__ volatile("nop");
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