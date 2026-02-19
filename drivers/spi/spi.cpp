#include "spi.hpp"
extern "C" {
#include "gd32vf103_spi.h"
}

Spi::Spi(Port _port)
{	
	initSpiPeriph(_port);

	spi_parameter_struct params;
	spi_struct_para_init(&params);
	params.device_mode = SPI_MASTER;
	params.trans_mode = SPI_TRANSMODE_BDTRANSMIT;
	params.frame_size = SPI_FRAMESIZE_16BIT;
	params.nss = SPI_NSS_SOFT;
	params.endian = SPI_ENDIAN_MSB;
	params.clock_polarity_phase = SPI_CK_PL_LOW_PH_1EDGE;
	params.prescale = SPI_PSC_4;
	//spi_init();
}



/* Private */
void Spi::initSpiPeriph(Port _port)
{
	switch(_port)
	{
		case Port::spi0:


			break;
		case Port::spi1:


			break;
		case Port::spi2: 



			break;
	}
}


