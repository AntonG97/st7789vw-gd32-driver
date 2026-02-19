#include <cstdint>

class Spi
{
	public:

		enum class Port : int {spi0,spi1,spi2};
	
	Spi(Port _port);

	~Spi(void);

	private:
	uint32_t _spiPeriph;

	void initSpiPeriph(Port _port);

};
