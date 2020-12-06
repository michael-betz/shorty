extern "C" {
	#include "i2cmaster.h"
}
#include "Arduino.h"
#include "main.h"
#include "magneto.h"

#define I2C_MAG  0x1E  // 7 bit
#define I2C_ACC  0x19

static void write_reg(uint8_t reg, uint8_t val)
{
	uint8_t ret = i2c_start(I2C_MAG << 1);
	ret |= i2c_write(reg);
	ret |= i2c_write(val);
	i2c_stop();

	if (ret)
		pf("magneto write_reg() err\n");
}

static uint8_t read_reg(uint8_t reg)
{
	uint8_t val;

	uint8_t ret = i2c_start(I2C_MAG << 1);
	ret |= i2c_write(reg);
	ret |= i2c_rep_start((I2C_MAG << 1) | 1);
	val = i2c_readNak();
	i2c_stop();

	if (ret)
		pf("magneto read_reg() err\n");

	return val;
}

void get_meas(t_meas *m, uint8_t N)
{
	uint8_t buf[7], reg=3;
	m->x = 0;
	m->z = 0;
	m->y = 0;

	for (uint8_t i=0; i<N; i++) {
		write_reg(0x02, 1); // single shot mode

		delay(4);

		// Read results, starting from address 0x03
		uint8_t ret = i2c_start(I2C_MAG << 1);
		ret |= i2c_write(reg);
		ret |= i2c_rep_start((I2C_MAG << 1) | 1);
		for (uint8_t i=0; i<sizeof(buf); i++)
			if (i < sizeof(buf) - 1)
				buf[i] = i2c_readAck();
			else
				buf[i] = i2c_readNak();

		if (ret) {
			pf("get_meas() error");
			return;
		}

		m->x += buf[0] << 8 | buf[1];
		m->z += buf[2] << 8 | buf[3];
		m->y += buf[4] << 8 | buf[5];
	}
}

// there seems to be only 2 bits active in TEMP_OUT_L_M???
// scaling is screwed up too. WTF?
int getTemp()
{
	uint8_t ret = i2c_start(I2C_MAG << 1);
	ret |= i2c_write(0x31);
	ret |= i2c_rep_start((I2C_MAG << 1) | 1);

	int temp = (i2c_readAck() << 8);
	temp |= i2c_readNak();

	if (ret) {
		pf("getTemp() error");
		return 0;
	}

	return (temp >> 5);
}

void magneto_init()
{
	pf(
		"ID (48 34 33): %x %x %x\n",
		read_reg(0x0A),
		read_reg(0x0B),
		read_reg(0x0C)
	);

	write_reg(0x00, (1 << 7) | (4 << 2));  // enable temp sensor, 15 Hz rate
	write_reg(0x01, (1 << 5));  // max. gain (+- 1.3 Gauss range)
	write_reg(0x02, 1); // single shot mode
}
