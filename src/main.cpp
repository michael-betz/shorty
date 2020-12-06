#include<Arduino.h>
extern "C" {
	#include "i2cmaster.h"
}

#include "ssd1306.h"
#include "magneto.h"
#include "main.h"

t_meas m_on;
t_meas m_off;
t_meas m_diff;

static void v_sub(t_meas *a, t_meas *b, t_meas *res)
{
	res->x = a->x - b->x;
	res->y = a->y - b->y;
	res->z = a->z - b->z;
}

void pf(const char *format, ...)
{
	char buf[128];
	va_list argptr;
	va_start(argptr, format);
	vsnprintf(buf, sizeof(buf), format, argptr);
	va_end(argptr);

	Serial.print(buf);
}

void setup()
{
	pinMode(PIN_PWR, OUTPUT);

	Serial.begin(115200);
	pf("Yo! This is shorty!\n");

	i2c_init();

	pf("I2C devices: ");
	for (unsigned i=0; i<127; i++) {
		uint8_t ret = i2c_start(i << 1);
		i2c_stop();
		if (ret == 0)
			pf("%02x ", i);
	}
	pf("\n");

	magneto_init();
	ssd_init();
}

void loop()
{
	digitalWrite(PIN_PWR, 1);
	get_meas(&m_on, 1);

	digitalWrite(PIN_PWR, 0);
	get_meas(&m_off, 1);

	v_sub(&m_on, &m_off, &m_diff);
	// pf("%5d, %5d, %5d\n", m_on.y, m_on.x, m_on.z);
	// pf("%5d, %5d, %5d\n", m_off.y, m_off.x, m_off.z);
	pf("%5d, %5d, %5d\n", m_diff.y, m_diff.x, m_diff.z);

	fill(0);

	line(64, 32, 64 + m_diff.y / 8, 32 - m_diff.x / 8);
	ssd_send();
}
