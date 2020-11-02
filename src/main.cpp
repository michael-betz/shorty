#include<Arduino.h>
extern "C" {
  #include "utility/twi.h"
}

#define I2C_MAG 0x1E
#define I2C_ACC 0x19
#define PIN_PWR 3

typedef struct {
  int16_t x;
  int16_t z;
  int16_t y;
} t_meas;

t_meas m_on;
t_meas m_off;

static void pf(const char *format, ...)
{
  char buf[128];
  va_list argptr;
  va_start(argptr, format);
  vsnprintf(buf, sizeof(buf), format, argptr);
  va_end(argptr);

  Serial.print(buf);
}

static void write_reg(uint8_t reg, uint8_t val)
{
  uint8_t buf[2];
  buf[0] = reg;
  buf[1] = val;
  uint8_t ret = twi_writeTo(I2C_MAG, buf, 2, true, true);

  if (ret != 0)
    pf("write_reg() err: %d\n", ret);
}

static uint8_t read_reg(uint8_t reg)
{
  uint8_t dat = 0;
  uint8_t ret1 = twi_writeTo(I2C_MAG, &reg, 1, true, false);
  uint8_t ret2 = twi_readFrom(I2C_MAG, &dat, 1, true);

  if (ret1 != 0 || ret2 != 1)
    pf("read_reg() error\n");

  return dat;
}

static void get_meas(t_meas *m, uint8_t N)
{
  uint8_t buf[7], reg=3;
  m->x = 0;
  m->z = 0;
  m->y = 0;

  for (uint8_t i=0; i<N; i++) {
    write_reg(0x02, 1); // single shot mode

    delay(4);

    // Read results, starting from address 0x03
    twi_writeTo(I2C_MAG, &reg, 1, true, false);
    uint8_t ret2 = twi_readFrom(I2C_MAG, buf, sizeof(buf), true);

    if (ret2 != sizeof(buf)) {
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
  uint8_t reg = 0x31;
  uint8_t buf[2];
  twi_writeTo(I2C_MAG, &reg, 1, true, false);
  twi_readFrom(I2C_MAG, buf, 2, true);
  int temp = (buf[0] << 8) | buf[1];
  return (temp >> 5);
}

void setup() {
  pinMode(PIN_PWR, OUTPUT);

  Serial.begin(115200);
  pf("Yo! This is shorty!\n");

  twi_init();
  twi_setFrequency(400000);

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

void loop() {
  static unsigned frm = 0;

  digitalWrite(PIN_PWR, 1);
  get_meas(&m_on, 1);

  digitalWrite(PIN_PWR, 0);
  get_meas(&m_off, 1);

  pf("%5d, %5d, %5d\n", m_on.y - m_off.y, m_on.x - m_off.x, m_on.z - m_off.z);
  // pf("%5d, %5d, %5d\n", m_on.x, m_off.x, m_on.x - m_off.x);

  // delay(200);
  frm++;
}
