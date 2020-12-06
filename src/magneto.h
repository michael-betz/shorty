#ifndef MAGNETO_H
#define MAGNETO_H

typedef struct {
    int16_t x;
    int16_t z;
    int16_t y;
} t_meas;

void get_meas(t_meas *m, uint8_t N);
int getTemp();
void magneto_init();

#endif
