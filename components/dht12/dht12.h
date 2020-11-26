/* dht12.h */
#ifndef DHT12_H
#define DHT12_H

int Dht12Init();

void Dht12Thread();

float TempGet(void);

float HumGet(void);

typedef struct {
	uint8_t State;
	float Hum;
	float Temp;
} sDht12Data;

sDht12Data Dht12DataGet(void);

#endif