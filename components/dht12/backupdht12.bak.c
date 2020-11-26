/* dht12.c */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <stdio.h>
#include <string.h>

#include "dht12.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "config.h"
#include <esp_log.h>


#define DHT12_SDA_IO_NUM		18
#define DHT12_SCL_IO_NUM		19
#define CONFIG_DHT12_I2C_FREQ	100000

#define DHT12_OK				0
#define DHT12_ERROR_CHECKSUM	-10
#define DHT12_ERROR_CONNECT		-11
#define DHT12_MISSING_BYTES		-12
#define DHT12_ADDRESS			((u8)0xB8)

#define MOD_TAG					"DHT1X"


static struct {
	float Temp;
	float Hum;
	uint8_t SensorAnswerFlag;
	uint8_t SensorErrorFlag;
} Dht12State;

static sDht12Data Dht12Data;

SemaphoreHandle_t	I2C1_Lock;

sDht12Data Dht12DataGet(void)
{
	sDht12Data Temp;
	xSemaphoreTake(I2C1_Lock, portMAX_DELAY);
	memcpy(&Temp, &Dht12Data, sizeof(Dht12Data));
	xSemaphoreGive(I2C1_Lock);
	return Temp;
}


int Dht12Init()
{
// Convert to i2cdev for thread safe calls
	int Ret = -1;


	i2c_config_t I2cConfig = {
		.mode = I2C_MODE_MASTER,
		.sda_io_num = DHT12_SDA_IO_NUM,
		.sda_pullup_en = GPIO_PULLUP_ENABLE,
		.scl_io_num = DHT12_SCL_IO_NUM,
		.scl_pullup_en = GPIO_PULLUP_ENABLE,
		.master.clk_speed = CONFIG_DHT12_I2C_FREQ
	};

	Ret = i2c_param_config(I2C_NUM_1, &I2cConfig);

	if(Ret != ESP_OK) {
		ESP_LOGE(MOD_TAG, "I2C config failed");
		return -1;
	}

	Ret = i2c_driver_install(I2C_NUM_1, I2cConfig.mode, 0, 0, 0);

	if(Ret != ESP_OK) {
		ESP_LOGE(MOD_TAG, "I2C driver install failed");
		return -1;
	}


	I2C1_Lock = xSemaphoreCreateMutex();
	if (!I2C1_Lock) {
		return -1;
	}

	ESP_LOGW(MOD_TAG, "Inited.");
	
	return 0;
}



/* I2C */
static esp_err_t Dht12Read()
{

	//******* Should change to i2cdev for threadsafe common usage. 
	int Ret = -1;
	uint8_t Buffer[10];
	memset(Buffer, 0, 10);
	i2c_cmd_handle_t I2cHandle = i2c_cmd_link_create();
	i2c_master_start(I2cHandle);
	i2c_master_write_byte(I2cHandle, (uint8_t)0xB8, I2C_MASTER_ACK);
	i2c_master_write_byte(I2cHandle, (uint8_t)0x0, I2C_MASTER_ACK);

	i2c_master_start(I2cHandle);
	i2c_master_write_byte(I2cHandle, (uint8_t)0xB9, I2C_MASTER_ACK);
	i2c_master_read_byte(I2cHandle, &Buffer[0], I2C_MASTER_ACK);
	i2c_master_read_byte(I2cHandle, &Buffer[1], I2C_MASTER_ACK);
	i2c_master_read_byte(I2cHandle, &Buffer[2], I2C_MASTER_ACK);
	i2c_master_read_byte(I2cHandle, &Buffer[3], I2C_MASTER_ACK);
	i2c_master_read_byte(I2cHandle, &Buffer[4], I2C_MASTER_NACK);
	i2c_master_stop(I2cHandle);
	Ret = i2c_master_cmd_begin(I2C_NUM_1, I2cHandle, 100 / portTICK_RATE_MS);
	i2c_cmd_link_delete(I2cHandle);

	if(Ret != ESP_OK) {
		Dht12Data.State = 1;
		ESP_LOGE(MOD_TAG, "Data was not vaild");
		return -1;
	}

	uint8_t Checksum = Buffer[0] + Buffer[1] + Buffer[2] + Buffer[3];
	if (Buffer[4] != Checksum) {
		Dht12Data.State = 1;
		ESP_LOGE(MOD_TAG, "Data was not vaild");
		return -1;
	} else {
		Dht12State.Hum = Buffer[0] + Buffer[1] * 0.1;
		Dht12State.Temp = Buffer[2] + (Buffer[3] & 0x7F) * 0.1;
		if (Buffer[4] & 0x80) {
			Dht12State.Temp = -Dht12State.Temp;
		}
		Dht12Data.State = 0;
		Dht12Data.Temp = Dht12State.Temp;
		Dht12Data.Hum = Dht12State.Hum;
		return 0;
	}
}

void Dht12Thread()
{	
	Dht12Init();
	vSemaphoreCreateBinary( I2C1_Lock );
	int Ret = 0;
	while(1) {
		vTaskDelay(5000 / portTICK_RATE_MS);
		xSemaphoreTake(I2C1_Lock, portMAX_DELAY);
		Ret = Dht12Read();
		xSemaphoreGive(I2C1_Lock);
		if (Ret != ESP_OK) {
			ESP_LOGE(MOD_TAG, "Dht12 data was not vaild");
		} else {
			ESP_LOGI(MOD_TAG, "Hum: %.2f, Temp: %.2f", Dht12State.Hum, Dht12State.Temp);
		}
		
	}
}