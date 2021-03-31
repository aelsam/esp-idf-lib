/*==========================================================
  * File Name	 : HLW8110.c
  * Describe 	 : HLW8110 UART ͨѶ����,ʹ��USATR2
  * Author	     : Tuqiang
  * Version	     : V1.2
  * Record	     : 2019/04/16
===========================================================*/
#include "HLW8110.h"
#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "soc/uart_struct.h"

#include "freertos/queue.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_event_base.h"
#include "esp_timer.h"

#include "cJSON.h"
#include "esp_http_client.h"

#include "esp_err.h"
#include <esp_log.h>


//extern unsigned char UART_EN;
//extern unsigned char SPI_EN;
static const char *TAG = "HLW8110_UART";

#define EX_UART_NUM UART_NUM_1

static QueueHandle_t uart2_queue;

static const int TX_BUF_SIZE =128;
static const int RX_BUF_SIZE = 128;

// Setup UART pins
//S1
// VUL100
//#define TXD_PIN (GPIO_NUM_26)
//#define RXD_PIN (GPIO_NUM_27)
// VLS100
//#define TXD_PIN (GPIO_NUM_14)
//#define RXD_PIN (GPIO_NUM_27)


//#define I2C_MASTER_SDA_GPIO         CONFIG_SDA_GPIO
//#define I2C_MASTER_SCL_GPIO         CONFIG_SCL_GPIO
#define TXD_PIN     CONFIG_TX_GPIO
#define RXD_PIN     CONFIG_RX_GPIO


//S2
//#define TXD_PIN (GPIO_NUM_18)
//#define RXD_PIN (GPIO_NUM_19)

#define UART_TASK_PRIORITY  7
#define UART_TASK_STACKSIZE 4 * 1024
#define UART_TASK_NAME      "UARTRX"

#define METERING_LOGGING 0


int64_t realtime_timer_durations_ms;

/*---------------------------------------------------------*/
union IntData
{
	uint16_t  inte;			
	uint8_t byte[2];		
};
union LongData
{
    uint32_t  word;		
    uint16_t  inte[2];		
    uint8_t   byte[4];		
};
/*---------------------------------------------------------*/


esp_err_t err;

//---------------------------------------------------------
unsigned int    U16_SYSCON_RegData;
unsigned int    U16_EMUCON2_RegData;
unsigned int    U16_HFConst_RegData;            //HFCONST�Ĵ���ֵ
unsigned int    U16_RMSUC_RegData;            //��ѹϵ��
unsigned long   U32_RMSU_RegData;             //Uͨ����ѹ�Ĵ��� 
unsigned int    U16_PowerSC_RegData;          //���ڹ���ת��ϵ��,���ѡ��Aͨ��������Aͨ�����ڹ���ת��ϵ����A��Bͨ��ֻ�ܶ���ѡ��һ
/*---------------------Aͨ��-------------------------------*/
unsigned int    U16_RMSIAC_RegData;           //Aͨ������ϵ��   
unsigned int    U16_PowerPAC_RegData;         //Aͨ������ϵ��   
unsigned int    U16_EnergyAC_RegData;         //Aͨ������ϵ��   
unsigned long   U32_RMSIA_RegData;            //Aͨ�������Ĵ�����Чֵ
unsigned long   U32_POWERPA_RegData;          //Aͨ�����ʼĴ�����Чֵ 
unsigned long   U32_ENERGY_PA_RegData;        //Aͨ���й�����(��)�Ĵ�����Чֵ 
/*---------------------Bͨ��-------------------------------*/
unsigned int    U16_RMSIBC_RegData;           //Bͨ������ϵ��,HLW8110Ӳ����û�У����Ĵ������� 
unsigned int    U16_PowerPBC_RegData;         //Bͨ������ϵ��,HLW8110Ӳ����û�У����Ĵ������� 
unsigned int    U16_EnergyBC_RegData;         //Bͨ������ϵ��,,HLW8110Ӳ����û�У����Ĵ�������
unsigned long   U32_ENERGY_PB_RegData;        //Aͨ���й�����(��)�Ĵ�����Чֵ 


/*---------------------------------------------------------*/
unsigned char	B_ReadReg_Time_EN;				//���ڶ�ȡ�Ĵ������ݣ�ʱ���������־λ��1--����������0--�رռ���
unsigned char	B_Tx_Finish;
unsigned char	B_Rx_Finish;
unsigned char	B_Rx_Data_ING;					//�������ݱ�־λ	,		< 1:����������,0:δ���յ����� >
unsigned char	B_Read_Error;					//UART��ȡ����У��ͳ���,< 1:���ݶ���0:���ݶ�ȡ��ȷ >
unsigned char	u8_TxBuf[32]; 
unsigned char	u8_RxBuf[32];
unsigned char	u8_TX_Length;
unsigned char	u8_RX_Length;
unsigned char	u8_RX_Index;
unsigned char	u8_ReadReg_Index;
unsigned char	u8_ReadReg_Time;				//���ڶ�ȡ�Ĵ������ݵ�ʱ��

float   U32_AC_V;                               //��ѹֵ��Чֵ
float   U32_AC_I_A;                       //Aͨ��������Чֵ
float   U32_AC_P_A;                               //Aͨ���й�����
float   U32_AC_E_A;                       //Aͨ���й�����(��)
float   U32_AC_BACKUP_E_A;                //Aͨ����������   
float   U8_AC_PF_A;                       //�������أ�Aͨ����Bͨ��ֻ��ѡ��һ 
float   U8_Angle_A;

float   U16_AC_LINE_Freq;               //�е�����Ƶ��
float   U16_IF_RegData;                 //IF�Ĵ���ֵ

unsigned long u32_Measuring_Counter=0;

void uart_init() {


    const uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_EVEN,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };


    if (METERING_LOGGING)
        ESP_LOGI(TAG, "UART Param Config");
    ESP_ERROR_CHECK(uart_param_config(EX_UART_NUM, &uart_config));
    if (METERING_LOGGING)
        ESP_LOGI(TAG, "UART Set Pin");
    ESP_ERROR_CHECK(uart_set_pin(EX_UART_NUM, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    // We won't use a buffer for sending data.
    if (METERING_LOGGING)
        ESP_LOGI(TAG, "UART Install");
    ESP_ERROR_CHECK(uart_driver_install(EX_UART_NUM, RX_BUF_SIZE * 2, TX_BUF_SIZE * 2, 20, &uart2_queue, 0));
 //   uart_enable_rx_intr(EX_UART_NUM);
}
/*==========================================================
 * Function : void Start_Send_UartData(unsigned char len)
 * Describe : UART2���ڷ�������
 * Input    : none
 * Output   : none
 * Return   : none
 * Record   : 2019/04/16
===========================================================*/

void Start_Send_UartData(unsigned char len)
{
    int bytewritten=0;
    bytewritten=uart_write_bytes(EX_UART_NUM, (char *) &u8_TxBuf[0],len);
    if (METERING_LOGGING)
        ESP_LOGI(TAG, "Byte Written%d", bytewritten);
}
void uart_event_loop()
{
	uart_event_t event;

    uint8_t* dtmp = (uint8_t*) malloc(RX_BUF_SIZE);


    for(;;) {
        //Waiting for UART event.
        if(xQueueReceive(uart2_queue, (void * )&event, (portTickType)portMAX_DELAY)) {
            bzero(dtmp, RX_BUF_SIZE);
            if (METERING_LOGGING)
                ESP_LOGI(TAG, "uart[%d] event:", EX_UART_NUM);
            switch(event.type) {
                //Event of UART receving data
                /*We'd better handler data event fast, there would be much more data events than
                other types of events. If we take too much time on data event, the queue might
                be full.*/
                case UART_DATA:
                if (METERING_LOGGING)
                    ESP_LOGI(TAG, "[UART DATA]: %d", event.size);
                uart_read_bytes(EX_UART_NUM, dtmp, event.size, portMAX_DELAY);
                for (int i=0;i<event.size;i++)
                {
                    u8_RxBuf[i]=dtmp[i];
                }
  //                  ESP_LOGI(TAG, "[DATA EVT]:");
  //                  uart_write_bytes(EX_UART_NUM, (const char*) dtmp, event.size);
                break;
                //Event of HW FIFO overflow detected
                case UART_FIFO_OVF:
                ESP_LOGI(TAG, "hw fifo overflow");
                    // If fifo overflow happened, you should consider adding flow control for your application.
                    // The ISR has already reset the rx FIFO,
                    // As an example, we directly flush the rx buffer here in order to read more data.
                uart_flush_input(EX_UART_NUM);
                xQueueReset(uart2_queue);
                break;
                //Event of UART ring buffer full
                case UART_BUFFER_FULL:
                ESP_LOGI(TAG, "ring buffer full");
                    // If buffer full happened, you should consider encreasing your buffer size
                    // As an example, we directly flush the rx buffer here in order to read more data.
                uart_flush_input(EX_UART_NUM);
                xQueueReset(uart2_queue);
                break;
                //Event of UART RX break detected
                case UART_BREAK:
                ESP_LOGI(TAG, "uart rx break");
                break;
                //Event of UART parity check error
                case UART_PARITY_ERR:
                ESP_LOGI(TAG, "uart parity error");
                break;
                //Event of UART frame error
                case UART_FRAME_ERR:
                ESP_LOGI(TAG, "uart frame error");
                break;

                //Others
                default:
                ESP_LOGI(TAG, "uart event type: %d", event.type);
                break;
            }
        }
        else
        {
            ESP_LOGI(TAG, "nothing in queue");
        }

        vTaskDelay(50 / portTICK_PERIOD_MS);	// 50ms OK
//        vTaskDelay(30 / portTICK_PERIOD_MS);	// ms
    }
    free(dtmp);
    dtmp = NULL;

}
/*==========================================================
 * Function : void Clear_RxBuf(void)
 * Describe : ��׼�����մ�������ǰ����ս��ջ�����������
 * Input    : none
 * Output   : none
 * Return   : none
 * Record   : 2019/04/03
===========================================================*/
void Clear_RxBuf(void)
{
    unsigned char i;
    for(i = 0;i<10;i++)
    {
        u8_RxBuf[i] = 0x00;
    }
    
    B_Rx_Data_ING = 0;
    B_Rx_Finish = false;
    u8_RX_Index = 0;
}
/*==========================================================
 * Function : unsigned char HLW8110_checkSum_Write(unsigned char u8_Reg_length)
 * Describe : 
 * Input    : none
 * Output   : none
 * Return   : none
 * Record   : 2019/04/03
===========================================================*/
unsigned char HLW8110_checkSum_Write(unsigned char u8_Reg_length)
{
    unsigned char i;
    unsigned char Temp_u8_checksum;
    unsigned int    a;

    a = 0x0000;
    Temp_u8_checksum = 0;
    for (i = 0; i< (u8_Reg_length-1); i++)
    {
        a += u8_TxBuf[i];
    }
    
    a = ~a;
    Temp_u8_checksum = a & 0xff;

    return Temp_u8_checksum;
    
}
void Uart_HLW8110_Reset(void)
{

    u8_TX_Length = 4;
    u8_RX_Length = 0;
    
    u8_TxBuf[0] = 0xa5;
    u8_TxBuf[1] = 0xea;
    u8_TxBuf[2] = 0x96;
//  u8_TxBuf[3] = 0xda;  //checksum
    
    u8_TxBuf[3] = HLW8110_checkSum_Write(u8_TX_Length);

    Start_Send_UartData(u8_TX_Length);
}

/*=====================================================
 * Function : void Uart_IO_Reset(void)
 * Describe : ��λUART��
 * Input    : none
 * Output   : none
 * Return   : none
 * Record   : 2019/04/12
=====================================================*/
void Uart_IO_Reset(void)
{
    Uart_Read_HLW8110_Reg(REG_SYSCON_ADDR,0x02);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    Uart_Read_HLW8110_Reg(REG_SYSCON_ADDR,0x02);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    Uart_Read_HLW8110_Reg(REG_SYSCON_ADDR,0x02);
    vTaskDelay(100 / portTICK_PERIOD_MS);
}

/*=====================================================
 * Function : void Uart_Read_HLW8110_Reg(unsigned char ADDR_Reg,unsigned char u8_reg_length)
 * Describe :
 * Input    : 
 * Output   : 
 * Return   : 
 * Record   : 2019/04/04
=====================================================*/
void Uart_Read_HLW8110_Reg(unsigned char ADDR_Reg,unsigned char u8_reg_length)
{
    u8_TxBuf[0] = 0xa5;
    u8_TxBuf[1] = ADDR_Reg;
    u8_TX_Length =  2;
    u8_RX_Length = u8_reg_length + 1;   // +1 �������һ��У���
    
    
    Clear_RxBuf();  //��ս��ջ�����
    Start_Send_UartData(u8_TX_Length);
}
/*=====================================================
 * Function : void Uart_Write_HLW8110_Reg(unsigned char ADDR_Reg,unsigned char u8_reg_length,unsigned long u32_data)
 * Describe : д�Ĵ������u8_reg_length��д��ļĴ��������ֽڳ���
 * Input    : 
 * Output   : 
 * Return   : 
 * Record   : 2019/04/03
=====================================================*/
void Uart_Write_HLW8110_Reg(unsigned char ADDR_Reg,unsigned char u8_reg_length,unsigned long u32_data)
{
    unsigned char i;
    union LongData Temp_u32_a;

    
    u8_TxBuf[0] = 0xa5;
    u8_TxBuf[1] = ADDR_Reg|0x80;

    Temp_u32_a.word = u32_data;
    for (i = 0; i< u8_reg_length; i++)
    {
        u8_TxBuf[i+2] = Temp_u32_a.byte[u8_reg_length-1-i];
    }


    u8_TX_Length = 3 + u8_reg_length ;
    u8_RX_Length = 0;
    
    u8_TxBuf[u8_TX_Length-1] = HLW8110_checkSum_Write(u8_TX_Length);


    Start_Send_UartData(u8_TX_Length);
}
/*============================================================================
 * Function : unsigned char HLW8110_checkSum_Read(void)
 * Describe : 
 * Input    : none
 * Output   : none
 * Return   : none
 * Record   : 2019/04/03
=============================================================================*/
unsigned char HLW8110_checkSum_Read(unsigned char u8_Reg_length)
{
    unsigned char i;
    unsigned char Temp_u8_checksum;
    unsigned int a;

    a = 0x0000;
    Temp_u8_checksum = 0;
    for (i = 0; i< (u8_Reg_length-1); i++)
    {
        a += u8_RxBuf[i];
    }

    a = a + u8_TxBuf[0] + u8_TxBuf[1];
    a = ~a;

    Temp_u8_checksum = a & 0xff;

    return Temp_u8_checksum;
    
}
/*=====================================================
 * Function : void Judge_CheckSum_HLW8110_Calfactor(void)
 * Describe : ��֤��ַ0x70-0x77��ַ��ϵ����
 * Input    : none
 * Output   : none
 * Return   : none
 * Record   : 2019/03/18
=====================================================*/
unsigned char Judge_CheckSum_HLW8110_Calfactor(void)
{
    unsigned long a;
    unsigned int b=0;
    unsigned int c;
    unsigned char d;

    Uart_Read_HLW8110_Reg(REG_EMUCON2_ADDR,2);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    if ( u8_RxBuf[u8_RX_Length-1] == HLW8110_checkSum_Read(u8_RX_Length) )
    {
        U16_EMUCON2_RegData = (u8_RxBuf[0]<<8) + u8_RxBuf[1] ;
        if (METERING_LOGGING)
            printf("Testing Storage:%x\n " ,U16_EMUCON2_RegData);  
    }

    Uart_Read_HLW8110_Reg(REG_RMS_IAC_ADDR,2);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    if ( u8_RxBuf[u8_RX_Length-1] == HLW8110_checkSum_Read(u8_RX_Length) )
    {
        U16_RMSIAC_RegData = (u8_RxBuf[0]<<8) + u8_RxBuf[1] ;
        if (METERING_LOGGING)
            printf("Current Channel A:%x\n " ,U16_RMSIAC_RegData);
    }
    
    
    Uart_Read_HLW8110_Reg(REG_RMS_IBC_ADDR,2);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    if ( u8_RxBuf[u8_RX_Length-1] == HLW8110_checkSum_Read(u8_RX_Length) )
    {
        U16_RMSIBC_RegData = (u8_RxBuf[0]<<8) + u8_RxBuf[1] ;
        if (METERING_LOGGING)
            printf("Current Channel B:%x\n " ,U16_RMSIBC_RegData);
    }
    
    
    Uart_Read_HLW8110_Reg(REG_RMS_UC_ADDR,2);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    if ( u8_RxBuf[u8_RX_Length-1] == HLW8110_checkSum_Read(u8_RX_Length) )
    {
        U16_RMSUC_RegData = (u8_RxBuf[0]<<8) + u8_RxBuf[1] ;
        if (METERING_LOGGING)
            printf("Voltage Channel:%x\n " ,U16_RMSUC_RegData);
    }

    Uart_Read_HLW8110_Reg(REG_POWER_PAC_ADDR,2);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    if ( u8_RxBuf[u8_RX_Length-1] == HLW8110_checkSum_Read(u8_RX_Length) )
    {
        U16_PowerPAC_RegData = (u8_RxBuf[0]<<8) + u8_RxBuf[1] ;
        if (METERING_LOGGING)
            printf("Current Channel A powerfactor:%x\n " ,U16_PowerPAC_RegData);
    }

    Uart_Read_HLW8110_Reg(REG_POWER_PBC_ADDR,2);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    if ( u8_RxBuf[u8_RX_Length-1] == HLW8110_checkSum_Read(u8_RX_Length) )
    {
        U16_PowerPBC_RegData = (u8_RxBuf[0]<<8) + u8_RxBuf[1] ;
        if (METERING_LOGGING)
            printf("Current Channel B powerfactor:%x\n " ,U16_PowerPBC_RegData);
    }
    
    Uart_Read_HLW8110_Reg(REG_POWER_SC_ADDR,2);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    if ( u8_RxBuf[u8_RX_Length-1] == HLW8110_checkSum_Read(u8_RX_Length) )
    {
        U16_PowerSC_RegData = (u8_RxBuf[0]<<8) + u8_RxBuf[1] ;
        if (METERING_LOGGING)
            printf("SC factor:%x\n " ,U16_PowerSC_RegData);
    }
    
    Uart_Read_HLW8110_Reg(REG_ENERGY_AC_ADDR,2);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    if ( u8_RxBuf[u8_RX_Length-1] == HLW8110_checkSum_Read(u8_RX_Length) )
    {
        U16_EnergyAC_RegData = (u8_RxBuf[0]<<8) + u8_RxBuf[1] ;
        if (METERING_LOGGING)
            printf("A channel Energy:%x\n " ,U16_EnergyAC_RegData);

    }
    Uart_Read_HLW8110_Reg(REG_ENERGY_BC_ADDR,2);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    if ( u8_RxBuf[u8_RX_Length-1] == HLW8110_checkSum_Read(u8_RX_Length) )
    {
        U16_EnergyBC_RegData = (u8_RxBuf[0]<<8) + u8_RxBuf[1] ;
        if (METERING_LOGGING)
            printf("B Channel Energy:%x\n " ,U16_EnergyBC_RegData);
    }
    

    Uart_Read_HLW8110_Reg(REG_CHECKSUM_ADDR,2);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    if ( u8_RxBuf[u8_RX_Length-1] == HLW8110_checkSum_Read(u8_RX_Length) )
    {
        b = (u8_RxBuf[0]<<8) + u8_RxBuf[1] ;
        if (METERING_LOGGING)
            printf("Checksum:%x\n " ,b);
    }
    a = 0;

    a = ~(a + 0xffff+U16_RMSIAC_RegData + U16_RMSIBC_RegData + U16_RMSUC_RegData + 
        U16_PowerPAC_RegData + U16_PowerPBC_RegData + U16_PowerSC_RegData + 
        U16_EnergyAC_RegData + U16_EnergyBC_RegData  );

    c = a & 0xffff;
    if (METERING_LOGGING)
        printf("Calculated checksum:%x\n " ,c);

    if (c == b)
    {
        d = 1;
        if (METERING_LOGGING)
            printf("Checksum correct\r\n ");

    }
    else
    {
        d = 0;
        printf("Checksum Error\r\n ");
    }

    return d;

}
/*=====================================================
 * Function : void Uart_HLW8110_WriteREG_EN(void)
 * Describe : 
 * Input    : none
 * Output   : none
 * Return   : none
 * Record   : 2019/04/03
=====================================================*/
void Uart_HLW8110_WriteREG_EN(void)
{

    u8_TX_Length = 4;
    u8_RX_Length = 0;
    
    u8_TxBuf[0] = 0xa5;
    u8_TxBuf[1] = 0xea;
    u8_TxBuf[2] = 0xe5;
//  u8_TxBuf[3] = 0x8b;  //checksum
    u8_TxBuf[3] = HLW8110_checkSum_Write(u8_TX_Length);

    Start_Send_UartData(u8_TX_Length);

}

/*=====================================================
 * Function : void Uart_HLW8110_WriteREG_DIS(void)
 * Describe : 
 * Input    : none
 * Output   : none
 * Return   : none
 * Record   : 2019/04/03
=====================================================*/
void Uart_HLW8110_WriteREG_DIS(void)
{

    u8_TX_Length = 4;
    u8_RX_Length = 0;
    
    u8_TxBuf[0] = 0xa5;
    u8_TxBuf[1] = 0xea;
    u8_TxBuf[2] = 0xdc;
//  u8_TxBuf[3] = 0x94;  //checksum
    u8_TxBuf[3] = HLW8110_checkSum_Write(u8_TX_Length);

    Start_Send_UartData(u8_TX_Length);
}

/*=====================================================
 * Function : void Uart_HLW8110_Set_Channel_A(void)
 * Describe : 
 * Input    : none
 * Output   : none
 * Return   : none
 * Record   : 2019/04/03
=====================================================*/
void Uart_HLW8110_Set_Channel_A(void)
{
    u8_TX_Length = 4;
    u8_RX_Length = 0;
    
    u8_TxBuf[0] = 0xa5;
    u8_TxBuf[1] = 0xea;
    u8_TxBuf[2] = 0x5a;
//  u8_TxBuf[3] = 0x16;  //checksum
    u8_TxBuf[3] = HLW8110_checkSum_Write(u8_TX_Length);

    Start_Send_UartData(u8_TX_Length);

}


/*=====================================================
 * Function : void Read_HLW8110_IA(void)
 * Describe : ��ȡAͨ������
 * Input    : none
 * Output   : none
 * Return   : none
 * Record   : 2018/05/09
=====================================================*/
void Read_HLW8110_IA(void)
{ 
    float a=0;
    Uart_Read_HLW8110_Reg(REG_RMSIA_ADDR,3);
    vTaskDelay(50 / portTICK_PERIOD_MS);
    if ( u8_RxBuf[u8_RX_Length-1] == HLW8110_checkSum_Read(u8_RX_Length) )
    {
        U32_RMSIA_RegData = (u8_RxBuf[0]<<16) + (u8_RxBuf[1]<<8) + (u8_RxBuf[2]); 
        if (METERING_LOGGING)
            printf("Channel A current:%lx\n " ,U32_RMSIA_RegData);
    }
    else
    {
        printf("Channel A current error\r\n");
        B_Read_Error = 1;

    }
    a = (float)U32_RMSIA_RegData;
    a = a * U16_RMSIAC_RegData;
//   a  = a/0x1600000;                     //������������ĸ�������λ��mA,����5003.12 
   a  = a/0x1000000;                     //������������ĸ�������λ��mA,����5003.12 
   a = a/1000;              //ת��������5003,��ʾ����5003mA
   U32_AC_I_A = a;

}


/*=====================================================
 * Function : void Read_HLW8110_U(void)
 * Describe : ��ȡ��ѹ
 * Input    : none
 * Output   : none
 * Return   : none
 * Record   : 2019/04/03
=====================================================*/
void Read_HLW8110_U(void)
{
    float a=0;
    Uart_Read_HLW8110_Reg(REG_RMSU_ADDR,3);

    vTaskDelay(50 / portTICK_PERIOD_MS);
    if ( u8_RxBuf[u8_RX_Length-1] == HLW8110_checkSum_Read(u8_RX_Length) )
    {
        U32_RMSU_RegData = (u8_RxBuf[0]<<16) + (u8_RxBuf[1]<<8) + (u8_RxBuf[2]);
        if (METERING_LOGGING)
            printf("Voltage value:%lx\n " ,U32_RMSU_RegData);
    }
    else
    {

        printf("Voltage Value error\r\n");
        B_Read_Error = 1;
    }

    a =  (float)U32_RMSU_RegData;
    a = a*U16_RMSUC_RegData;  
 //   a = a/0x400000;       //��ѹ��������ĸ�������λ��1mV,����22083.12����ʾ220.8312V
    a = a/0x220AD0;       //��ѹ��������ĸ�������λ��1mV,����22083.12����ʾ220.8312V
    a = a/100;           
    U32_AC_V = a;
}


/*=====================================================
 * Function : void Read_HLW8110_PA(void)
 * Describe : ��ȡAͨ������
 * Input    : none
 * Output   : none
 * Return   : none
 * Record   : 2018/05/09
=====================================================*/
void Read_HLW8110_PA(void)
{
    float a=0; 
    unsigned long b=0;
    Uart_Read_HLW8110_Reg(REG_POWER_PA_ADDR,4);
    vTaskDelay(50 / portTICK_PERIOD_MS);
    if ( u8_RxBuf[u8_RX_Length-1] == HLW8110_checkSum_Read(u8_RX_Length) )
    {
        U32_POWERPA_RegData = (u8_RxBuf[0]<<24) + (u8_RxBuf[1]<<16) + (u8_RxBuf[2]<<8) + (u8_RxBuf[3]);
        if (METERING_LOGGING)
            printf("Channel A Active Power:%lx\n " ,U32_POWERPA_RegData);
    }
    else
    {
        printf("Channel A Active Power Error\r\n");
        B_Read_Error = 1;
    }

    if (U32_POWERPA_RegData > 0x80000000)
    {
     b = ~U32_POWERPA_RegData;
     a = (float)b;
 }
 else
     a =  (float)U32_POWERPA_RegData;


 a = a*U16_PowerPAC_RegData;
    a = a/0x80000000;             //��λΪmW,���������5000123����ʾ5000.123W
    U32_AC_P_A = a;

}

/*=====================================================
 * Function : void void Read_HLW8110_EA(void)
 * Describe : ��ȡBͨ���й�����
 * Input    : none
 * Output   : none
 * Return   : none
 * Record   : 2019/03/28
=====================================================*/

void Read_HLW8110_EA(void)
{
//Read EEPROM + this measurement
    float a=0;
    Uart_Read_HLW8110_Reg(REG_ENERGY_PA_ADDR,3); 
    vTaskDelay(50 / portTICK_PERIOD_MS);
    if ( u8_RxBuf[u8_RX_Length-1] == HLW8110_checkSum_Read(u8_RX_Length) )
    {
        U32_ENERGY_PA_RegData = (u8_RxBuf[0]<<16) + (u8_RxBuf[1]<<8) + (u8_RxBuf[2]);
        if(METERING_LOGGING)
            printf("Channel A energy:%lx\n " ,U32_ENERGY_PA_RegData);
    }
    else
    {
        printf("Channel A energy error\r\n");
        B_Read_Error = 1;
    }
    
    Uart_Read_HLW8110_Reg(REG_HFCONST_ADDR,2); 
    vTaskDelay(50 / portTICK_PERIOD_MS);
    if ( u8_RxBuf[u8_RX_Length-1] == HLW8110_checkSum_Read(u8_RX_Length) )
    {
        U16_HFConst_RegData = (u8_RxBuf[0]<<8) + (u8_RxBuf[1]);
        if(METERING_LOGGING)
            printf("HFCONST value = :%d\n " ,U16_HFConst_RegData);
    }
    else
    {
        printf("HFCONST value = Error\r\n");
        B_Read_Error = 1;
    }
    

    a =  (float)U32_ENERGY_PA_RegData;    
    a = a*U16_EnergyAC_RegData;
  a = a/0x20000000;             //������λ��0.001KWH,�����������2.002,��ʾ2.002KWH 
//��ΪK1��K2����1������a/(K1*K2) = a    
  U32_AC_E_A = a;
  
  
  if (U32_AC_E_A >= 1)    //ÿ����0.001�ȵ������
  {
    U32_AC_BACKUP_E_A += U32_AC_E_A;
    
    
  //IO_HLW8110_SDI = LOW;
    Uart_HLW8110_WriteREG_EN();

  //���� REG_ENERGY_PB_ADDR�Ĵ���
    Uart_Write_HLW8110_Reg(REG_EMUCON2_ADDR,2,0x03ff);  //0x0001��EMUCON2��Ĭ��ֵ��waveEn = 1,zxEn = 1��Aͨ�������Ĵ�����������0��EPA_CB = 1���򿪹������ؼ��


    vTaskDelay(50 / portTICK_PERIOD_MS);
    Uart_Read_HLW8110_Reg(REG_ENERGY_PA_ADDR,3); 
    vTaskDelay(50 / portTICK_PERIOD_MS);
    if ( u8_RxBuf[u8_RX_Length-1] == HLW8110_checkSum_Read(u8_RX_Length) )
    {
        U32_ENERGY_PA_RegData = (u8_RxBuf[0]<<16) + (u8_RxBuf[1]<<8) + (u8_RxBuf[2]);
        printf("Channel A energy:%lx\n " ,U32_ENERGY_PA_RegData);
    }
    else
    {
        printf("Channel A energy error\r\n");
        B_Read_Error= 1;
    }
    vTaskDelay(50 / portTICK_PERIOD_MS);

    U32_AC_E_A = 0;
      //ÿ����0.001�ȵ������,Ȼ�������ö�������

      Uart_Write_HLW8110_Reg(REG_EMUCON2_ADDR,2,0x07ff);  //0x0001��EMUCON2��Ĭ��ֵ��waveEn = 1,zxEn = 1��Aͨ�������Ĵ�����������0��EPA_CB = 1���򿪹������ؼ��
      vTaskDelay(30 / portTICK_PERIOD_MS);
      Uart_HLW8110_WriteREG_DIS();   //�ر�д8110 Reg
      vTaskDelay(30 / portTICK_PERIOD_MS);

  }
}

void Reset_8110_Energy_A()
{
     //IO_HLW8110_SDI = LOW;
    Uart_HLW8110_WriteREG_EN();

  //���� REG_ENERGY_PB_ADDR�Ĵ���
    Uart_Write_HLW8110_Reg(REG_EMUCON2_ADDR,2,0x03ff);  //0x0001��EMUCON2��Ĭ��ֵ��waveEn = 1,zxEn = 1��Aͨ�������Ĵ�����������0��EPA_CB = 1���򿪹������ؼ��


    vTaskDelay(20 / portTICK_PERIOD_MS);
    Uart_Read_HLW8110_Reg(REG_ENERGY_PA_ADDR,3); 
    vTaskDelay(20 / portTICK_PERIOD_MS);
    if ( u8_RxBuf[u8_RX_Length-1] == HLW8110_checkSum_Read(u8_RX_Length) )
    {
        U32_ENERGY_PA_RegData = (u8_RxBuf[0]<<16) + (u8_RxBuf[1]<<8) + (u8_RxBuf[2]);
        if(METERING_LOGGING)
            printf("Channel A Power Post Reset:%lx\n " ,U32_ENERGY_PA_RegData);
    }
    else
    {
        printf("Channel A Power error\r\n");
        B_Read_Error = 1;
    }
    vTaskDelay(20 / portTICK_PERIOD_MS);

    Uart_Read_HLW8110_Reg(REG_ENERGY_AC_ADDR,2);
    vTaskDelay(20 / portTICK_PERIOD_MS);
    if ( u8_RxBuf[u8_RX_Length-1] == HLW8110_checkSum_Read(u8_RX_Length) )
    {
        U16_EnergyAC_RegData = (u8_RxBuf[0]<<8) + u8_RxBuf[1] ;
        if(METERING_LOGGING)
            printf("A channel Energy :%x\n " ,U16_EnergyAC_RegData);

    }

    U32_AC_E_A = 0;
  //ÿ����0.001�ȵ������,Ȼ�������ö�������

  Uart_Write_HLW8110_Reg(REG_EMUCON2_ADDR,2,0x07ff);  //0x0001��EMUCON2��Ĭ��ֵ��waveEn = 1,zxEn = 1��Aͨ�������Ĵ�����������0��EPA_CB = 1���򿪹������ؼ��
  vTaskDelay(20 / portTICK_PERIOD_MS);
  Uart_HLW8110_WriteREG_DIS();   //�ر�д8110 Reg
  vTaskDelay(20 / portTICK_PERIOD_MS);
}


/*=====================================================
 * Function : void Read_HLW8110_LineFreq(void)
 * Describe : ��ȡAͨ��������Ƶ��
 * Input    : none
 * Output   : none
 * Return   : none
 * Record   : 2019/04/12
=====================================================*/
void Read_HLW8110_LineFreq(void)
{
    float a=0;
    unsigned long b=0;
    Uart_Read_HLW8110_Reg(REG_UFREQ_ADDR,2);

    vTaskDelay(20 / portTICK_PERIOD_MS);
    if ( u8_RxBuf[u8_RX_Length-1] == HLW8110_checkSum_Read(u8_RX_Length) )
    {
        b = (u8_RxBuf[0]<<8) + (u8_RxBuf[1]);
        if(METERING_LOGGING)
            printf("Line frequency:%ld\n " ,b);
    }
    else
    {
        printf("Line frequency error\r\n");
        B_Read_Error = 1;
    }
    a = (float)b;
    a = 3579545/(8*a);    
    U16_AC_LINE_Freq = a;
}

/*=====================================================
 * Function : void Read_HLW8110_PF(void)
 * Describe : ��ȡ��������
 * Input    : none
 * Output   : none
 * Return   : none
 * Record   : 2019/03/18
=====================================================*/
float Read_HLW8110_PF(void)
{
  float a=0;
  unsigned long b=0;
  float F_PowerFactor=0;

//����Aͨ���Ĺ������أ���Ҫ����EA+5A����
//����Bͨ���Ĺ������أ���Ҫ����EA+A5����    

  Uart_Read_HLW8110_Reg(REG_PF_ADDR,3);

  vTaskDelay(20 / portTICK_PERIOD_MS);
  if ( u8_RxBuf[u8_RX_Length-1] == HLW8110_checkSum_Read(u8_RX_Length) )
  {
    b = (u8_RxBuf[0]<<16) + (u8_RxBuf[1]<<8) + (u8_RxBuf[2]);
    if(METERING_LOGGING)
        printf("Power factor:%lu\n " ,b);
}
else
{
    printf("{Power factor error\r\n");
    B_Read_Error = 1;
}

  if (b>0x800000)       //Ϊ�������Ը���
  {
      a = (float)(0xffffff-b + 1)/0x7fffff;
  }
  else if (b==0x7fffff)
  {
      a=0;
  }
  else
  {
      a = (float)b/0x7fffff;
  }
  

//��������*100�����Ϊ100����С��100
  F_PowerFactor = a;
  return F_PowerFactor;
  
}


/*=====================================================
 * Function : void Read_HLW8110_Angle(void)
 * Describe : ��ȡ��λ��
 * Input    : none
 * Output   : none
 * Return   : none
 * Record   : 2019/04/12
=====================================================*/
float Read_HLW8110_Angle(void)
{
    float a=0;   
    float F_Angle=0;
    unsigned long b=0;
    Uart_Read_HLW8110_Reg(REG_ANGLE_ADDR,2);

    vTaskDelay(20 / portTICK_PERIOD_MS);
    if ( u8_RxBuf[u8_RX_Length-1] == HLW8110_checkSum_Read(u8_RX_Length) )
    {
        b = (u8_RxBuf[0]<<8) + (u8_RxBuf[1]);
        if(METERING_LOGGING)
            printf("Angle counter:%ld\n " ,b);
    }
    else
    {
        printf("Angle counter error\r\n");
        B_Read_Error = 1;
    }
    
    if ( U16_AC_LINE_Freq < 55) //����Ƶ��50HZ
    {
        a = b;
        a = a * 0.0805;
        F_Angle = a;
    }
    else
    {
        //����Ƶ��60HZ
        a = b;
        a = a * 0.0965;
        F_Angle = a;
    }

    return F_Angle;
}

void HLW8110_init()
{
    B_Read_Error = 0;
    ESP_LOGI(TAG, "UART HLW8110 Reset");
    Uart_HLW8110_Reset();
    vTaskDelay(50 / portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "UART IO Reset");
    Uart_IO_Reset();
    vTaskDelay(20 / portTICK_PERIOD_MS);
    Uart_IO_Reset();
    vTaskDelay(20 / portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "UART Read Chip ID");
    Uart_Read_HLW8110_Reg(REG_CHIP_ID_ADDR,3);
    vTaskDelay(20 / portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "Write Reg EN");
    Uart_HLW8110_WriteREG_EN();
    vTaskDelay(20 / portTICK_PERIOD_MS);

//    Uart_Write_HLW8110_Reg(REG_SYSCON_ADDR,2,0x0a04);   //����Aͨ�����ر�Bͨ������ѹͨ��PGA = 1������ͨ��PGA = 16
    Uart_Write_HLW8110_Reg(REG_SYSCON_ADDR,2,0x0a04);   //����Aͨ�����ر�Bͨ������ѹͨ��PGA = 1������ͨ��PGA = 16
    vTaskDelay(20 / portTICK_PERIOD_MS);
    
    /*------    �����������-------���´��룬�������Ҫ�������� ------*/
    ESP_LOGI(TAG, "Setup Counter");
//    Uart_Write_HLW8110_Reg(REG_EMUCON_ADDR,2,0x0001);   //1��ʹ��PFA ����������й����ܼĴ����ۼӣ�
    Uart_Write_HLW8110_Reg(REG_EMUCON_ADDR,2,0x0001);   //1��ʹ��PFA ����������й����ܼĴ����ۼӣ�

    vTaskDelay(20 / portTICK_PERIOD_MS);
//  Uart_Write_HLW8110_Reg(REG_EMUCON_ADDR,2,0x0018);   //����͸�������������仯��ZXD0 = 1��ZXD1 = 1
    ESP_LOGI(TAG, "EMNUCON2 setup");
//    Uart_Write_HLW8110_Reg(REG_EMUCON2_ADDR,2,0x0465);  //0x0001��EMUCON2��Ĭ��ֵ��waveEn = 1,zxEn = 1��Aͨ�������Ĵ�����������0��EPA_CB = 1���򿪹������ؼ��
    Uart_Write_HLW8110_Reg(REG_EMUCON2_ADDR,2,0x07ff);  //0x0001��EMUCON2��Ĭ��ֵ��waveEn = 1,zxEn = 1��Aͨ�������Ĵ�����������0��EPA_CB = 1���򿪹������ؼ��

    vTaskDelay(20 / portTICK_PERIOD_MS);
    /*------    �����������-------���ϴ��룬�������Ҫ�������� ------*/
    /*------    ��ѹ�������-------���´��룬�������Ҫ�������� ------*/
    /*------    ��ѹ�������-------���ϴ��룬�������Ҫ�������� ------*/
    /*------    �����������-------���´��룬�������Ҫ�������� ------*/
    /*------    �����������-------���ϴ��룬�������Ҫ�������� ------*/
    /*------    �����������-------���´��룬�������Ҫ�������� ------*/
    /*------    �����������-------���ϴ��룬�������Ҫ�������� ------*/
    ESP_LOGI(TAG, "UART Write DIS");
    Uart_HLW8110_WriteREG_DIS();
    vTaskDelay(20 / portTICK_PERIOD_MS);
  //��ȡ��ַ��0x6F��0x77�ļĴ�������֤ϵ���Ƿ���ȷ
    ESP_LOGI(TAG, "Checksum Calfactor");
    Judge_CheckSum_HLW8110_Calfactor();   
}
void HLW8110_Measure(cJSON *metering_data)
{
         //Set Channel A
 //   float F_PowerFactor_A=0;
 //   float F_Angle_A=0;
    cJSON *item_lv1, *item_lv2;

    if (B_Read_Error == 1)
    {

        Clear_RxBuf();

        vTaskDelay(30 / portTICK_PERIOD_MS);

        Uart_IO_Reset();
        vTaskDelay(30 / portTICK_PERIOD_MS);
        Uart_IO_Reset();
        vTaskDelay(30 / portTICK_PERIOD_MS);
        vTaskDelay(30 / portTICK_PERIOD_MS);
        B_Read_Error = 0;
        Clear_RxBuf();
    }
    u32_Measuring_Counter++;
    Uart_HLW8110_Set_Channel_A(); 

    vTaskDelay(20 / portTICK_PERIOD_MS);  

    Read_HLW8110_U();
  //  Read_HLW8110_LineFreq(); 

    Read_HLW8110_IA();
    Read_HLW8110_PA();
    Read_HLW8110_EA();

//    F_Angle_A=Read_HLW8110_Angle();
//    F_PowerFactor_A=Read_HLW8110_PF();

        //update JSON
    item_lv1 = cJSON_GetObjectItem(metering_data,"Seq");

    cJSON_SetNumberValue(item_lv1,u32_Measuring_Counter);
    item_lv1 = cJSON_GetObjectItem(metering_data,"V");
    cJSON_SetNumberValue(item_lv1,U32_AC_V);
//    item_lv1 = cJSON_GetObjectItem(metering_data,"Hz");
//    cJSON_SetNumberValue(item_lv1,U16_AC_LINE_Freq);

    item_lv1 = cJSON_GetObjectItem(metering_data,"ChA");
    item_lv2 = cJSON_GetObjectItem(item_lv1,"A");
    cJSON_SetNumberValue(item_lv2,U32_AC_I_A);
    item_lv2 = cJSON_GetObjectItem(item_lv1,"W");
    cJSON_SetNumberValue(item_lv2,U32_AC_P_A);
    item_lv2 = cJSON_GetObjectItem(item_lv1,"kWH");
    cJSON_SetNumberValue(item_lv2,U32_AC_E_A);
//    item_lv2 = cJSON_GetObjectItem(item_lv1,"PF");
//    cJSON_SetNumberValue(item_lv2,F_PowerFactor_A);
//    item_lv2 = cJSON_GetObjectItem(item_lv1,"Ang");
//    cJSON_SetNumberValue(item_lv2,F_Angle_A);

}


// Creating the metering loop
void metering_8110_thread_entry(void *p)
{

    //Set UART log level
    esp_log_level_set(TAG, ESP_LOG_INFO);
    //Init Uart 
    ESP_LOGI(TAG, "UART Initiating");
    uart_init();
    ESP_LOGI(TAG, "Starting UART loop");

    xTaskCreate(uart_event_loop, UART_TASK_NAME, UART_TASK_STACKSIZE,
        NULL,  UART_TASK_PRIORITY, NULL);

    HLW8110_init();
    vTaskDelete(NULL);

}
