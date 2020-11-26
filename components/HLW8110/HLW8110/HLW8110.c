/*==========================================================
  * File Name	 : HLW8110.c
  * Describe 	 : HLW8110 UART 通讯程序,使用USATR2
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
#define TXD_PIN (GPIO_NUM_26)
#define RXD_PIN (GPIO_NUM_27)

//S2
//#define TXD_PIN (GPIO_NUM_18)
//#define RXD_PIN (GPIO_NUM_19)

#define UART_TASK_PRIORITY  7
#define UART_TASK_STACKSIZE 4 * 1024
#define UART_TASK_NAME      "UARTRX"

#define METERING_LOGGING  0


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
unsigned int    U16_HFConst_RegData;            //HFCONST寄存器值
unsigned int    U16_RMSUC_RegData;            //电压系数
unsigned long   U32_RMSU_RegData;             //U通道电压寄存器 
unsigned int    U16_PowerSC_RegData;          //视在功率转换系数,如果选择A通道，则是A通道视在功率转换系数。A和B通道只能二者选其一
/*---------------------A通道-------------------------------*/
unsigned int    U16_RMSIAC_RegData;           //A通道电流系数   
unsigned int    U16_PowerPAC_RegData;         //A通道功率系数   
unsigned int    U16_EnergyAC_RegData;         //A通道电量系数   
unsigned long   U32_RMSIA_RegData;            //A通道电流寄存器有效值
unsigned long   U32_POWERPA_RegData;          //A通道功率寄存器有效值 
unsigned long   U32_ENERGY_PA_RegData;        //A通道有功电能(量)寄存器有效值 
/*---------------------B通道-------------------------------*/
unsigned int    U16_RMSIBC_RegData;           //B通道电流系数,HLW8110硬件上没有，但寄存器存在 
unsigned int    U16_PowerPBC_RegData;         //B通道功率系数,HLW8110硬件上没有，但寄存器存在 
unsigned int    U16_EnergyBC_RegData;         //B通道电量系数,,HLW8110硬件上没有，但寄存器存在
unsigned long   U32_ENERGY_PB_RegData;        //A通道有功电能(量)寄存器有效值 


/*---------------------------------------------------------*/
unsigned char	B_ReadReg_Time_EN;				//串口读取寄存器数据，时间计数器标志位，1--开启计数，0--关闭计数
unsigned char	B_Tx_Finish;
unsigned char	B_Rx_Finish;
unsigned char	B_Rx_Data_ING;					//接收数据标志位	,		< 1:接收数据中,0:未接收到数据 >
unsigned char	B_Read_Error;					//UART读取出据校验和出错,< 1:数据读错，0:数据读取正确 >
unsigned char	u8_TxBuf[32]; 
unsigned char	u8_RxBuf[32];
unsigned char	u8_TX_Length;
unsigned char	u8_RX_Length;
unsigned char	u8_RX_Index;
unsigned char	u8_ReadReg_Index;
unsigned char	u8_ReadReg_Time;				//串口读取寄存器数据的时间

float   U32_AC_V;                               //电压值有效值
float   U32_AC_I_A;                       //A通道电流有效值
float   U32_AC_P_A;                               //A通道有功功率
float   U32_AC_E_A;                       //A通道有功电能(量)
float   U32_AC_BACKUP_E_A;                //A通道电量备份   
float   U8_AC_PF_A;                       //功率因素，A通道和B通道只能选其一 
float   U8_Angle_A;

float   U16_AC_LINE_Freq;               //市电线性频率
float   U16_IF_RegData;                 //IF寄存器值

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
 * Describe : UART2串口发送数据
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
    }
    free(dtmp);
    dtmp = NULL;

}
/*==========================================================
 * Function : void Clear_RxBuf(void)
 * Describe : 在准备接收串口数据前，清空接收缓存器的数据
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
 * Describe : 复位UART口
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
    u8_RX_Length = u8_reg_length + 1;   // +1 是最后有一个校验和
    
    
    Clear_RxBuf();  //清空接收缓冲区
    Start_Send_UartData(u8_TX_Length);
}
/*=====================================================
 * Function : void Uart_Write_HLW8110_Reg(unsigned char ADDR_Reg,unsigned char u8_reg_length,unsigned long u32_data)
 * Describe : 写寄存器命令，u8_reg_length：写入的寄存器数据字节长度
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
 * Describe : 验证地址0x70-0x77地址的系数和
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
 * Describe : 读取A通道电流
 * Input    : none
 * Output   : none
 * Return   : none
 * Record   : 2018/05/09
=====================================================*/
void Read_HLW8110_IA(void)
{ 
    float a=0;
    Uart_Read_HLW8110_Reg(REG_RMSIA_ADDR,3);
    vTaskDelay(20 / portTICK_PERIOD_MS);
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
//   a  = a/0x1600000;                     //电流计算出来的浮点数单位是mA,比如5003.12 
   a  = a/0x1000000;                     //电流计算出来的浮点数单位是mA,比如5003.12 
   a = a/1000;              //转换成整数5003,表示电流5003mA
   U32_AC_I_A = a;

}


/*=====================================================
 * Function : void Read_HLW8110_U(void)
 * Describe : 读取电压
 * Input    : none
 * Output   : none
 * Return   : none
 * Record   : 2019/04/03
=====================================================*/
void Read_HLW8110_U(void)
{
    float a=0;
    Uart_Read_HLW8110_Reg(REG_RMSU_ADDR,3);

    vTaskDelay(20 / portTICK_PERIOD_MS);
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
 //   a = a/0x400000;       //电压计算出来的浮点数单位是1mV,比如22083.12，表示220.8312V
    a = a/0x220AD0;       //电压计算出来的浮点数单位是1mV,比如22083.12，表示220.8312V
    a = a/100;           
    U32_AC_V = a;
}


/*=====================================================
 * Function : void Read_HLW8110_PA(void)
 * Describe : 读取A通道功率
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
    a = a/0x80000000;             //单位为mW,比如算出来5000123，表示5000.123W
    U32_AC_P_A = a;

}

/*=====================================================
 * Function : void void Read_HLW8110_EA(void)
 * Describe : 读取B通道有功电量
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
  a = a/0x20000000;             //电量单位是0.001KWH,比如算出来是2.002,表示2.002KWH 
//因为K1和K2都是1，所以a/(K1*K2) = a    
  U32_AC_E_A = a;
  
  
  if (U32_AC_E_A >= 1)    //每读到0.001度电就清零
  {
    U32_AC_BACKUP_E_A += U32_AC_E_A;
    
    
  //IO_HLW8110_SDI = LOW;
    Uart_HLW8110_WriteREG_EN();

  //清零 REG_ENERGY_PB_ADDR寄存器
    Uart_Write_HLW8110_Reg(REG_EMUCON2_ADDR,2,0x03ff);  //0x0001是EMUCON2的默认值，waveEn = 1,zxEn = 1，A通道电量寄存器，读后不清0，EPA_CB = 1；打开功率因素检测


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
      //每读到0.001度电就清零,然后再设置读后不清零

      Uart_Write_HLW8110_Reg(REG_EMUCON2_ADDR,2,0x07ff);  //0x0001是EMUCON2的默认值，waveEn = 1,zxEn = 1，A通道电量寄存器，读后不清0，EPA_CB = 1；打开功率因素检测
      vTaskDelay(30 / portTICK_PERIOD_MS);
      Uart_HLW8110_WriteREG_DIS();   //关闭写8110 Reg
      vTaskDelay(30 / portTICK_PERIOD_MS);

  }
}

void Reset_8110_Energy_A()
{
     //IO_HLW8110_SDI = LOW;
    Uart_HLW8110_WriteREG_EN();

  //清零 REG_ENERGY_PB_ADDR寄存器
    Uart_Write_HLW8110_Reg(REG_EMUCON2_ADDR,2,0x03ff);  //0x0001是EMUCON2的默认值，waveEn = 1,zxEn = 1，A通道电量寄存器，读后不清0，EPA_CB = 1；打开功率因素检测


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
  //每读到0.001度电就清零,然后再设置读后不清零

  Uart_Write_HLW8110_Reg(REG_EMUCON2_ADDR,2,0x07ff);  //0x0001是EMUCON2的默认值，waveEn = 1,zxEn = 1，A通道电量寄存器，读后不清0，EPA_CB = 1；打开功率因素检测
  vTaskDelay(20 / portTICK_PERIOD_MS);
  Uart_HLW8110_WriteREG_DIS();   //关闭写8110 Reg
  vTaskDelay(20 / portTICK_PERIOD_MS);
}


/*=====================================================
 * Function : void Read_HLW8110_LineFreq(void)
 * Describe : 读取A通道的线性频率
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
 * Describe : 读取功率因素
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

//测量A通道的功率因素，需要发送EA+5A命令
//测量B通道的功率因素，需要发送EA+A5命令    

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

  if (b>0x800000)       //为负，容性负载
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
  

//功率因素*100，最大为100，最小负100
  F_PowerFactor = a;
  return F_PowerFactor;
  
}


/*=====================================================
 * Function : void Read_HLW8110_Angle(void)
 * Describe : 读取相位角
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
    
    if ( U16_AC_LINE_Freq < 55) //线性频率50HZ
    {
        a = b;
        a = a * 0.0805;
        F_Angle = a;
    }
    else
    {
        //线性频率60HZ
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

//    Uart_Write_HLW8110_Reg(REG_SYSCON_ADDR,2,0x0a04);   //开启A通道，关闭B通道，电压通道PGA = 1，电流通道PGA = 16
    Uart_Write_HLW8110_Reg(REG_SYSCON_ADDR,2,0x0a04);   //开启A通道，关闭B通道，电压通道PGA = 1，电流通道PGA = 16
    vTaskDelay(20 / portTICK_PERIOD_MS);
    
    /*------    过零输出设置-------以下代码，如果不需要可以屏蔽 ------*/
    ESP_LOGI(TAG, "Setup Counter");
//    Uart_Write_HLW8110_Reg(REG_EMUCON_ADDR,2,0x0001);   //1，使能PFA 脉冲输出和有功电能寄存器累加；
    Uart_Write_HLW8110_Reg(REG_EMUCON_ADDR,2,0x0001);   //1，使能PFA 脉冲输出和有功电能寄存器累加；

    vTaskDelay(20 / portTICK_PERIOD_MS);
//  Uart_Write_HLW8110_Reg(REG_EMUCON_ADDR,2,0x0018);   //正向和负向过零点均发生变化，ZXD0 = 1，ZXD1 = 1
    ESP_LOGI(TAG, "EMNUCON2 setup");
//    Uart_Write_HLW8110_Reg(REG_EMUCON2_ADDR,2,0x0465);  //0x0001是EMUCON2的默认值，waveEn = 1,zxEn = 1，A通道电量寄存器，读后不清0，EPA_CB = 1；打开功率因素检测
    Uart_Write_HLW8110_Reg(REG_EMUCON2_ADDR,2,0x07ff);  //0x0001是EMUCON2的默认值，waveEn = 1,zxEn = 1，A通道电量寄存器，读后不清0，EPA_CB = 1；打开功率因素检测

    vTaskDelay(20 / portTICK_PERIOD_MS);
    /*------    过零输出设置-------以上代码，如果不需要可以屏蔽 ------*/
    /*------    过压输出设置-------以下代码，如果不需要可以屏蔽 ------*/
    /*------    过压输出设置-------以上代码，如果不需要可以屏蔽 ------*/
    /*------    过流输出设置-------以下代码，如果不需要可以屏蔽 ------*/
    /*------    过流输出设置-------以上代码，如果不需要可以屏蔽 ------*/
    /*------    过载输出设置-------以下代码，如果不需要可以屏蔽 ------*/
    /*------    过载输出设置-------以上代码，如果不需要可以屏蔽 ------*/
    ESP_LOGI(TAG, "UART Write DIS");
    Uart_HLW8110_WriteREG_DIS();
    vTaskDelay(20 / portTICK_PERIOD_MS);
  //读取地址是0x6F至0x77的寄存器，验证系数是否正确
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

//    Read_HLW8110_IA();
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
//    item_lv2 = cJSON_GetObjectItem(item_lv1,"A");
//    cJSON_SetNumberValue(item_lv2,U32_AC_I_A);
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

    vTaskDelete(NULL);

}
