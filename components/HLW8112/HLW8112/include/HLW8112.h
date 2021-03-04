#ifndef __HLW8112_H
#define __HLW8112_H	 


#include "esp_err.h"
#include "esp_event.h"
#include "esp_timer.h"
#include "cJSON.h"

//8112/8110 reg define
// Read and Write
#define REG_SYSCON_ADDR         0x00
#define REG_EMUCON_ADDR         0x01
#define REG_HFCONST_ADDR        0x02
#define REG_PASTART_ADDR		0x03
#define REG_PBSTART_ADDR		0x04
#define REG_PAGAIN_ADDR			0x05
#define REG_PBGAIN_ADDR			0x06
#define REG_PHASEA_ADDR			0x07
#define REG_PHASEB_ADDR			0x08
#define REG_PAOS_ADDR			0x0A
#define REG_PBOS_ADDR			0x0B
#define REG_RMSIAOS_ADDR		0x0E
#define REG_RMSIBOS_ADDR		0x0F
#define REG_IBGAIN_ADDR			0x10
#define REG_PSGAIN_ADDR			0x11
#define REG_PSOS_ADDR			0x12
#define REG_EMUCON2_ADDR        0x13
#define REG_DCIA_ADDR			0x14
#define REG_DCIB_ADDR			0x15
#define REG_DCIC_ADDR			0x16
#define REG_SAGCYC_ADDR			0x17
#define REG_SAGLVL_ADDR			0x18
#define REG_OVLVL_ADDR			0x19
#define REG_OIALVL_ADDR			0x1A
#define REG_OIBLVL_ADDR			0x1B
#define REG_INT_ADDR			0x1D
#define REG_PFCNTPA_ADDR		0x20
#define REG_PFCNTPB_ADDR		0x21

// Read only
#define REG_ANGLE_ADDR        	0x22		//相角寄存器
#define REG_UFREQ_ADDR          0x23     	//市电线性频率地址
#define REG_RMSIA_ADDR          0x24
#define REG_RMSIB_ADDR          0x25
#define REG_RMSU_ADDR           0x26
#define REG_PF_ADDR             0x27
#define REG_ENERGY_PA_ADDR		0x28
#define REG_ENERGY_PB_ADDR		0x29
#define REG_POWER_PA_ADDR       0x2C
#define REG_POWER_PB_ADDR       0x2D
#define REG_POWER_S_ADDR        0x2E
#define REG_EMUSTATUS_ADDR      0x2F
#define REG_PEAK_IA_ADDR		0x30
#define REG_PEAK_IB_ADDR		0x31
#define REG_PEAK_IU_ADDR		0x32
#define REG_INSTA_IA_ADDR		0x33
#define REG_INSTA_IB_ADDR		0x34
#define REG_INSTA_IU_ADDR		0x35
#define REG_WAVE_IA_ADDR		0x36
#define REG_WAVE_IB_ADDR		0x37
#define REG_WAVE_IU_ADDR		0x38
#define REG_INSTA_P_ADDR		0x3C
#define REG_INSTA_S_ADDR		0x3D
#define REG_IE_ADDR          	0x40
#define REG_IF_ADDR          	0x41
#define REG_RIF_ADDR          	0x42
#define REG_SYS_STATUS_ADDR     0x43
#define REG_RDATA_ADDR          0x44

#define REG_CHECKSUM_ADDR		0x6f
#define REG_RMS_IAC_ADDR		0x70
#define REG_RMS_IBC_ADDR		0x71
#define REG_RMS_UC_ADDR			0x72
#define REG_POWER_PAC_ADDR		0x73
#define REG_POWER_PBC_ADDR		0x74
#define REG_POWER_SC_ADDR		0x75
#define REG_ENERGY_AC_ADDR		0x76
#define REG_ENERGY_BC_ADDR		0x77

#define REG_TRIM_RC_ADDR		0x7C
#define REG_TRIM_VREF_ADDR		0x7D

#define REG_CHIP_ID_ADDR		0x7F



enum {
    SWITCH1_GPIO,
    SWITCH2_GPIO,
    TURNON_1,
    TURNON_2,
    TURNOFF_1,
    TURNOFF_2,
    INUSE_PRESSED,
    INUSE_RELEASED,
};

void metering_thread_entry(void *p);
void uart_init();
void Clear_RxBuf(void);
void uart_event_loop();
void HLW8112_init();
void Uart_HLW8112_Reset();
void Uart_IO_Reset();
void Uart_Read_HLW8112_Reg(unsigned char ADDR_Reg,unsigned char u8_reg_length);
void Uart_Write_HLW8112_Reg(unsigned char ADDR_Reg,unsigned char u8_reg_length,unsigned long u32_data);
void Uart_HLW8112_WriteREG_EN();
void Uart_HLW8112_Set_Channel_A();	
void Uart_HLW8112_Set_Channel_B();	
void Uart_HLW8112_WriteREG_DIS();
unsigned char Judge_CheckSum_HLW8112_Calfactor(void);
void Start_Send_UartData(unsigned char len);
unsigned char HLW8112_checkSum_Read(unsigned char u8_Reg_length);
void Calculate_HLW8112_MeterData(void);
void HLW8112_Measure(cJSON *metering_data);

void Read_HLW8112_IA();
void Read_HLW8112_PA();
void Read_HLW8112_EA();
void Read_HLW8112_IB();
void Read_HLW8112_PB();
void Read_HLW8112_EB();
float Read_HLW8112_Angle();

void Read_HLW8112_U();
void Read_HLW8112_LineFreq();
float Read_HLW8112_PF();

void Queue_Reset_A();
void Queue_Reset_B();

void StartTimer(uint32_t timer);
void StopTimer();

void Reset_Energy_A();
void Reset_Energy_B();


#endif
