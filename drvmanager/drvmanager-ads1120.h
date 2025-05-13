/*
 * drvmanager-ads1120.h
 *
 *  Created on: 2025年4月21日
 *      Author: 47015
 */

#ifndef DRVMANAGER_DRVMANAGER_ADS1120_H_
#define DRVMANAGER_DRVMANAGER_ADS1120_H_

#include "drvmanager.h"

/* ===== ADC Commands ===== */
#define ADC_CMD_RESET               0x06    // 0000 011x - 复位设备
#define ADC_CMD_START               0x08    // 0000 100x - 开始或重启转换
#define ADC_CMD_POWERDOWN           0x02    // 0000 001x - 掉电
#define ADC_CMD_RDATA               0x10    // 0001 xxxx - 读取数据命令
#define ADC_CMD_RREG                0x20    // 0010 rrnn - 从地址rr开始读取nn个寄存器
#define ADC_CMD_WREG                0x40    // 0100 rrnn - 从地址rr开始写入nn个寄存器

/* ===== Register Addresses ===== */
#define ADC_REG0                    0x00    // 0000 0000
#define ADC_REG1                    0x01    // 0000 0001
#define ADC_REG2                    0x02    // 0000 0010
#define ADC_REG3                    0x03    // 0000 0011

/* ===== Register 0 Settings ===== */
// MUX Settings
#define ADC_REG0_MUX_AIN0_AIN1      0x00    // 0000 0000
#define ADC_REG0_MUX_AIN0_AIN2      0x10    // 0001 0000
#define ADC_REG0_MUX_AIN0_AIN3      0x20    // 0010 0000
#define ADC_REG0_MUX_AIN1_AIN2      0x30    // 0011 0000
#define ADC_REG0_MUX_AIN1_AIN3      0x40    // 0100 0000
#define ADC_REG0_MUX_AIN2_AIN3      0x50    // 0101 0000
#define ADC_REG0_MUX_AIN1_AIN0      0x60    // 0110 0000
#define ADC_REG0_MUX_AIN3_AIN2      0x70    // 0111 0000
#define ADC_REG0_MUX_AIN0_AVSS      0x80    // 1000 0000
#define ADC_REG0_MUX_AIN1_AVSS      0x90    // 1001 0000
#define ADC_REG0_MUX_AIN2_AVSS      0xA0    // 1010 0000
#define ADC_REG0_MUX_AIN3_AVSS      0xB0    // 1011 0000
#define ADC_REG0_MUX_MODE_VREF      0xC0    // 1100 0000
#define ADC_REG0_MUX_MODE_AVDD_AVSS 0xD0    // 1101 0000
#define ADC_REG0_MUX_MODE_14        0xE0    // 1110 0000
#define ADC_REG0_MUX_RESERVED       0xF0    // 1111 0000

// Gain Settings
#define ADC_REG0_GAIN1              0x00    // 0000 0000
#define ADC_REG0_GAIN2              0x02    // 0000 0010
#define ADC_REG0_GAIN4              0x04    // 0000 0100
#define ADC_REG0_GAIN8              0x06    // 0000 0110
#define ADC_REG0_GAIN16             0x08    // 0000 1000
#define ADC_REG0_GAIN32             0x0A    // 0000 1010
#define ADC_REG0_GAIN64             0x0C    // 0000 1100
#define ADC_REG0_GAIN128            0x0E    // 0000 1110

// PGA Settings
#define ADC_REG0_PGA_BYPASS_ENABLE  0x00    // 0000 0000
#define ADC_REG0_PGA_BYPASS_DISABLE 0x01    // 0000 0001

/* ===== Register 1 Settings ===== */
// Data Rate Settings
#define ADC_REG1_DR_NORM_MODE_20SPS 0x00    // 0000 0000
#define ADC_REG1_DR_NORM_MODE_45SPS 0x20    // 0010 0000
#define ADC_REG1_DR_NORM_MODE_90SPS 0x40    // 0100 0000
#define ADC_REG1_DR_NORM_MODE_175SPS 0x60   // 0110 0000
#define ADC_REG1_DR_NORM_MODE_330SPS 0x80   // 1000 0000
#define ADC_REG1_DR_NORM_MODE_600SPS 0xA0   // 1010 0000
#define ADC_REG1_DR_NORM_MODE_1000SPS 0xC0  // 1100 0000

// Mode Settings
#define ADC_REG1_MODE_NORMAL        0x00    // 0000 0000
#define ADC_REG1_MODE_DUTY_CYCLE    0x08    // 0000 1000
#define ADC_REG1_MODE_TURBO         0x10    // 0001 0000
#define ADC_REG1_MODE_RESERVED      0x18    // 0001 1000

// Conversion Mode
#define ADC_REG1_CM_SINGLE          0x00    // 0000 0000  //单次转换
#define ADC_REG1_CM_CONTINUOUS      0x04    // 0000 0100  //连续转换

// Temperature Sensor
#define ADC_REG1_TS_DISABLE         0x00    // 0000 0000
#define ADC_REG1_TS_ENABLE          0x02    // 0000 0010

// Burnout Current Source
#define ADC_REG1_BCS_OFF            0x00    // 0000 0000
#define ADC_REG1_BCS_ON             0x01    // 0000 0001

/* ===== Register 2 Settings ===== */
// Reference Settings
#define ADC_REG2_VREF_INTERNAL      0x00    // 0000 0000
#define ADC_REG2_VREF_EXTERNAL_REFP0_REFN0 0x40  // 0100 0000
#define ADC_REG2_VREF_EXTERNAL_REFP1_REFN1 0x80  // 1000 0000
#define ADC_REG2_VREF_ANALOG        0xC0    // 1100 0000

// FIR Filter Settings
#define ADC_REG2_FIR_NO             0x00    // 0000 0000
#define ADC_REG2_FIR_SIMULTANEOUS   0x10    // 0001 0000
#define ADC_REG2_FIR_50             0x20    // 0010 0000
#define ADC_REG2_FIR_60             0x30    // 0011 0000

// Power Switch Settings
#define ADC_REG2_PSW_OPEN           0x00    // 0000 0000
#define ADC_REG2_PSW_CLOSE          0x08    // 0000 1000

// IDAC Settings
#define ADC_REG2_IDAC_OFF           0x00    // 0000 0000
#define ADC_REG2_IDAC_RESERVED      0x01    // 0000 0001
#define ADC_REG2_IDAC_50u           0x02    // 0000 0010
#define ADC_REG2_IDAC_100u          0x03    // 0000 0011
#define ADC_REG2_IDAC_250u          0x04    // 0000 0100
#define ADC_REG2_IDAC_500u          0x05    // 0000 0101
#define ADC_REG2_IDAC_1000u         0x06    // 0000 0110
#define ADC_REG2_IDAC_1500u         0x07    // 0000 0111

/* ===== Register 3 Settings ===== */
// I1MUX Settings
#define ADC_REG3_I1MUX_DISABLED     0x00    // 0000 0000
#define ADC_REG3_I1MUX_AIN0_REFP1   0x20    // 0010 0000
#define ADC_REG3_I1MUX_AIN1         0x40    // 0100 0000
#define ADC_REG3_I1MUX_AIN2         0x60    // 0110 0000
#define ADC_REG3_I1MUX_AIN3_REFN1   0x80    // 1000 0000
#define ADC_REG3_I1MUX_REFP0        0xA0    // 1010 0000
#define ADC_REG3_I1MUX_REFN0        0xC0    // 1100 0000
#define ADC_REG3_I1MUX_RESERVED     0xE0    // 1110 0000

// I2MUX Settings
#define ADC_REG3_I2MUX_DISABLED     0x00    // 0000 0000
#define ADC_REG3_I2MUX_AIN0_REFP1   0x04    // 0000 0100
#define ADC_REG3_I2MUX_AIN1         0x08    // 0000 1000
#define ADC_REG3_I2MUX_AIN2         0x0C    // 0000 1100
#define ADC_REG3_I2MUX_AIN3_REFN1   0x10    // 0001 0000
#define ADC_REG3_I2MUX_REFP0        0x14    // 0001 0100
#define ADC_REG3_I2MUX_REFN0        0x18    // 0001 1000
#define ADC_REG3_I2MUX_RESERVED     0x1C    // 0001 1100

// DRDY Mode
#define ADC_REG3_DRDYM_ON           0x00    // 0000 0000
#define ADC_REG3_DRDYM_OFF          0x02    // 0000 0010

#define ADC_REG3_RESERVED           0x00    // 0000 0000

/* ===== Constants for Calculations ===== */
#define ADC_U_REF                   2.5     // 参考电压
#define ADC_R_REF                   994.4010318
#define ADC_U_REF_INTER             2.048
#define ADC_I_REF                   1.0E-3
#define ADC_PRECISION               32768
#define ADC_GAIN                    1.0
#define ADC_LIMIT_MIN               9000
#define ADC_LIMIT_MAX               27600
#define ADC_EXTREF_MIN              23600
#define ADC_EXTREF_MAX              30150
#define ADC_AIN0_MIN                17000
#define ADC_AIN0_MAX                22000
#define ADC_AIN1_MIN                19000
#define ADC_AIN1_MAX                25000
#define ADC_UNAP_MIN                12000
#define ADC_UNAP_MAX                14000
#define ADC_UREF_MIN                3200
#define ADC_UREF_MAX                4800
#define TEMP_STEP                   0.03125
#define TEMP_BORDER                 8192
#define TEMP_LOW                    16385
#define TEMP_CORRECT                49152
#define TEMP_MINUS_SIXTY            1920

/* ===== Temperature Calculation Constants ===== */
#define ADC_PT100_CONST_C0          -245.19
#define ADC_PT100_CONST_C1          2.5293
#define ADC_PT100_CONST_C2          -0.066046
#define ADC_PT100_CONST_C3          4.0422E-3
#define ADC_PT100_CONST_C4          -2.0697E-6
#define ADC_PT100_CONST_C5          -0.025422
#define ADC_PT100_CONST_C6          1.6883E-3
#define ADC_PT100_CONST_C7          -1.3601E-6

/* ===== Current Range Constants ===== */
#define ADS1120_REF_R_IA            (119.5)  // 采样电阻,单位:欧
#define AD_MIN_mA                   (4.0)    // 4mA
#define AD_MAX_mA                   (20.0)   // 20mA

/* ===== Status Flags ===== */
#define ADS_IA_J1_OT                (1U << 0)
#define ADS_IA_J2_OT                (1U << 1)
#define ADS_IA_J3_OT                (1U << 2)
#define ADS_IA_J4_OT                (1U << 3)

#define ADS_D_J16_IN7_OT            (1U << 0)
#define ADS_D_J16_IN8_OT            (1U << 1)
#define ADS_D_J16_IN5_OT            (1U << 2)
#define ADS_D_J16_IN6_OT            (1U << 3)

/* ===== ADC Conversion Status ===== */
typedef enum {
    ADS_CONV_STAS_IA_0_CFG    = 0,  // 本安转换通道0配置
    ADS_CONV_STAS_IA_0_WAIT,        // 本安转换通道0等待转换结果
    ADS_CONV_STAS_IA_1_CFG,         // 本安转换通道1配置
    ADS_CONV_STAS_IA_1_WAIT,        // 本安转换通道1等待转换结果
    ADS_CONV_STAS_IA_2_CFG,         // 本安转换通道2配置
    ADS_CONV_STAS_IA_2_WAIT,        // 本安转换通道2等待转换结果
    ADS_CONV_STAS_IA_3_CFG,         // 本安转换通道3配置
    ADS_CONV_STAS_IA_3_WAIT,        // 本安转换通道3等待转换结果
} ADS_CONV_STAS_IA_E;

#define ADS_CONV_WAIT_ON_PERIOD     (20)  // ADS1120通道转换等待超时 200ms

/* ===== External Variables ===== */
extern unsigned char Ads_stav;
extern struct VariableTemp Variable_Temp;
extern unsigned int currentPresure;
extern struct PARA para;
extern unsigned int currentPresure1;
extern unsigned int currentPresure2;

/* ===== Function Declarations ===== */
// GPIO Control Functions
void ADS1120_SetCS(uint8_t level);
void ADS1120_SetCLK(uint8_t level);
void ADS1120_SetMOSI(uint8_t level);
uint8_t ADS1120_GetMISO(void);
void ADS1120_SetDRDY(uint8_t level);
uint8_t ADS1120_GetDRDY(void);
float DRVMGR_ADCGetValue(uint8_t adcChx);
// ADC Control Functions
void ADS1120_Init(void);
void ADS1120_Restart(void);
void ADS1120_Handle(void);
void ADS1120_Gpio_Init(void);
void ADS1120_Set_Finished(unsigned char flag);

// ADC Register Operations
void adc_set_reg(uint8_t reg, uint8_t value);
void adc_set_regs(uint8_t start_reg, uint8_t number_reg, uint8_t values[]);
void adc_reset(void);
void adc_start(void);

#endif /* DRVMANAGER_DRVMANAGER_ADS1120_H_ */
