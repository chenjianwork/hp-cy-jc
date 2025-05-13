#include "stm32f4xx.h"                 // STM32 标准库主头文件
#include "stm32f4xx_spi.h"               // SPI 外设驱动
#include "stm32f4xx_gpio.h"              // GPIO 外设驱动
#include "stm32f4xx_rcc.h"               // 时钟管理
#include "drvmanager.h"
#include "stm32f4xx_flash.h"             // Flash 操作
#include "stm32f4xx_syscfg.h"
#include "drvmanager-ads1120.h"
#include "stm32f4xx_exti.h"
#include "misc.h"
#include <string.h>      // memcpy, memset
#include "drvmanager-dac8552.h"

/* ===== Global Variables ===== */
uint8_t adc_rx_data[ADC_BUFFER_SIZE];
uint8_t adc_tx_data[ADC_BUFFER_SIZE];
ADS_CONV_STAS_IA_E ads_conv_status = ADS_CONV_STAS_IA_0_CFG;

unsigned char Ads_stav = 0;           // 转换完成标志
double Ads_vol_IA[4] = {0};          // 根据采样值计算出的电压([0]-J1 [1]-J2 [2]-J3 [3]-J4)-本安

uint16_t Ads_sample[8];
uint16_t Ads_sample_IA[4];
uint16_t Ads_sample_D[4];
uint16_t Ads_sample_11;

// 全局数据存储
double ADS1120_Raw[4];               // 当前采样值
double ADS1120_V[4];                 // 当前采样值

// 滤波相关定义
#define FILTER_LEN 16  // 滤波点数

#define REF_VOL		  (2500.0f) // 参考电压，单位mV
#define ADC_BITS	  (12)		// ADC位数

// ADC管理器结构体
struct _ADC_MGR {
    bool    Refresh;                // 数据更新标志
    float   Value;                  // 滤波后的电压值
    size_t  WrIndex;               // 写入索引
    size_t  SampleCount;           // 采样次数
    uint16_t LastValues[FILTER_LEN]; // 历史采样值
};

// 每个通道的ADC管理器
static struct _ADC_MGR adc_mgr[4];

/* ===== GPIO Control Functions ===== */
void ADS1120_SetCS(uint8_t level)
{
    if(level)
        GPIO_SetBits(SPI_CS_PORT, SPI_CS_ADC_PIN);
    else
        GPIO_ResetBits(SPI_CS_PORT, SPI_CS_ADC_PIN);
}

void ADS1120_SetCLK(uint8_t level)
{
    if(level)
        GPIO_SetBits(SPI_SCK_PORT, SPI_SCK_PIN);
    else
        GPIO_ResetBits(SPI_SCK_PORT, SPI_SCK_PIN);
}

void ADS1120_SetMOSI(uint8_t level)
{
    if(level)
        GPIO_SetBits(SPI_MOSI_PORT, SPI_MOSI_PIN);
    else
        GPIO_ResetBits(SPI_MOSI_PORT, SPI_MOSI_PIN);
}

uint8_t ADS1120_GetMISO(void)
{
    return GPIO_ReadInputDataBit(SPI_MISO_PORT, SPI_MISO_PIN);
}

void ADS1120_SetDRDY(uint8_t level)
{
    if(level)
        GPIO_SetBits(SPI_DRDY_PORT, SPI_DRDY_PIN);
    else
        GPIO_ResetBits(SPI_DRDY_PORT, SPI_DRDY_PIN);
}

uint8_t ADS1120_GetDRDY(void)
{
    return GPIO_ReadInputDataBit(SPI_DRDY_PORT, SPI_DRDY_PIN);
}

/* ===== Function Prototypes ===== */
static void adc_set_channel(unsigned char channel);

static void SPI_Write(unsigned char *outData, unsigned char length);
static void SPI_WriteRead(unsigned char *outData, unsigned char *inData, unsigned char length);
 void adc_buffer_clear(uint8_t buffer[]);
static uint16_t adc_calculate_voltage(uint8_t msb, uint8_t lsb, unsigned char channel);
static void adc_get_sample(void);
//static double ADC_Cal_Val(double range, double *mV);

/* ===== SPI Communication Functions ===== */
// 设置CS（片选）电平
void SPI_SetCS(uint8_t device, uint8_t level)
{
    // 切换前先全部拉高，避免两个CS同时为低
    GPIO_SetBits(SPI_CS_PORT, SPI_CS_ADC_PIN);
    GPIO_SetBits(SPI_CS_PORT, SPI_CS_DAC_PIN);
    DRVMGR_TimerDelayUs(1); // 切换建立时间
    if(device == DEVICE_ADC) // ADS1120
    {
        if(level == 0)
            GPIO_ResetBits(SPI_CS_PORT, SPI_CS_ADC_PIN);
    }
    else // DAC8552
    {
        if(level == 0)
            GPIO_ResetBits(SPI_CS_PORT, SPI_CS_DAC_PIN);
    }
    DRVMGR_TimerDelayUs(1); // 切换后建立时间
}

/* CPOL=0，CPHA=1, MSB first 模式1 */
uint8_t Ads1120_SPI_RW(uint8_t byte)
{
    uint8_t i, temp=0;
    __disable_irq();
    DRVMGR_TimerDelayUs(1);
    for(i=0; i<8; i++)     // 循环8次
    {
        ADS1120_SetCLK(1);     //拉高时钟
        if(byte & 0x80)
        {
            ADS1120_SetMOSI(1);  //若最到位为高，则输出高
        }
        else
        {
            ADS1120_SetMOSI(0);   //若最到位为低，则输出低
        }
        byte <<= 1;     // 低一位移位到最高位
        DRVMGR_TimerDelayUs(1);
        ADS1120_SetCLK(0);     //拉低时钟
        temp <<= 1;     //数据左移

        if(ADS1120_GetMISO())
            temp++;     //若从从机接收到高电平，数据自加一
        DRVMGR_TimerDelayUs(1);
    }
    DRVMGR_TimerDelayUs(1);
    __enable_irq(); // 退出临界区
    return (temp);     //返回数据
}

static void SPI_Write(unsigned char *outData, unsigned char length)
{
    unsigned char i;
    __disable_irq(); // 进入临界区，禁止中断
    // ---【切换到 DAC】确保 ADS1120 的 CS 拉高并等待空档----
    GPIO_SetBits(SPI_CS_PORT, SPI_CS_ADC_PIN);      // ADS1120 CS = 1 (释放)
    DRVMGR_TimerDelayUs(20);                 // 延时 ≥100ns，满足 t(CSSC) & t8

    // 拉低 DAC8552 CS，开始 SPI 事务
    ADS1120_SetCS(0);    // DAC_CS = 0 (选中)
    DRVMGR_TimerDelayUs(20);

    for (i=0; i<length; i++)
    {
        Ads1120_SPI_RW(outData[i]);
        DRVMGR_TimerDelayUs(20);
    }
    ADS1120_SetCS(1);      // ADS1120 CS = 1 (释放)
    DRVMGR_TimerDelayUs(20);                 // 延时 ≥100ns，满足 t(CSSC) & t8
    __enable_irq(); // 退出临界区
}

static void SPI_WriteRead(unsigned char *outData, unsigned char *inData, unsigned char length)
{
    unsigned char i;
    __disable_irq(); // 进入临界区，禁止中断

    GPIO_SetBits(SPI_CS_PORT, SPI_CS_ADC_PIN);      // ADS1120 CS = 1 (释放)
    DRVMGR_TimerDelayUs(20);                 // 延时 ≥100ns，满足 t(CSSC) & t8

    // 拉低 DAC8552 CS，开始 SPI 事务
    ADS1120_SetCS(0);    // DAC_CS = 0 (选中)
    DRVMGR_TimerDelayUs(20);

    for (i=0; i<length; i++)
    {
        inData[i] = Ads1120_SPI_RW(*(outData+i));
    }

    ADS1120_SetCS(1);      // ADS1120 CS = 1 (释放)
    DRVMGR_TimerDelayUs(20);                 // 延时 ≥100ns，满足 t(CSSC) & t8
    __enable_irq(); // 退出临界区
}

/* ===== ADC Register Operations ===== */
void adc_set_reg(uint8_t reg, uint8_t value)
{
    uint8_t config[2];
    __disable_irq(); // 进入临界区，禁止中断
    config[0] = ADC_CMD_WREG | (reg << 2); // nn + 1
    config[1] = value;

    ADS1120_SetCS(0);
    SPI_Write(config, 2);
    ADS1120_SetCS(1);
    __enable_irq(); // 退出临界区
}

void adc_set_regs(uint8_t start_reg, uint8_t number_reg, uint8_t values[])
{
    uint8_t config[5];
    uint8_t i;
    __disable_irq(); // 进入临界区，禁止中断
    config[0] = ADC_CMD_WREG | (start_reg << 2) | (number_reg - 1); // nn + 1

    for (i = 0; i < number_reg; i++)
    {
        config[i + 1] = values[i];
    }

    ADS1120_SetCS(0);
    SPI_Write(config, number_reg + 1);
    ADS1120_SetCS(1);
    __enable_irq(); // 退出临界区
}

void adc_reset(void)
{
    uint8_t config[] = { ADC_CMD_RESET };
    __disable_irq(); // 进入临界区，禁止中断
    delay_ms(1);
    NVIC_EnableIRQ(EXTI9_5_IRQn);  // disable interruption

    ADS1120_SetCS(0);
    SPI_Write(config, 1);
    ADS1120_SetCS(1);
    __enable_irq(); // 退出临界区
}

void adc_start(void)
{
    uint8_t config[] = { ADC_CMD_START };
    __disable_irq(); // 进入临界区，禁止中断
    ADS1120_SetCS(0);
    SPI_Write(config, 1);
    ADS1120_SetCS(1);
    delay_ms(1);
    NVIC_EnableIRQ(EXTI9_5_IRQn);  // enable interruption
    __enable_irq(); // 退出临界区
}

void adc_buffer_clear(uint8_t buffer[])
{
    uint8_t i;
    for (i = 0; i < ADC_BUFFER_SIZE; i++)
        buffer[i] = 0;
}

/* ===== ADC Channel Configuration ===== */
static void adc_set_channel(unsigned char channel)
{
    switch (channel)
    {
        case 0:
            adc_set_reg(ADC_REG0, ADC_REG0_MUX_AIN0_AVSS | ADC_REG0_GAIN1 | ADC_REG0_PGA_BYPASS_DISABLE);
            break;
        case 1:
            adc_set_reg(ADC_REG0, ADC_REG0_MUX_AIN1_AVSS | ADC_REG0_GAIN1 | ADC_REG0_PGA_BYPASS_DISABLE);
            break;
        case 2:
            adc_set_reg(ADC_REG0, ADC_REG0_MUX_AIN2_AVSS | ADC_REG0_GAIN1 | ADC_REG0_PGA_BYPASS_DISABLE);
            break;
        case 3:
            adc_set_reg(ADC_REG0, ADC_REG0_MUX_AIN3_AVSS | ADC_REG0_GAIN1 | ADC_REG0_PGA_BYPASS_DISABLE);
            break;
    }
}

/* ===== ADC Data Processing ===== */
static uint16_t adc_calculate_voltage(uint8_t msb, uint8_t lsb, unsigned char channel)
{
    // 1. 参数有效性检查
    if (channel >= 4) {
        return 0;
    }

    // 2. 计算原始采样值
    uint16_t sample = ((uint16_t)msb << 8) | lsb;
    
    // 3. 异常值处理
    if (sample >= ADC_PRECISION) {
        sample = 0;
    }

    // 4. 更新采样计数（决定是否启用滑动滤波器）
    adc_mgr[channel].SampleCount++;
    if (adc_mgr[channel].SampleCount >= FILTER_LEN) {
        adc_mgr[channel].Refresh = true;
        adc_mgr[channel].SampleCount = FILTER_LEN;
    }

    // 5. 更新采样值
    adc_mgr[channel].LastValues[adc_mgr[channel].WrIndex++] = sample;
    if (adc_mgr[channel].WrIndex >= FILTER_LEN) {
        adc_mgr[channel].WrIndex = 0;
    }

    // 6. 计算滤波后的值
    uint32_t sum = 0;
    for (size_t i = 0; i < adc_mgr[channel].SampleCount; i++) {
        sum += adc_mgr[channel].LastValues[i];
    }
    sample = (uint16_t)(sum / adc_mgr[channel].SampleCount);

    return sample;
}

static void adc_get_sample(void)
{
    ADS1120_SetCS(0);
    SPI_WriteRead(adc_tx_data, adc_rx_data, 2);
    ADS1120_SetCS(1);
}
#if 0
/* ===== ADC Calibration ===== */
static double ADC_Cal_Val(double range, double *mV)
{
    double tmp_mA;
    static double ret_value;
    tmp_mA = *mV / ADS1120_REF_R_IA * 1000.0;

    ret_value = ((tmp_mA - AD_MIN_mA) * range /(AD_MAX_mA - AD_MIN_mA));
    if (ret_value <= 0.0)
        ret_value = 0.0;
    return ret_value;
}
#endif

/* ===== Hardware Initialization ===== */
void ADS1120_Gpio_Init(void)
{
    GPIO_InitTypeDef  gpio;

    // 时钟：只要 GPIOC/B
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB|RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

    // SCK (PB10) 推挽输出
    gpio.GPIO_Pin   = SPI_SCK_PIN;
    gpio.GPIO_Mode  = GPIO_Mode_OUT;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_Init(SPI_SCK_PORT, &gpio);
    ADS1120_SetCLK(0);  // 默认低

    // MOSI (PC3) 推挽输出
    gpio.GPIO_Pin = SPI_MOSI_PIN;
    GPIO_Init(SPI_MOSI_PORT, &gpio);
    ADS1120_SetMOSI(0); // 默认低

    // MISO (PC2) 上拉输入
    gpio.GPIO_Pin  = SPI_MISO_PIN;
    gpio.GPIO_Mode = GPIO_Mode_IN;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(SPI_MISO_PORT, &gpio);

    // CS (PC1) 推挽 + 上拉，默认高
    gpio.GPIO_Pin   = SPI_CS_ADC_PIN;
    gpio.GPIO_Mode  = GPIO_Mode_OUT;
    gpio.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(SPI_CS_PORT, &gpio);
    ADS1120_SetCS(1);

    // DRDY (PC5) 下拉输入 + EXTI
    gpio.GPIO_Pin  = SPI_DRDY_PIN;
    gpio.GPIO_Mode = GPIO_Mode_IN;
    gpio.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(SPI_DRDY_PORT, &gpio);

    // 关联 EXTI5 → PC5
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource5);
    EXTI_InitTypeDef exti = {
        .EXTI_Line    = EXTI_Line5,
        .EXTI_Mode    = EXTI_Mode_Interrupt,
        .EXTI_Trigger = EXTI_Trigger_Falling,
        .EXTI_LineCmd = ENABLE
    };
    EXTI_Init(&exti);

    NVIC_InitTypeDef nvic = {
        .NVIC_IRQChannel            = EXTI9_5_IRQn,
        .NVIC_IRQChannelPreemptionPriority = 3,
        .NVIC_IRQChannelSubPriority = 3,
        .NVIC_IRQChannelCmd         = ENABLE
    };
    NVIC_Init(&nvic);
}

/* ===== Interrupt Handlers ===== */
void EXTI9_5_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line5)) {
        EXTI_ClearITPendingBit(EXTI_Line5);
        Ads_stav = true;
    }
}

/* ===== Main ADC Functions ===== */
void ADS1120_Init(void)
{
    ADS1120_Gpio_Init();

    // 初始化ADC管理器
    for (int i = 0; i < 4; i++) {
        adc_mgr[i].Refresh = false;
        adc_mgr[i].Value = 0.0f;
        adc_mgr[i].WrIndex = 0;
        adc_mgr[i].SampleCount = 0;
        memset(adc_mgr[i].LastValues, 0, sizeof(adc_mgr[i].LastValues));
    }

    adc_buffer_clear(adc_rx_data);
    adc_buffer_clear(adc_tx_data);
    memset(ADS1120_Raw, 0, sizeof(ADS1120_Raw));
    memset(ADS1120_V, 0, sizeof(ADS1120_V));
    memset(ADC1120Data, 0, sizeof(ADC1120Data));
}

void ADS1120_Restart(void)
{
    //setting registers
    uint8_t config[] = {
        ADC_REG0_MUX_AIN0_AVSS | ADC_REG0_GAIN1 | ADC_REG0_PGA_BYPASS_DISABLE,
        ADC_REG1_DR_NORM_MODE_90SPS | ADC_REG1_MODE_NORMAL | ADC_REG1_CM_CONTINUOUS | ADC_REG1_TS_DISABLE | ADC_REG1_BCS_OFF,
        ADC_REG2_VREF_EXTERNAL_REFP0_REFN0 | ADC_REG2_FIR_NO | ADC_REG2_PSW_OPEN | ADC_REG2_IDAC_1000u,
        ADC_REG3_I1MUX_DISABLED| ADC_REG3_I2MUX_DISABLED | ADC_REG3_DRDYM_ON | ADC_REG3_RESERVED
    };
    adc_buffer_clear(adc_rx_data);
    adc_buffer_clear(adc_tx_data);

    adc_reset();
    delay_ms(1);
    adc_set_regs(ADC_REG0, 4, config);
    adc_start();
}

void ADS1120_Handle(void)
{
    //setting registers
    uint8_t config[] = {
        ADC_REG0_MUX_AIN0_AVSS | ADC_REG0_GAIN1 | ADC_REG0_PGA_BYPASS_DISABLE,
        ADC_REG1_DR_NORM_MODE_175SPS | ADC_REG1_MODE_NORMAL | ADC_REG1_CM_CONTINUOUS | ADC_REG1_TS_DISABLE | ADC_REG1_BCS_OFF,
        ADC_REG2_VREF_EXTERNAL_REFP0_REFN0 | ADC_REG2_FIR_NO | ADC_REG2_PSW_OPEN | ADC_REG2_IDAC_1000u,
        ADC_REG3_I1MUX_DISABLED| ADC_REG3_I2MUX_DISABLED | ADC_REG3_DRDYM_ON | ADC_REG3_RESERVED
    };

    switch (ads_conv_status)
    {
        case ADS_CONV_STAS_IA_0_CFG:
            adc_reset();
            delay_ms(1);
            adc_set_regs(ADC_REG0, 4, config);//设置通道0
            adc_start();
            ads_conv_status = ADS_CONV_STAS_IA_0_WAIT;
            DRVMGR_MSTimerStart(MSEC_TMR_ID_ADS_WAIT, ADS_CONV_WAIT_ON_PERIOD);
            break;

        case ADS_CONV_STAS_IA_0_WAIT:
            if (Ads_stav)
            {//收到转换结果
                Ads_stav = 0;
                adc_buffer_clear(adc_rx_data);
                adc_get_sample();
                adc_calculate_voltage(adc_rx_data[0], adc_rx_data[1], 0); //处理通道0转换结果
                ads_conv_status = ADS_CONV_STAS_IA_1_CFG;
                DRVMGR_MSTimerCancel(MSEC_TMR_ID_ADS_WAIT);
            }
            else if (DRVMGR_MSTimerIsExpiration(MSEC_TMR_ID_ADS_WAIT))
            {//等待通道转换结果出现超时
                Ads_vol_IA[0] = 0; //转换超时,则数据清零
                ads_conv_status = ADS_CONV_STAS_IA_1_CFG;
                DRVMGR_MSTimerCancel(MSEC_TMR_ID_ADS_WAIT);
            }
            break;

        case ADS_CONV_STAS_IA_1_CFG:
            adc_set_channel(1); //切换通道1
            adc_start();
            ads_conv_status = ADS_CONV_STAS_IA_1_WAIT;
            DRVMGR_MSTimerStart(MSEC_TMR_ID_ADS_WAIT, ADS_CONV_WAIT_ON_PERIOD);
            break;

        case ADS_CONV_STAS_IA_1_WAIT:
            if (Ads_stav)
            {//收到转换结果
                Ads_stav = 0;
                adc_buffer_clear(adc_rx_data);
                adc_get_sample();
                ads_conv_status = ADS_CONV_STAS_IA_2_CFG;
                DRVMGR_MSTimerCancel(MSEC_TMR_ID_ADS_WAIT);
            }
            else if (DRVMGR_MSTimerIsExpiration(MSEC_TMR_ID_ADS_WAIT))
            {//等待通道转换结果出现超时
                Ads_vol_IA[1] = 0; //转换超时,则数据清零
                ads_conv_status = ADS_CONV_STAS_IA_2_CFG;
                DRVMGR_MSTimerCancel(MSEC_TMR_ID_ADS_WAIT);
            }
            break;

        case ADS_CONV_STAS_IA_2_CFG:
            adc_set_channel(2); //切换通道2
            adc_start();
            ads_conv_status = ADS_CONV_STAS_IA_2_WAIT;
            DRVMGR_MSTimerStart(MSEC_TMR_ID_ADS_WAIT, ADS_CONV_WAIT_ON_PERIOD);
            break;

        case ADS_CONV_STAS_IA_2_WAIT:
            if (Ads_stav)
            {//收到转换结果
                Ads_stav = 0;
                adc_buffer_clear(adc_rx_data);
                adc_get_sample();
                adc_calculate_voltage(adc_rx_data[0], adc_rx_data[1], 2);
            //    ADS1120_Gpio_Init();
             //   DAC8552_GPIO_Init();
            //    DRVMGR_TimerDelayUs(10);
             //   DAC8552_WriteChannelB(DAC_CMD_CH_B, temp);
             //   temp = 0;

                ads_conv_status = ADS_CONV_STAS_IA_3_CFG;
                DRVMGR_MSTimerCancel(MSEC_TMR_ID_ADS_WAIT);
            }
            else if (DRVMGR_MSTimerIsExpiration(MSEC_TMR_ID_ADS_WAIT))
            {//等待通道转换结果出现超时
                Ads_vol_IA[2] = 0; //转换超时,则数据清零
                ads_conv_status = ADS_CONV_STAS_IA_3_CFG;
                DRVMGR_MSTimerCancel(MSEC_TMR_ID_ADS_WAIT);
            }
            break;

        case ADS_CONV_STAS_IA_3_CFG: //J4-H70旁通阀控制的联动压力
            adc_set_channel(3); //切换通道3
            adc_start();
            ads_conv_status = ADS_CONV_STAS_IA_3_WAIT;
            DRVMGR_MSTimerStart(MSEC_TMR_ID_ADS_WAIT, ADS_CONV_WAIT_ON_PERIOD);
            break;

        case ADS_CONV_STAS_IA_3_WAIT:
            if (Ads_stav)
            {//收到转换结果
                Ads_stav = 0;
                adc_buffer_clear(adc_rx_data);
                adc_get_sample();
                adc_calculate_voltage(adc_rx_data[0], adc_rx_data[1], 3); //处理通道3转换结果

            //    ADS1120_Gpio_Init();
             //   DAC8552_GPIO_Init();
             //   DRVMGR_TimerDelayUs(10);
             //   DAC8552_WriteChannelA(DAC_CMD_CH_A, Ads_sample_11);
                ads_conv_status = ADS_CONV_STAS_IA_0_WAIT;
                DRVMGR_MSTimerCancel(MSEC_TMR_ID_ADS_WAIT);
            }
            else if (DRVMGR_MSTimerIsExpiration(MSEC_TMR_ID_ADS_WAIT))
            {//等待通道转换结果出现超时
                Ads_vol_IA[3] = 0; //转换超时,则数据清零
                ads_conv_status = ADS_CONV_STAS_IA_0_WAIT;
                DRVMGR_MSTimerCancel(MSEC_TMR_ID_ADS_WAIT);
            }
            break;
    }
}

// 添加获取ADC值的函数
float ADS1120_GetValue(unsigned char channel)
{
    if (channel >= 4) {
        return 0.0f;
    }
    adc_mgr[channel].Refresh = false;  // 读取后清除更新标志
    return adc_mgr[channel].Value;
}

// 添加检查数据是否更新的函数
bool ADS1120_IsDataUpdated(unsigned char channel)
{
    if (channel >= 4) {
        return false;
    }
    return adc_mgr[channel].Refresh;
}

/*!
****************************************************************************************************
* 功能描述：读取指定通道的AD采样值
* 注意事项：NA
* 输入参数：通道号（0-3）
* 输出参数：NA
* 返回参数：此通道的电压值单位mV
****************************************************************************************************
*/
float DRVMGR_ADCGetValue(uint8_t adcChx)
{
    size_t i;
    float sum = 0;
    float sampleValue = 0;
    struct _ADC_MGR* adcMGR;

    // 参数检查
    if (adcChx >= 4) {
        return 0;
    }

    adcMGR = &adc_mgr[adcChx];

    // 计算平均值
    if (adcMGR->SampleCount >= FILTER_LEN) {
        for (i = 0; i < FILTER_LEN; i++) {
            sum += adcMGR->LastValues[i];
        }
        sampleValue = sum / FILTER_LEN;
    }

    // 转换为实际电压值
	sampleValue = sampleValue * REF_VOL;
	sampleValue /= (1 << ADC_BITS);
    
    return sampleValue;
}
