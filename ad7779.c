#include <stm32f7xx_hal.h>

#include "ad7779.h"

SPI_HandleTypeDef ad7779_spi;

#define RESET_7779() do { GPIOD->BSRR = GPIO_PIN_15 << 16; } while(0)
#define UNRESET_7779() do { GPIOD->BSRR = GPIO_PIN_15; } while(0)
#define SPI_ENABLE_7779() do { GPIOD->BSRR = GPIO_PIN_14 << 16; } while(0)
#define SPI_DISABLE_7779() do { GPIOD->BSRR = GPIO_PIN_14; } while(0)

static void AD7779_GPIO_Interrupt_Initialize(void)
{
	// Enable the GPIO clock
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOF_CLK_ENABLE();

	// Configure the GPIO for SPI peripheral
	GPIO_InitTypeDef ad7779_gpio;
	ad7779_gpio.Alternate = GPIO_AF5_SPI1;
	ad7779_gpio.Mode = GPIO_MODE_AF_PP;
	ad7779_gpio.Pin = GPIO_PIN_5 | GPIO_PIN_6;
	ad7779_gpio.Pull = GPIO_NOPULL;
	ad7779_gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

	HAL_GPIO_Init(GPIOA, &ad7779_gpio);

	ad7779_gpio.Pin = GPIO_PIN_5;
	HAL_GPIO_Init(GPIOB, &ad7779_gpio);

	// Configure the GPIO for SPI chip-select and chip-RESET
	ad7779_gpio.Mode = GPIO_MODE_OUTPUT_PP;
	ad7779_gpio.Pin = GPIO_PIN_14 | GPIO_PIN_15;
	ad7779_gpio.Pull = GPIO_PULLUP;
	ad7779_gpio.Speed = GPIO_SPEED_FREQ_LOW;

	HAL_GPIO_Init(GPIOD, &ad7779_gpio);

	// Do not reset/select the chip when no operations
	UNRESET_7779();
	SPI_DISABLE_7779();

	// Configure a GPIO for external interrupt input for /DRDY in
	ad7779_gpio.Mode = GPIO_MODE_IT_FALLING;
	ad7779_gpio.Pin = GPIO_PIN_12;
	ad7779_gpio.Pull = GPIO_NOPULL;
	ad7779_gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

	HAL_GPIO_Init(GPIOF, &ad7779_gpio); 

	// Set the /DRDY interrupt priority (most important when use OS)
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, AD7779_DRDY_INT_MAIN_PRIO, AD7779_DRDY_INT_SUB_PRIO);
}

static void AD7779_SPI_Initialize(void)
{
	// Enable the SPI clock
	__HAL_RCC_SPI1_CLK_ENABLE();
	
	// Configure the SPI parameters
	ad7779_spi.Instance = SPI1;
	ad7779_spi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
	ad7779_spi.Init.Mode = SPI_MODE_MASTER;
	ad7779_spi.Init.Direction = SPI_DIRECTION_2LINES;
	ad7779_spi.Init.FirstBit = SPI_FIRSTBIT_MSB;
	ad7779_spi.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	ad7779_spi.Init.CRCLength = 8;
	ad7779_spi.Init.CRCPolynomial = 7;
	ad7779_spi.Init.CLKPolarity = SPI_POLARITY_LOW;
	ad7779_spi.Init.CLKPhase = SPI_PHASE_1EDGE;
	ad7779_spi.Init.DataSize = SPI_DATASIZE_16BIT;
	ad7779_spi.Init.TIMode = SPI_TIMODE_DISABLE;
	ad7779_spi.Init.NSS = SPI_NSS_SOFT;

	// Initialize the SPI peripheral
	HAL_SPI_Init(&ad7779_spi);

	// Enable the SPI peripheral
	__HAL_SPI_ENABLE(&ad7779_spi);
}

static uint16_t AD7779_ReadWrite(uint16_t write)
{
	while ((SPI1->SR & SPI_SR_TXE) == 0x00);
	SPI1->DR = write;
	while ((SPI1->SR & SPI_SR_RXNE) == 0x00);
	return SPI1->DR;
}

void AD7779_Interface_Initialize(void)
{
	// Configure the interfaces
	AD7779_GPIO_Interrupt_Initialize();
	AD7779_SPI_Initialize();

	// Hardware reset the chip to reset all registers
	AD7779_HardwareReset();

	//Read register GEN_ERR_REG_2 to clear RESET bit
	if (AD7779_FLAGCHECK(AD7779_ReadRegister(AD7779_REG_GEN_ERR_REG_2), AD7779_GEN2_ERR_RESET_DETECTED));

	// Set the default SPI mode to register operation mode
	AD7779_Set_SPIOperationMode(AD7779_INT_REG);
}

void AD7779_HardwareReset(void)
{
	RESET_7779();
	HAL_Delay(20);
	UNRESET_7779();
	HAL_Delay(20);
}

uint8_t AD7779_ReadRegister(uint16_t address)
{
	SPI_ENABLE_7779();
	uint8_t data = (AD7779_ReadWrite((address << 8) | 0x8000) & 0x00FF);
	SPI_DISABLE_7779();
	return data;
}

void AD7779_WriteRegister(uint8_t address, uint8_t data)
{
	SPI_ENABLE_7779();
	AD7779_ReadWrite((address << 8) + data);
	SPI_DISABLE_7779();
}

void AD7779_Collection_Start(void)
{
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

void AD7779_Collection_Stop(void)
{
	HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
}

void AD7779_Set_SPIOperationMode(ad7779_spi_op_mode mode)
{
	uint8_t gen2_data = AD7779_ReadRegister(AD7779_REG_GENERAL_USER_CONFIG_2);
	uint8_t gen3_data = AD7779_ReadRegister(AD7779_REG_GENERAL_USER_CONFIG_3);

	switch (mode)
	{
	case AD7779_INT_REG:
		gen2_data &= ~AD7779_SAR_DIAG_MODE_EN;
		gen3_data &= ~AD7779_SPI_SLAVE_MODE_EN;
		break;
	case AD7779_SD_CONV:
		gen2_data &= ~AD7779_SAR_DIAG_MODE_EN;
		gen3_data |= AD7779_SPI_SLAVE_MODE_EN;
		break;
	case AD7779_SAR_CONV:
		gen2_data |= AD7779_SAR_DIAG_MODE_EN;
		gen3_data &= ~AD7779_SPI_SLAVE_MODE_EN;
		break;
	}

	AD7779_WriteRegister(AD7779_REG_GENERAL_USER_CONFIG_2, gen2_data);
	AD7779_WriteRegister(AD7779_REG_GENERAL_USER_CONFIG_3, gen3_data);
}

void AD7779_Set_PowerMode(ad7779_pwr_mode mode)
{
	uint8_t config = AD7779_ReadRegister(AD7779_REG_GENERAL_USER_CONFIG_1);

	if (mode == AD7779_HIGH_RES) config |= AD7779_MOD_POWERMODE;
	else config &= ~AD7779_MOD_POWERMODE;

	AD7779_WriteRegister(AD7779_REG_GENERAL_USER_CONFIG_1, config);
}

ad7779_pwr_mode AD7779_Get_PowerMode(void)
{
	uint8_t config = AD7779_ReadRegister(AD7779_REG_GENERAL_USER_CONFIG_1);

	if (AD7779_FLAGCHECK(config, AD7779_MOD_POWERMODE)) return AD7779_HIGH_RES;
	else return AD7779_LOW_PWR;
}

void AD7779_Set_OutputRate(double odr_kHz)
{
	uint8_t src_n_lower = 0x00, src_n_upper = 0x00;
	uint8_t src_if_lower = 0x00, src_if_upper = 0x00;

	uint16_t src_n = 0x0000, src_if = 0x0000;
	
	// Get the fmod from mode register
	// High-precision = 2048, Low-power = 512
	double f_mod = 512.0;
	if (AD7779_Get_PowerMode() == AD7779_HIGH_RES) f_mod = 2048.0;

	// Calculate the SRC value
	double n = f_mod / odr_kHz;

	src_n = (uint16_t)n; // Convert to integer for floor
	src_if = (uint16_t)((n - (double)src_n) * 65536.0); // Convert the decimal part

	// Split lower/upper bits
	src_n_lower = src_n & 0x00FF;
	src_n_upper = (src_n & 0xFF00) >> 8;

	src_if_lower = src_if & 0x00FF;
	src_if_upper = (src_if & 0xFF00) >> 8;

	// Write registers
	AD7779_WriteRegister(AD7779_REG_SRC_N_MSB, src_n_upper);
	AD7779_WriteRegister(AD7779_REG_SRC_N_LSB, src_n_lower);
	AD7779_WriteRegister(AD7779_REG_SRC_IF_MSB, src_if_upper);
	AD7779_WriteRegister(AD7779_REG_SRC_IF_LSB, src_if_lower);

	// Update
	AD7779_WriteRegister(AD7779_REG_SRC_UPDATE, 0x01);
	HAL_Delay(1);
	AD7779_WriteRegister(AD7779_REG_SRC_UPDATE, 0x00);
}

void AD7779_Set_OutputFormat(ad7779_dout_format format)
{
	uint8_t config = AD7779_ReadRegister(AD7779_REG_DOUT_FORMAT);

	config &= ~AD7779_DOUT_FORMAT(0x03);
	config |= AD7779_DOUT_FORMAT(format);

	AD7779_WriteRegister(AD7779_REG_DOUT_FORMAT, config);
}

void AD7779_Set_OutputHeaderFormat(ad7779_dout_header_format format)
{
	uint8_t config = AD7779_ReadRegister(AD7779_REG_DOUT_FORMAT);

	if (format == AD7779_HEADER_STATUS) config &= ~AD7779_DOUT_HEADER_FORMAT;
	else config |= AD7779_DOUT_HEADER_FORMAT;

	AD7779_WriteRegister(AD7779_REG_DOUT_FORMAT, config);
}

void AD7779_Set_DCLKDivision(ad7779_dclk_div div)
{
	uint8_t config = AD7779_ReadRegister(AD7779_REG_DOUT_FORMAT);

	config &= ~AD7779_DCLK_CLK_DIV(0x07);
	config |= AD7779_DCLK_CLK_DIV(div);

	AD7779_WriteRegister(AD7779_REG_DOUT_FORMAT, config);
}

void AD7779_Set_SigmaDelta_ReferenceMultiplexing(ad7779_ref_mux mux)
{
	uint8_t config = AD7779_ReadRegister(AD7779_REG_ADC_MUX_CONFIG);

	config &= ~AD7779_REF_MUX_CTRL(0x03);
	config |= AD7779_REF_MUX_CTRL(mux);

	AD7779_WriteRegister(AD7779_REG_ADC_MUX_CONFIG, config);
}

void AD7779_Set_SigmaDelta_ADCMultiplexing(ad7779_mtr_mux mux)
{
	uint8_t config = AD7779_ReadRegister(AD7779_REG_ADC_MUX_CONFIG);

	config &= ~AD7779_MTR_MUX_CTRL(0x0F);
	config |= AD7779_MTR_MUX_CTRL(mux);

	AD7779_WriteRegister(AD7779_REG_ADC_MUX_CONFIG, config);
}

void AD7779_Set_SAR_Multiplexing(ad7779_sar_mux mux)
{
	uint8_t config = AD7779_ReadRegister(AD7779_REG_GLOBAL_MUX_CONFIG);

	config &= ~GLOBAL_MUX_CTRL(0x1F);
	config |= GLOBAL_MUX_CTRL(mux);

	AD7779_WriteRegister(AD7779_REG_GLOBAL_MUX_CONFIG, config);
}

void AD7779_GPIO_SetMode(uint8_t gpio_pin, uint8_t gpio_mode)
{
	uint8_t config = AD7779_ReadRegister(AD7779_REG_GPIO_CONFIG);

	if (gpio_mode == GPIO_MODE_OUTPUT_PP) config |= gpio_pin;
	else if (gpio_mode == GPIO_MODE_INPUT) config &= ~gpio_pin;

	AD7779_WriteRegister(AD7779_REG_GPIO_CONFIG, config);
}

void AD7779_GPIO_WritePin(uint8_t gpio_pin, GPIO_PinState state)
{
	uint8_t config = AD7779_ReadRegister(AD7779_REG_GPIO_DATA);

	if (state == GPIO_PIN_SET) config |= gpio_pin;
	else config &= ~gpio_pin;

	AD7779_WriteRegister(AD7779_REG_GPIO_CONFIG, config);
}

GPIO_PinState AD7779_GPIO_ReadPin(uint8_t gpio_pin)
{
	uint8_t config = AD7779_ReadRegister(AD7779_REG_GPIO_CONFIG);

	if (AD7779_FLAGCHECK(config, gpio_pin << 3)) return GPIO_PIN_SET;
	else return GPIO_PIN_RESET;
}

uint16_t AD7779_Get_GeneralError(void)
{
	uint8_t gen1 = AD7779_ReadRegister(AD7779_REG_GEN_ERR_REG_1);
	uint8_t gen2 = AD7779_ReadRegister(AD7779_REG_GEN_ERR_REG_2);

	return ((uint16_t)gen2 << 8) + gen1;
}

void AD7779_Set_GeneralErrorCheckEnable(uint16_t error_en, ad7779_ldo_psm_test_en psm_en, ad7779_ldo_psm_trip_test_en psm_trip_en)
{
	uint8_t gen1 = error_en & 0x00FF;
	uint8_t gen2 = (error_en & 0x2000) >> 8;

	gen2 |= AD7779_LDO_PSM_TEST_EN(psm_en);
	gen2 |= AD7779_LDO_PSM_TRIP_TEST_EN(psm_trip_en);

	AD7779_WriteRegister(AD7779_REG_GEN_ERR_REG_1, gen1);
	AD7779_WriteRegister(AD7779_REG_GEN_ERR_REG_2, gen2);
}

void AD7779_EnableChannel(ad7779_ch channel)
{
	uint8_t ch_disable = AD7779_ReadRegister(AD7779_REG_CH_DISABLE);

	ch_disable &= ~AD7779_CH_DISABLE(channel);

	AD7779_WriteRegister(AD7779_REG_CH_DISABLE, ch_disable);
}

void AD7779_DisableChannel(ad7779_ch channel)
{
	uint8_t ch_disable = AD7779_ReadRegister(AD7779_REG_CH_DISABLE);

	ch_disable |= AD7779_CH_DISABLE(channel);

	AD7779_WriteRegister(AD7779_REG_CH_DISABLE, ch_disable);
}

void AD7779_Set_ChannelEnable(ad7779_ch channel, ad7779_state state)
{
	if (state == AD7779_ENABLE) AD7779_EnableChannel(channel);
	else AD7779_DisableChannel(channel);
}

void AD7779_Set_ChannelPGAGain(ad7779_ch channel, ad7779_gain gain)
{
	uint8_t config = AD7779_ReadRegister(AD7779_REG_CH_CONFIG(channel));

	config &= ~AD7779_CH_GAIN(0x03);
	config |= AD7779_CH_GAIN(gain);

	AD7779_WriteRegister(AD7779_REG_CH_CONFIG(channel), config);
}

void AD7779_Set_ChannelReferenceMonitor(ad7779_ch channel, ad7779_state state)
{
	uint8_t config = AD7779_ReadRegister(AD7779_REG_CH_CONFIG(channel));

	if (state == AD7779_ENABLE) config |= AD7779_CH_REF_MONITOR;
	else config &= ~AD7779_CH_REF_MONITOR;

	AD7779_WriteRegister(AD7779_REG_CH_CONFIG(channel), config);
}

void AD7779_Set_ChannelMuxRxMode(ad7779_ch channel, ad7779_state state)
{
	uint8_t config = AD7779_ReadRegister(AD7779_REG_CH_CONFIG(channel));

	if (state == AD7779_ENABLE) config |= AD7779_CH_RX;
	else config &= ~AD7779_CH_RX;

	AD7779_WriteRegister(AD7779_REG_CH_CONFIG(channel), config);
}

void AD7779_Set_ChannelSyncOffset(ad7779_ch channel, uint8_t offset)
{
	AD7779_WriteRegister(AD7779_REG_CH_SYNC_OFFSET(channel), offset);
}

void AD7779_Set_ChannelOffset(ad7779_ch channel, uint32_t offset)
{
	uint8_t lower, mid, upper;
	lower = offset & 0x0000FF;
	mid = (offset & 0x00FF00) >> 8;
	upper = (offset & 0xFF0000) >> 16;

	AD7779_WriteRegister(AD7779_REG_CH_OFFSET_UPPER_BYTE(channel), upper);
	AD7779_WriteRegister(AD7779_REG_CH_OFFSET_MID_BYTE(channel), mid);
	AD7779_WriteRegister(AD7779_REG_CH_OFFSET_LOWER_BYTE(channel), lower);
}

void AD7779_Set_ChannelGain(ad7779_ch channel, uint32_t gain)
{
	uint8_t lower, mid, upper;
	lower = gain & 0x0000FF;
	mid = (gain & 0x00FF00) >> 8;
	upper = (gain & 0xFF0000) >> 16;

	AD7779_WriteRegister(AD7779_REG_CH_GAIN_UPPER_BYTE(channel), upper);
	AD7779_WriteRegister(AD7779_REG_CH_GAIN_MID_BYTE(channel), mid);
	AD7779_WriteRegister(AD7779_REG_CH_GAIN_LOWER_BYTE(channel), lower);
}

uint8_t AD7779_Get_ChannelError(ad7779_ch channel)
{
	return AD7779_ReadRegister(AD7779_REG_CH_ERR_REG(channel));
}

uint8_t AD7779_Get_ChannelDSPError(ad7779_ch channel)
{
	switch (channel)
	{
	case 0:
	case 1:
		return AD7779_ReadRegister(AD7779_REG_CH0_1_SAT_ERR);
		break;
	case 2:
	case 3:
		return AD7779_ReadRegister(AD7779_REG_CH2_3_SAT_ERR);
		break;
	case 4:
	case 5:
		return AD7779_ReadRegister(AD7779_REG_CH4_5_SAT_ERR);
		break;
	case 6:
	case 7:
		return AD7779_ReadRegister(AD7779_REG_CH6_7_SAT_ERR);
		break;
	}
}

void AD7779_Set_ChannelErrorCheckEnable(uint8_t error_en)
{
	AD7779_WriteRegister(AD7779_REG_CHX_ERR_REG_EN, error_en);
}

void AD7779_Get_SigmaDelta_Value(int32_t * values)
{
	uint16_t data[16];

	SPI_ENABLE_7779();
	for (uint8_t i = 0; i < 16; i++) data[i] = AD7779_ReadWrite(0x8000);
	SPI_DISABLE_7779();

	for (uint8_t ch = 0; ch < 8; ch++)
	{
		uint8_t pointer = ch << 1;
		values[ch] = (((int32_t)data[pointer] & 0x00FF) << 16) + (data[pointer + 1]);
		if ((data[pointer] & 0x0080) > 0) values[ch] | 0xFF000000;
	}
}

void AD7779_Get_SigmaDelta_Original(uint32_t * datas)
{
	uint16_t data[16];

	SPI_ENABLE_7779();
	for (uint8_t i = 0; i < 16; i++) data[i] = AD7779_ReadWrite(0x8000);
	SPI_DISABLE_7779();

	for (uint8_t ch = 0; ch < 8; ch++)
	{
		uint8_t pointer = ch << 1;
		datas[ch] = (data[pointer] << 16) + data[pointer + 1];
	}
}

