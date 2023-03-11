#pragma once

// AD7779 /DRDY interrupt priority
#define AD7779_DRDY_INT_MAIN_PRIO 9
#define AD7779_DRDY_INT_SUB_PRIO 0

// AD7779 CRC polynomial
#define AD7779_CRC_POLY			0x07

// AD7779 registers
#define AD7779_REG_CH_CONFIG(ch)		(0x00 + (ch))		// Channel Configuration
#define AD7779_REG_CH_DISABLE			0x08			// Disable clocks to ADC channel
#define AD7779_REG_CH_SYNC_OFFSET(ch)		(0x09 + (ch))		// Channel SYNC Offset
#define AD7779_REG_GENERAL_USER_CONFIG_1	0x11			// General User Config 1
#define AD7779_REG_GENERAL_USER_CONFIG_2	0x12			// General User Config 2
#define AD7779_REG_GENERAL_USER_CONFIG_3	0x13			// General User Config 3
#define AD7779_REG_DOUT_FORMAT			0x14			// Data out format
#define AD7779_REG_ADC_MUX_CONFIG		0x15			// Main ADC meter and reference Mux control
#define AD7779_REG_GLOBAL_MUX_CONFIG		0x16			// Global diagnostics mux
#define AD7779_REG_GPIO_CONFIG			0x17			// GPIO config
#define AD7779_REG_GPIO_DATA			0x18			// GPIO Data
#define AD7779_REG_BUFFER_CONFIG_1		0x19			// Buffer Config 1
#define AD7779_REG_BUFFER_CONFIG_2		0x1A			// Buffer Config 2
#define AD7779_REG_CH_OFFSET_UPPER_BYTE(ch)	(0x1C + (ch) * 6)	// Channel offset upper byte
#define AD7779_REG_CH_OFFSET_MID_BYTE(ch)	(0x1D + (ch) * 6)	// Channel offset middle byte
#define AD7779_REG_CH_OFFSET_LOWER_BYTE(ch)	(0x1E + (ch) * 6)	// Channel offset lower byte
#define AD7779_REG_CH_GAIN_UPPER_BYTE(ch)	(0x1F + (ch) * 6)	// Channel gain upper byte
#define AD7779_REG_CH_GAIN_MID_BYTE(ch)		(0x20 + (ch) * 6)	// Channel gain middle byte
#define AD7779_REG_CH_GAIN_LOWER_BYTE(ch)	(0x21 + (ch) * 6)	// Channel gain lower byte
#define AD7779_REG_CH_ERR_REG(ch)		(0x4C + (ch))		// Channel Status Register
#define AD7779_REG_CH0_1_SAT_ERR		0x54			// Channel 0/1 DSP errors
#define AD7779_REG_CH2_3_SAT_ERR		0x55			// Channel 2/3 DSP errors
#define AD7779_REG_CH4_5_SAT_ERR		0x56			// Channel 4/5 DSP errors
#define AD7779_REG_CH6_7_SAT_ERR		0x57			// Channel 6/7 DSP errors
#define AD7779_REG_CHX_ERR_REG_EN		0x58			// Channel 0-7 Error Reg Enable
#define AD7779_REG_GEN_ERR_REG_1		0x59			// General Errors Register 1
#define AD7779_REG_GEN_ERR_REG_1_EN		0x5A			// General Errors Register 1 Enable
#define AD7779_REG_GEN_ERR_REG_2		0x5B			// General Errors Register 2
#define AD7779_REG_GEN_ERR_REG_2_EN		0x5C			// General Errors Register 2 Enable
#define AD7779_REG_STATUS_REG_1			0x5D			// Error Status Register 1
#define AD7779_REG_STATUS_REG_2			0x5E			// Error Status Register 2
#define AD7779_REG_STATUS_REG_3			0x5F			// Error Status Register 3
#define AD7779_REG_SRC_N_MSB			0x60			// Decimation Rate (N) MSB
#define AD7779_REG_SRC_N_LSB			0x61			// Decimation Rate (N) LSB
#define AD7779_REG_SRC_IF_MSB			0x62			// Decimation Rate (IF) MSB
#define AD7779_REG_SRC_IF_LSB			0x63			// Decimation Rate (IF) LSB
#define AD7779_REG_SRC_UPDATE			0x64			// SRC load source and load update

// AD7779 registers configurations
// AD7779_REG_CHx_CONFIG
#define AD7779_CH_GAIN(x)			(((x) & 0x3) << 6)
#define AD7779_CH_REF_MONITOR (1 << 5)
#define AD7779_CH_RX				(1 << 4)

// AD7779_REG_CH_DISABLE
#define AD7779_CH_DISABLE(x)			(1 << (x))

// AD7779_REG_GENERAL_USER_CONFIG_1
#define AD7779_ALL_CH_DIS_MCLK_EN		(1 << 7)
#define AD7779_MOD_POWERMODE			(1 << 6)
#define AD7779_PDB_VCM				(1 << 5)
#define AD7779_PDB_REFOUT_BUF			(1 << 4)
#define AD7779_PDB_SAR				(1 << 3)
#define AD7779_PDB_RC_OSC			(1 << 2)
#define AD7779_SOFT_RESET(x)			(((x) & 0x3) << 0)

// AD7779_REG_GENERAL_USER_CONFIG_2
#define AD7779_FILTER_MODE			(1 << 6)
#define AD7779_SAR_DIAG_MODE_EN			(1 << 5)
#define AD7779_SDO_DRIVE_STR(x)			(((x) & 0x3) << 3)
#define AD7779_DOUT_DRIVE_STR(x)		(((x) & 0x3) << 1)
#define AD7779_SPI_SYNC				(1 << 0)

// AD7779_REG_GENERAL_USER_CONFIG_3
#define AD7779_CONVST_SAR_DEBURRING_1_5MCLK (0x02 << 6)
#define AD7779_CONVST_SAR_DEBURRING_NONE (0x03 << 6)
#define AD7779_SPI_SLAVE_MODE_EN		(1 << 4)
#define AD7779_CLK_QUAL_DIS			(1 << 0)

// AD7779_REG_DOUT_FORMAT
#define AD7779_DOUT_FORMAT(x)			(((x) & 0x3) << 6)
#define AD7779_DOUT_HEADER_FORMAT		(1 << 5)
#define AD7779_DCLK_CLK_DIV(x)			(((x) & 0x7) << 1)

// AD7779_REG_ADC_MUX_CONFIG
#define AD7779_REF_MUX_CTRL(x) (((x) & 0x3) << 6)
#define AD7779_MTR_MUX_CTRL(x) (((x) & 0x0F) << 2)

// AD7779_REG_GLOBAL_MUX_CONFIG
#define AD7779_GLOBAL_MUX_CTRL(x)		(((x) & 0x1F) << 3)

// AD7779_REG_BUFFER_CONFIG_1
#define AD7779_REF_BUF_POS_EN			(1 << 4)
#define AD7779_REF_BUF_NEG_EN			(1 << 3)

// AD7779_REG_BUFFER_CONFIG_2
#define AD7779_REFBUFP_PREQ			(1 << 7)
#define AD7779_REFBUFN_PREQ			(1 << 6)
#define AD7779_PDB_ALDO1_OVRDRV			(1 << 2)
#define AD7779_PDB_ALDO2_OVRDRV			(1 << 1)
#define AD7779_PDB_DLDO_OVRDRV			(1 << 0)

// AD7779_REG_CHX_ERR_REG_EN
#define AD7779_OUTPUT_SAT_TEST_EN		(1 << 7)
#define AD7779_FILTER_SAT_TEST_EN				(1 << 6)
#define AD7779_MOD_SAT_TEST_EN		(1 << 5)
#define AD7779_AINM_UV_TEST_EN				(1 << 4)
#define AD7779_AINM_OV_TEST_EN			(1 << 3)
#define AD7779_AINP_UV_TEST_EN			(1 << 2)
#define AD7779_AINP_OV_TEST_EN	(1 << 1)
#define AD7779_REG_DET_TEST_EN			(1 << 0)

// AD7779_REG_GEN_ERR_REG_1_EN
#define AD7779_MEMMAP_CRC_TEST_EN		(1 << 5)
#define AD7779_ROM_CRC_TEST_EN			(1 << 4)
#define AD7779_SPI_CLK_COUNT_TEST_EN		(1 << 3)
#define AD7779_SPI_INVALID_READ_TEST_EN		(1 << 2)
#define AD7779_SPI_INVALID_WRITE_TEST_EN	(1 << 1)
#define AD7779_SPI_CRC_TEST_EN			(1 << 0)

// AD7779_REG_GEN_ERR_REG_2_EN
#define AD7779_RESET_DETECT_TEST_EN		(1 << 5)
#define AD7779_LDO_PSM_TEST_EN(x) (((x) & 0x03) << 2)
#define AD7779_LDO_PSM_TRIP_TEST_EN(x) (((x) & 0x03) << 0)

// AD7779 error flags
// GEN_ERR_REG1
#define AD7779_GEN1_ERR_MEMMAP_CRC_TEST (1 << 5)
#define AD7779_GEN1_ERR_ROM_CRC_TEST (1 << 4)
#define AD7779_GEN1_ERR_SPI_CLK_COUNT_TEST (1 << 3)
#define AD7779_GEN1_ERR_SPI_INVALID_READ_TEST (1 << 2)
#define AD7779_GEN1_ERR_SPI_INVALID_WRITE_TEST (1 << 1)
#define AD7779_GEN1_ERR_SPI_CRC_TEST (1 << 0)

// GEN_ERR_REG2
#define AD7779_GEN2_ERR_RESET_DETECTED (1 << 5)
#define AD7779_GEN2_ERR_EXT_MCLK_SWITCH (1 << 4)
#define AD7779_GEN2_ERR_ALDO1_PSM (1 << 2)
#define AD7779_GEN2_ERR_ALDO2_PSM (1 << 1)
#define AD7779_GEN2_ERR_DLDO_PSM (1 << 0)

// General errors
#define AD7779_GEN_ERR_MEMMAP_CRC_TEST AD7779_GEN1_ERR_MEMMAP_CRC_TEST
#define AD7779_GEN_ERR_ROM_CRC_TEST AD7779_GEN1_ERR_ROM_CRC_TEST
#define AD7779_GEN_ERR_SPI_CLK_COUNT_TEST AD7779_GEN1_ERR_SPI_CLK_COUNT_TEST
#define AD7779_GEN_ERR_SPI_INVALID_READ_TEST AD7779_GEN1_ERR_SPI_INVALID_READ_TEST
#define AD7779_GEN_ERR_SPI_INVALID_WRITE_TEST AD7779_GEN1_ERR_SPI_INVALID_WRITE_TEST
#define AD7779_GEN_ERR_SPI_CRC_TEST AD7779_GEN1_ERR_SPI_CRC_TEST
#define AD7779_GEN_ERR_RESET_DETECTED (AD7779_GEN2_ERR_RESET_DETECTED << 8)
#define AD7779_GEN_ERR_EXT_MCLK_SWITCH (AD7779_GEN2_ERR_EXT_MCLK_SWITCH << 8)
#define AD7779_GEN_ERR_ALDO1_PSM (AD7779_GEN2_ERR_ALDO1_PSM << 8)
#define AD7779_GEN_ERR_ALDO2_PSM (AD7779_GEN2_ERR_ALDO2_PSM << 8)
#define AD7779_GEN_ERR_DLDO_PSM (AD7779_GEN2_ERR_DLDO_PSM << 8)

// Channel errors
#define AD7779_CH_ERR_AINM_UV (1 << 4)
#define AD7779_CH_ERR_AINM_OV (1 << 3)
#define AD7779_CH_ERR_AINP_UV (1 << 2)
#define AD7779_CH_ERR_AINP_OV (1 << 1)
#define AD7779_CH_ERR_REF_DET (1 << 0)

// Channel DSP errors
#define AD7779_CH0246_ERR_DSP_MOD_SAT (1 << 2)
#define AD7779_CH1357_ERR_DSP_MOD_SAT (1 << 5)
#define AD7779_CH0246_ERR_DSP_FILTER_SAT (1 << 1)
#define AD7779_CH1357_ERR_DSP_FILTER_SAT (1 << 4)
#define AD7779_CH0246_ERR_DSP_OUTPUT_SAT (1 << 0)
#define AD7779_CH1357_ERR_DSP_OUTPUT_SAT (1 << 3)

// Flag check tool
#define AD7779_FLAGCHECK(reg_data, flag) (((reg_data) & (flag)) > 0)

// AD7779 configuration enumeration
typedef enum
{
	AD7779_INT_REG,
	AD7779_SD_CONV,
	AD7779_SAR_CONV,
} ad7779_spi_op_mode;

typedef enum
{
	AD7779_CH0,
	AD7779_CH1,
	AD7779_CH2,
	AD7779_CH3,
	AD7779_CH4,
	AD7779_CH5,
	AD7779_CH6,
	AD7779_CH7,
} ad7779_ch;

typedef enum
{
	AD7779_ENABLE,
	AD7779_DISABLE,
} ad7779_state;

typedef enum
{
	AD7779_GAIN_1,
	AD7779_GAIN_2,
	AD7779_GAIN_4,
	AD7779_GAIN_8,
} ad7779_gain;

typedef enum
{
	AD7779_DOUT_FORMAT_4LINES,
	AD7779_DOUT_FORMAT_2LINES,
	AD7779_DOUT_FORMAT_1LINE
}ad7779_dout_format;

typedef enum
{
	AD7779_HEADER_STATUS,
	AD7779_HEADER_CRC
}ad7779_dout_header_format;

typedef enum
{
	AD7779_LDO_PSM_TEST_DISABLE,
	AD7779_LDO_PSM_TEST_AREGxCAP,
	AD7779_LDO_PSM_TEST_DREGCAP,
	AD7779_LDO_PSM_TEST_ALL
}ad7779_ldo_psm_test_en;

typedef enum
{
	AD7779_LDO_PSM_TRIP_TEST_DISABLE,
	AD7779_LDO_PSM_TRIP_TEST_AREG1CAP,
	AD7779_LDO_PSM_TRIP_TEST_AREG2CAP,
	AD7779_LDO_PSM_TRIP_TEST_DREGCAP
}ad7779_ldo_psm_trip_test_en;

typedef enum
{
	AD7779_DCLK_DIV_1,
	AD7779_DCLK_DIV_2,
	AD7779_DCLK_DIV_4,
	AD7779_DCLK_DIV_8,
	AD7779_DCLK_DIV_16,
	AD7779_DCLK_DIV_32,
	AD7779_DCLK_DIV_64,
	AD7779_DCLK_DIV_128,
} ad7779_dclk_div;

typedef enum
{
	AD7779_REFMUX_EXTREF,
	AD7779_REFMUX_INTREF,
	AD7779_REFMUX_EXTPWR_AVDD1X_AVSSX,
	AD7779_REFMUX_EXTREF_INVERSE
}ad7779_ref_mux;

typedef enum
{
	AD7779_MTRMUX_280mV = 0x02,
	AD7779_MTRMUX_EXTREF = 0x03,
	AD7779_MTRMUX_EXTREF_INVERSE = 0x04,
	AD7779_MTRMUX_EXTREF_ALLNEG = 0x05,
	AD7779_MTRMUX_INTREF = 0x06,
	AD7779_MTRMUX_INTREF_INVERSE = 0x07,
	AD7779_MTRMUX_INTREF_ALLPOS = 0x08,
	AD7779_MTRMUX_EXTREF_ALLPOS = 0x09
}ad7779_mtr_mux;

typedef enum
{
	AD7779_HIGH_RES,
	AD7779_LOW_PWR,
} ad7779_pwr_mode;

typedef enum
{
	AD7779_EXT_REF,
	AD7779_INT_REF,
} ad7779_ref_type;

typedef enum
{
	AD7779_AUXAINP_AUXAINN,
	AD7779_DVBE_AVSSX,
	AD7779_REF1P_REF1N,
	AD7779_REF2P_REF2N,
	AD7779_REF_OUT_AVSSX,
	AD7779_VCM_AVSSX,
	AD7779_AREG1CAP_AVSSX_ATT,
	AD7779_AREG2CAP_AVSSX_ATT,
	AD7779_DREGCAP_DGND_ATT,
	AD7779_AVDD1A_AVSSX_ATT,
	AD7779_AVDD1B_AVSSX_ATT,
	AD7779_AVDD2A_AVSSX_ATT,
	AD7779_AVDD2B_AVSSX_ATT,
	AD7779_IOVDD_DGND_ATT,
	AD7779_AVDD4_AVSSX,
	AD7779_DGND_AVSS1A_ATT,
	AD7779_DGND_AVSS1B_ATT,
	AD7779_DGND_AVSSX_ATT,
	AD7779_AVDD4_AVSSX_ATT,
	AD7779_REF1P_AVSSX,
	AD7779_REF2P_AVSSX,
	AD7779_AVSSX_AVDD4_ATT,
} ad7779_sar_mux;

// Hardware
void AD7779_Interface_Initialize(void); // Initialize the GPIO/SPI/EXTI for AD7779
void AD7779_HardwareReset(void); // Hardware reset AD7779 by output a reset signal
uint8_t AD7779_ReadRegister(uint16_t address); // Read the register from AD7779
void AD7779_WriteRegister(uint8_t address, uint8_t data); // Write the register from AD7779

// Collection
void AD7779_Collection_Start(void); // Enable the interrupt for start collection from AD7779
void AD7779_Collection_Stop(void); // Disable the interrupt for stop collection from AD7779

// Communication and transmission
void AD7779_Set_SPIOperationMode(ad7779_spi_op_mode mode); // Set the SPI operation mode

// Power mode
void AD7779_Set_PowerMode(ad7779_pwr_mode mode); // Set the power mode of AD7779
ad7779_pwr_mode AD7779_Get_PowerMode(void); // Get the power mode of AD7779

// Output data rate(ODR)
void AD7779_Set_OutputRate(double odr_kHz); // Set the output data rate (sample rate) by SRC_N/SRC_IF

// Output format/DCLK frequency division
void AD7779_Set_OutputFormat(ad7779_dout_format format); // Set DOUT format
void AD7779_Set_OutputHeaderFormat(ad7779_dout_header_format format); // Set DOUT header format
void AD7779_Set_DCLKDivision(ad7779_dclk_div div); // Set DCLK frequency division

// Main ADC(Σ-Δ) reference/multiplexing control
void AD7779_Set_SigmaDelta_ReferenceMultiplexing(ad7779_ref_mux mux); // Set Σ-Δ reference multiplexing
void AD7779_Set_SigmaDelta_ADCMultiplexing(ad7779_mtr_mux mux); // Set Σ-Δ ADC multiplexing

// Global SAR diagnosis multiplexing
void AD7779_Set_SAR_Multiplexing(ad7779_sar_mux mux); // Set SAR ADC multiplexing

// GPIOs
void AD7779_GPIO_SetMode(uint8_t gpio_pin, uint8_t gpio_mode); // Set GPIO mode
void AD7779_GPIO_WritePin(uint8_t gpio_pin, GPIO_PinState state); // Write GPIO pin
GPIO_PinState AD7779_GPIO_ReadPin(uint8_t gpio_pin); // Read GPIO pin

// General errors
uint16_t AD7779_Get_GeneralError(void); // Get general error flags
void AD7779_Set_GeneralErrorCheckEnable(uint16_t error_en, ad7779_ldo_psm_test_en psm_en, ad7779_ldo_psm_trip_test_en psm_trip_en); // Enable/disable the general error check

// Channel enable/disable
void AD7779_EnableChannel(ad7779_ch channel); // Enable the channel
void AD7779_DisableChannel(ad7779_ch channel); // Disable the channel
void AD7779_Set_ChannelEnable(ad7779_ch channel, ad7779_state state); // Enable/disable channel

// Channel special function
void AD7779_Set_ChannelPGAGain(ad7779_ch channel, ad7779_gain gain); // Set the PGA (programmable gain amplifier) gain of channel
void AD7779_Set_ChannelReferenceMonitor(ad7779_ch channel, ad7779_state state); // Enable/disable reference moniter function of channel
void AD7779_Set_ChannelMuxRxMode(ad7779_ch channel, ad7779_state state); // Enable/disable multiplexing RX mode

// Channel synchrony offset
void AD7779_Set_ChannelSyncOffset(ad7779_ch channel, uint8_t offset); // Set the synchrony offset of channel

// Channel offset/gain 
void AD7779_Set_ChannelOffset(ad7779_ch channel, uint32_t offset); // Set the offset of channel
void AD7779_Set_ChannelGain(ad7779_ch channel, uint32_t gain); // Set the gain of channel

// Channel errors
uint8_t AD7779_Get_ChannelError(ad7779_ch channel); // Get the error status of channel
uint8_t AD7779_Get_ChannelDSPError(ad7779_ch channel); // Get the DSP error status of channel
void AD7779_Set_ChannelErrorCheckEnable(uint8_t error_en); // Enable/disable the channel error check

// Get channels value in interrupt
void AD7779_Get_SigmaDelta_Value(int32_t* values); // Get all channels of Σ-Δ ADC's values
void AD7779_Get_SigmaDelta_Original(uint32_t* datas); // Get all channels of Σ-Δ ADC's original data return from SPI
