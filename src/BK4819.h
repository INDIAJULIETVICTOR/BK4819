/*
 * Project Name: Radio Firmware
 * File: BK4819.h
 *
 * Copyright (C) 2024 Fabrizio Palumbo (IU0IJV)
 * 
 * This program is distributed under the terms of the MIT license.
 * You can obtain a copy of the license at:
 * https://opensource.org/licenses/MIT
 *
 * DESCRIPTION:
 * Library implementation for interfacing with the Beken BK4819 radio module.
 *
 * AUTHOR: Fabrizio Palumbo
 * CREATION DATE: October 27, 2024
 *
 * CONTACT: t.me/IU0IJV
 *
 * NOTES:
 * - This implementation includes functions for initializing and controlling the BK4819 module.
 * - Verify correct SPI pin configuration before use.
 */


#ifndef BK4819_H
#define BK4819_H

#include <SPI.h>
#include <Arduino.h>
#include <ArduinoQueue.h> 

// Definizione dei registri principali del BK4819

#define BK4819_REG_02_MASK_FSK_TX_FINISHED		(1U << BK4819_REG_02_SHIFT_FSK_TX)
#define BK4819_REG_02_MASK_FSK_FIFO_ALMOST_EMPTY	(1U << BK4819_REG_02_SHIFT_FSK_FIFO_ALMOST_EMPTY)
#define BK4819_REG_02_MASK_FSK_RX_FINISHED		(1U << BK4819_REG_02_SHIFT_FSK_RX_FINISHED)
#define BK4819_REG_02_MASK_FSK_FIFO_ALMOST_FULL		(1U << BK4819_REG_02_SHIFT_FSK_FIFO_ALMOST_FULL)
#define BK4819_REG_02_MASK_DTMF_5TONE_FOUND		(1U << BK4819_REG_02_SHIFT_DTMF_5TONE_FOUND)
#define BK4819_REG_02_MASK_CxCSS_TAIL			(1U << BK4819_REG_02_SHIFT_CxCSS_TAIL)
#define BK4819_REG_02_MASK_CDCSS_FOUND			(1U << BK4819_REG_02_SHIFT_CDCSS_FOUND)
#define BK4819_REG_02_MASK_CDCSS_LOST			(1U << BK4819_REG_02_SHIFT_CDCSS_LOST)
#define BK4819_REG_02_MASK_CTCSS_FOUND			(1U << BK4819_REG_02_SHIFT_CTCSS_FOUND)
#define BK4819_REG_02_MASK_CTCSS_LOST			(1U << BK4819_REG_02_SHIFT_CTCSS_LOST)
#define BK4819_REG_02_MASK_VOX_FOUND			(1U << BK4819_REG_02_SHIFT_VOX_FOUND)
#define BK4819_REG_02_MASK_VOX_LOST			(1U << BK4819_REG_02_SHIFT_VOX_LOST)
#define BK4819_REG_02_MASK_SQUELCH_FOUND		(1U << BK4819_REG_02_SHIFT_SQUELCH_FOUND)
#define BK4819_REG_02_MASK_SQUELCH_LOST			(1U << BK4819_REG_02_SHIFT_SQUELCH_LOST)
#define BK4819_REG_02_MASK_FSK_RX_SYNC			(1U << BK4819_REG_02_SHIFT_FSK_RX_SYNC)

#define BK4819_REG_02_FSK_TX_FINISHED			(1U << BK4819_REG_02_SHIFT_FSK_TX_FINISHED)
#define BK4819_REG_02_FSK_FIFO_ALMOST_EMPTY		(1U << BK4819_REG_02_SHIFT_FSK_FIFO_ALMOST_EMPTY)
#define BK4819_REG_02_FSK_RX_FINISHED			(1U << BK4819_REG_02_SHIFT_FSK_RX_FINISHED)
#define BK4819_REG_02_FSK_FIFO_ALMOST_FULL		(1U << BK4819_REG_02_SHIFT_FSK_FIFO_ALMOST_FULL)
#define BK4819_REG_02_DTMF_5TONE_FOUND			(1U << BK4819_REG_02_SHIFT_DTMF_5TONE_FOUND)
#define BK4819_REG_02_CxCSS_TAIL			(1U << BK4819_REG_02_SHIFT_CxCSS_TAIL)
#define BK4819_REG_02_CDCSS_FOUND			(1U << BK4819_REG_02_SHIFT_CDCSS_FOUND)
#define BK4819_REG_02_CDCSS_LOST			(1U << BK4819_REG_02_SHIFT_CDCSS_LOST)
#define BK4819_REG_02_CTCSS_FOUND			(1U << BK4819_REG_02_SHIFT_CTCSS_FOUND)
#define BK4819_REG_02_CTCSS_LOST			(1U << BK4819_REG_02_SHIFT_CTCSS_LOST)
#define BK4819_REG_02_VOX_FOUND				(1U << BK4819_REG_02_SHIFT_VOX_FOUND)
#define BK4819_REG_02_VOX_LOST				(1U << BK4819_REG_02_SHIFT_VOX_LOST)
#define BK4819_REG_02_SQUELCH_FOUND			(1U << BK4819_REG_02_SHIFT_SQUELCH_FOUND)
#define BK4819_REG_02_SQUELCH_LOST			(1U << BK4819_REG_02_SHIFT_SQUELCH_LOST)
#define BK4819_REG_02_FSK_RX_SYNC			(1U << BK4819_REG_02_SHIFT_FSK_RX_SYNC) 


// REG 07

#define BK4819_REG_07_SHIFT_FREQUENCY_MODE	13
#define BK4819_REG_07_SHIFT_FREQUENCY		0

#define BK4819_REG_07_MASK_FREQUENCY_MODE	(0x0007U << BK4819_REG_07_SHIFT_FREQUENCY_MODE)
#define BK4819_REG_07_MASK_FREQUENCY		(0x1FFFU << BK4819_REG_07_SHIFT_FREQUENCY)

#define BK4819_REG_07_MODE_CTC1			(0U << BK4819_REG_07_SHIFT_FREQUENCY_MODE)
#define BK4819_REG_07_MODE_CTC2			(1U << BK4819_REG_07_SHIFT_FREQUENCY_MODE)
#define BK4819_REG_07_MODE_CDCSS		(2U << BK4819_REG_07_SHIFT_FREQUENCY_MODE)

// REG 24

#define BK4819_REG_24_SHIFT_UNKNOWN_15   15
#define BK4819_REG_24_SHIFT_THRESHOLD    7
#define BK4819_REG_24_SHIFT_UNKNOWN_6    6
#define BK4819_REG_24_SHIFT_ENABLE       5
#define BK4819_REG_24_SHIFT_SELECT       4
#define BK4819_REG_24_SHIFT_MAX_SYMBOLS  0

#define BK4819_REG_24_MASK_THRESHOLD     (0x2Fu << BK4819_REG_24_SHIFT_THRESHOLD)
#define BK4819_REG_24_MASK_ENABLE        (0x01u << BK4819_REG_24_SHIFT_ENABLE)
#define BK4819_REG_24_MASK_SELECT        (0x04u << BK4819_REG_24_SHIFT_SELECT)
#define BK4819_REG_24_MASK_MAX_SYMBOLS   (0x0Fu << BK4819_REG_24_SHIFT_MAX_SYMBOLS)

#define BK4819_REG_24_ENABLE             (1u << BK4819_REG_24_SHIFT_ENABLE)
#define BK4819_REG_24_DISABLE            (0u << BK4819_REG_24_SHIFT_ENABLE)
#define BK4819_REG_24_SELECT_DTMF        (1u << BK4819_REG_24_SHIFT_SELECT)
#define BK4819_REG_24_SELECT_SELCALL     (0u << BK4819_REG_24_SHIFT_SELECT)

// REG 30

#define BK4819_REG_30_SHIFT_ENABLE_VCO_CALIB	15
#define BK4819_REG_30_SHIFT_ENABLE_UNKNOWN	14
#define BK4819_REG_30_SHIFT_ENABLE_RX_LINK	10
#define BK4819_REG_30_SHIFT_ENABLE_AF_DAC	9
#define BK4819_REG_30_SHIFT_ENABLE_DISC_MODE	8
#define BK4819_REG_30_SHIFT_ENABLE_PLL_VCO	4
#define BK4819_REG_30_SHIFT_ENABLE_PA_GAIN	3
#define BK4819_REG_30_SHIFT_ENABLE_MIC_ADC	2
#define BK4819_REG_30_SHIFT_ENABLE_TX_DSP	1
#define BK4819_REG_30_SHIFT_ENABLE_RX_DSP	0

#define BK4819_REG_30_MASK_ENABLE_VCO_CALIB	(0x1U << BK4819_REG_30_SHIFT_ENABLE_VCO_CALIB)
#define BK4819_REG_30_MASK_ENABLE_UNKNOWN	(0x1U << BK4819_REG_30_SHIFT_ENABLE_UNKNOWN)
#define BK4819_REG_30_MASK_ENABLE_RX_LINK	(0xFU << BK4819_REG_30_SHIFT_ENABLE_RX_LINK)
#define BK4819_REG_30_MASK_ENABLE_AF_DAC	(0x1U << BK4819_REG_30_SHIFT_ENABLE_AF_DAC)
#define BK4819_REG_30_MASK_ENABLE_DISC_MODE	(0x1U << BK4819_REG_30_SHIFT_ENABLE_DISC_MODE)
#define BK4819_REG_30_MASK_ENABLE_PLL_VCO	(0xFU << BK4819_REG_30_SHIFT_ENABLE_PLL_VCO)
#define BK4819_REG_30_MASK_ENABLE_PA_GAIN	(0x1U << BK4819_REG_30_SHIFT_ENABLE_PA_GAIN)
#define BK4819_REG_30_MASK_ENABLE_MIC_ADC	(0x1U << BK4819_REG_30_SHIFT_ENABLE_MIC_ADC)
#define BK4819_REG_30_MASK_ENABLE_TX_DSP	(0x1U << BK4819_REG_30_SHIFT_ENABLE_TX_DSP)
#define BK4819_REG_30_MASK_ENABLE_RX_DSP	(0x1U << BK4819_REG_30_SHIFT_ENABLE_RX_DSP)

enum {
	BK4819_REG_30_ENABLE_VCO_CALIB		= (0x1U << BK4819_REG_30_SHIFT_ENABLE_VCO_CALIB),
	BK4819_REG_30_DISABLE_VCO_CALIB		= (0x0U << BK4819_REG_30_SHIFT_ENABLE_VCO_CALIB),
	BK4819_REG_30_ENABLE_UNKNOWN		= (0x1U << BK4819_REG_30_SHIFT_ENABLE_UNKNOWN),
	BK4819_REG_30_DISABLE_UNKNOWN		= (0x0U << BK4819_REG_30_SHIFT_ENABLE_UNKNOWN),
	BK4819_REG_30_ENABLE_RX_LINK		= (0xFU << BK4819_REG_30_SHIFT_ENABLE_RX_LINK),
	BK4819_REG_30_DISABLE_RX_LINK		= (0x0U << BK4819_REG_30_SHIFT_ENABLE_RX_LINK),
	BK4819_REG_30_ENABLE_AF_DAC		= (0x1U << BK4819_REG_30_SHIFT_ENABLE_AF_DAC),
	BK4819_REG_30_DISABLE_AF_DAC		= (0x0U << BK4819_REG_30_SHIFT_ENABLE_AF_DAC),
	BK4819_REG_30_ENABLE_DISC_MODE		= (0x1U << BK4819_REG_30_SHIFT_ENABLE_DISC_MODE),
	BK4819_REG_30_DISABLE_DISC_MODE		= (0x0U << BK4819_REG_30_SHIFT_ENABLE_DISC_MODE),
	BK4819_REG_30_ENABLE_PLL_VCO		= (0xFU << BK4819_REG_30_SHIFT_ENABLE_PLL_VCO),
	BK4819_REG_30_DISABLE_PLL_VCO		= (0x0U << BK4819_REG_30_SHIFT_ENABLE_PLL_VCO),
	BK4819_REG_30_ENABLE_PA_GAIN		= (0x1U << BK4819_REG_30_SHIFT_ENABLE_PA_GAIN),
	BK4819_REG_30_DISABLE_PA_GAIN		= (0x0U << BK4819_REG_30_SHIFT_ENABLE_PA_GAIN),
	BK4819_REG_30_ENABLE_MIC_ADC		= (0x1U << BK4819_REG_30_SHIFT_ENABLE_MIC_ADC),
	BK4819_REG_30_DISABLE_MIC_ADC		= (0x0U << BK4819_REG_30_SHIFT_ENABLE_MIC_ADC),
	BK4819_REG_30_ENABLE_TX_DSP		= (0x1U << BK4819_REG_30_SHIFT_ENABLE_TX_DSP),
	BK4819_REG_30_DISABLE_TX_DSP		= (0x0U << BK4819_REG_30_SHIFT_ENABLE_TX_DSP),
	BK4819_REG_30_ENABLE_RX_DSP		= (0x1U << BK4819_REG_30_SHIFT_ENABLE_RX_DSP),
	BK4819_REG_30_DISABLE_RX_DSP		= (0x0U << BK4819_REG_30_SHIFT_ENABLE_RX_DSP),
};

// REG 3F

#define BK4819_REG_3F_SHIFT_FSK_TX_FINISHED		15
#define BK4819_REG_3F_SHIFT_FSK_FIFO_ALMOST_EMPTY	14
#define BK4819_REG_3F_SHIFT_FSK_RX_FINISHED		13
#define BK4819_REG_3F_SHIFT_FSK_FIFO_ALMOST_FULL	12
#define BK4819_REG_3F_SHIFT_DTMF_5TONE_FOUND		11
#define BK4819_REG_3F_SHIFT_CxCSS_TAIL			10
#define BK4819_REG_3F_SHIFT_CDCSS_FOUND			9
#define BK4819_REG_3F_SHIFT_CDCSS_LOST			8
#define BK4819_REG_3F_SHIFT_CTCSS_FOUND			7
#define BK4819_REG_3F_SHIFT_CTCSS_LOST			6
#define BK4819_REG_3F_SHIFT_VOX_FOUND			5
#define BK4819_REG_3F_SHIFT_VOX_LOST			4
#define BK4819_REG_3F_SHIFT_SQUELCH_FOUND		3
#define BK4819_REG_3F_SHIFT_SQUELCH_LOST		2
#define BK4819_REG_3F_SHIFT_FSK_RX_SYNC			1

#define BK4819_REG_3F_MASK_FSK_TX_FINISHED		(1U << BK4819_REG_3F_SHIFT_FSK_TX)
#define BK4819_REG_3F_MASK_FSK_FIFO_ALMOST_EMPTY	(1U << BK4819_REG_3F_SHIFT_FSK_FIFO_ALMOST_EMPTY)
#define BK4819_REG_3F_MASK_FSK_RX_FINISHED		(1U << BK4819_REG_3F_SHIFT_FSK_RX_FINISHED)
#define BK4819_REG_3F_MASK_FSK_FIFO_ALMOST_FULL		(1U << BK4819_REG_3F_SHIFT_FSK_FIFO_ALMOST_FULL)
#define BK4819_REG_3F_MASK_DTMF_5TONE_FOUND		(1U << BK4819_REG_3F_SHIFT_DTMF_5TONE_FOUND)
#define BK4819_REG_3F_MASK_CxCSS_TAIL			(1U << BK4819_REG_3F_SHIFT_CxCSS_TAIL)
#define BK4819_REG_3F_MASK_CDCSS_FOUND			(1U << BK4819_REG_3F_SHIFT_CDCSS_FOUND)
#define BK4819_REG_3F_MASK_CDCSS_LOST			(1U << BK4819_REG_3F_SHIFT_CDCSS_LOST)
#define BK4819_REG_3F_MASK_CTCSS_FOUND			(1U << BK4819_REG_3F_SHIFT_CTCSS_FOUND)
#define BK4819_REG_3F_MASK_CTCSS_LOST			(1U << BK4819_REG_3F_SHIFT_CTCSS_LOST)
#define BK4819_REG_3F_MASK_VOX_FOUND			(1U << BK4819_REG_3F_SHIFT_VOX_FOUND)
#define BK4819_REG_3F_MASK_VOX_LOST			(1U << BK4819_REG_3F_SHIFT_VOX_LOST)
#define BK4819_REG_3F_MASK_SQUELCH_FOUND		(1U << BK4819_REG_3F_SHIFT_SQUELCH_FOUND)
#define BK4819_REG_3F_MASK_SQUELCH_LOST			(1U << BK4819_REG_3F_SHIFT_SQUELCH_LOST)
#define BK4819_REG_3F_MASK_FSK_RX_SYNC			(1U << BK4819_REG_3F_SHIFT_FSK_RX_SYNC)

#define BK4819_REG_3F_FSK_TX_FINISHED			(1U << BK4819_REG_3F_SHIFT_FSK_TX_FINISHED)
#define BK4819_REG_3F_FSK_FIFO_ALMOST_EMPTY		(1U << BK4819_REG_3F_SHIFT_FSK_FIFO_ALMOST_EMPTY)
#define BK4819_REG_3F_FSK_RX_FINISHED			(1U << BK4819_REG_3F_SHIFT_FSK_RX_FINISHED)
#define BK4819_REG_3F_FSK_FIFO_ALMOST_FULL		(1U << BK4819_REG_3F_SHIFT_FSK_FIFO_ALMOST_FULL)
#define BK4819_REG_3F_DTMF_5TONE_FOUND			(1U << BK4819_REG_3F_SHIFT_DTMF_5TONE_FOUND)
#define BK4819_REG_3F_CxCSS_TAIL			(1U << BK4819_REG_3F_SHIFT_CxCSS_TAIL)
#define BK4819_REG_3F_CDCSS_FOUND			(1U << BK4819_REG_3F_SHIFT_CDCSS_FOUND)
#define BK4819_REG_3F_CDCSS_LOST			(1U << BK4819_REG_3F_SHIFT_CDCSS_LOST)
#define BK4819_REG_3F_CTCSS_FOUND			(1U << BK4819_REG_3F_SHIFT_CTCSS_FOUND)
#define BK4819_REG_3F_CTCSS_LOST			(1U << BK4819_REG_3F_SHIFT_CTCSS_LOST)
#define BK4819_REG_3F_VOX_FOUND				(1U << BK4819_REG_3F_SHIFT_VOX_FOUND)
#define BK4819_REG_3F_VOX_LOST				(1U << BK4819_REG_3F_SHIFT_VOX_LOST)
#define BK4819_REG_3F_SQUELCH_FOUND			(1U << BK4819_REG_3F_SHIFT_SQUELCH_FOUND)
#define BK4819_REG_3F_SQUELCH_LOST			(1U << BK4819_REG_3F_SHIFT_SQUELCH_LOST)
#define BK4819_REG_3F_FSK_RX_SYNC			(1U << BK4819_REG_3F_SHIFT_FSK_RX_SYNC)

// REG 51

#define BK4819_REG_51_SHIFT_ENABLE_CxCSS        15
#define BK4819_REG_51_SHIFT_GPIO6_PIN2_INPUT    14
#define BK4819_REG_51_SHIFT_TX_CDCSS_POLARITY   13
#define BK4819_REG_51_SHIFT_CxCSS_MODE          12
#define BK4819_REG_51_SHIFT_CDCSS_BIT_WIDTH     11
#define BK4819_REG_51_SHIFT_1050HZ_DETECTION    10
#define BK4819_REG_51_SHIFT_AUTO_CDCSS_BW       9
#define BK4819_REG_51_SHIFT_AUTO_CTCSS_BW       8
#define BK4819_REG_51_SHIFT_CxCSS_TX_GAIN1      0

#define BK4819_REG_51_MASK_ENABLE_CxCSS        (0x01U << BK4819_REG_51_SHIFT_ENABLE_CxCSS)
#define BK4819_REG_51_MASK_GPIO6_PIN2_INPUT    (0x01U << BK4819_REG_51_SHIFT_GPIO6_PIN2_INPUT)
#define BK4819_REG_51_MASK_TX_CDCSS_POLARITY   (0x01U << BK4819_REG_51_SHIFT_TX_CDCSS_POLARITY)
#define BK4819_REG_51_MASK_CxCSS_MODE          (0x01U << BK4819_REG_51_SHIFT_CxCSS_MODE)
#define BK4819_REG_51_MASK_CDCSS_BIT_WIDTH     (0x01U << BK4819_REG_51_SHIFT_CDCSS_BIT_WIDTH)
#define BK4819_REG_51_MASK_1050HZ_DETECTION    (0x01U << BK4819_REG_51_SHIFT_1050HZ_DETECTION)
#define BK4819_REG_51_MASK_AUTO_CDCSS_BW       (0x01U << BK4819_REG_51_SHIFT_AUTO_CDCSS_BW)
#define BK4819_REG_51_MASK_AUTO_CTCSS_BW       (0x01U << BK4819_REG_51_SHIFT_AUTO_CTCSS_BW)
#define BK4819_REG_51_MASK_CxCSS_TX_GAIN1      (0x7FU << BK4819_REG_51_SHIFT_CxCSS_TX_GAIN1)


enum {
	BK4819_REG_51_ENABLE_CxCSS          = (1U << BK4819_REG_51_SHIFT_ENABLE_CxCSS),
	BK4819_REG_51_DISABLE_CxCSS         = (0U << BK4819_REG_51_SHIFT_ENABLE_CxCSS),

	BK4819_REG_51_GPIO6_PIN2_INPUT      = (1U << BK4819_REG_51_SHIFT_GPIO6_PIN2_INPUT),
	BK4819_REG_51_GPIO6_PIN2_NORMAL     = (0U << BK4819_REG_51_SHIFT_GPIO6_PIN2_INPUT),

	BK4819_REG_51_TX_CDCSS_NEGATIVE     = (1U << BK4819_REG_51_SHIFT_TX_CDCSS_POLARITY),
	BK4819_REG_51_TX_CDCSS_POSITIVE     = (0U << BK4819_REG_51_SHIFT_TX_CDCSS_POLARITY),

	BK4819_REG_51_MODE_CTCSS            = (1U << BK4819_REG_51_SHIFT_CxCSS_MODE),
	BK4819_REG_51_MODE_CDCSS            = (0U << BK4819_REG_51_SHIFT_CxCSS_MODE),

	BK4819_REG_51_CDCSS_24_BIT          = (1U << BK4819_REG_51_SHIFT_CDCSS_BIT_WIDTH),
	BK4819_REG_51_CDCSS_23_BIT          = (0U << BK4819_REG_51_SHIFT_CDCSS_BIT_WIDTH),

	BK4819_REG_51_1050HZ_DETECTION      = (1U << BK4819_REG_51_SHIFT_1050HZ_DETECTION),
	BK4819_REG_51_1050HZ_NO_DETECTION   = (0U << BK4819_REG_51_SHIFT_1050HZ_DETECTION),

	BK4819_REG_51_AUTO_CDCSS_BW_DISABLE = (1U << BK4819_REG_51_SHIFT_AUTO_CDCSS_BW),
	BK4819_REG_51_AUTO_CDCSS_BW_ENABLE  = (0U << BK4819_REG_51_SHIFT_AUTO_CDCSS_BW),

	BK4819_REG_51_AUTO_CTCSS_BW_DISABLE = (1U << BK4819_REG_51_SHIFT_AUTO_CTCSS_BW),
	BK4819_REG_51_AUTO_CTCSS_BW_ENABLE  = (0U << BK4819_REG_51_SHIFT_AUTO_CTCSS_BW),
};

// REG 70

#define BK4819_REG_70_SHIFT_ENABLE_TONE1	15
#define BK4819_REG_70_SHIFT_TONE1_TUNING_GAIN	8
#define BK4819_REG_70_SHIFT_ENABLE_TONE2	7
#define BK4819_REG_70_SHIFT_TONE2_TUNING_GAIN	0

#define BK4819_REG_70_MASK_ENABLE_TONE1		(0x01U << BK4819_REG_70_SHIFT_ENABLE_TONE1)
#define BK4819_REG_70_MASK_TONE1_TUNING_GAIN	(0x7FU << BK4819_REG_70_SHIFT_TONE1_TUNING_GAIN)
#define BK4819_REG_70_MASK_ENABLE_TONE2		(0x01U << BK4819_REG_70_SHIFT_ENABLE_TONE2)
#define BK4819_REG_70_MASK_TONE2_TUNING_GAIN	(0x7FU << BK4819_REG_70_SHIFT_TONE2_TUNING_GAIN)

#define QUEUE_MAX_SIZE 10
#define MAX_AGC_TABLE 25

// ----------------------------------------------------
// Struttura per memorizzare i comandi di scrittura nei registri
struct RegisterWriteCommand 
{
    uint16_t address;
    uint16_t data;
};

// ----------------------------------------------------
enum BK4819_Filter_status_t
{
	RX_OFF_TX_OFF = 0,
	RX_ON_TX_OFF,
	RX_OFF_TX_ON
};
typedef enum BK4819_Filter_status_t BK4819_Filter_status_t;

// ----------------------------------------------------
enum AF_Type_t
{
	AF_MUTE      =  0u,  // Mute
	AF_FM        =  1u,  // FM
	AF_TONE      =  2u,  // Tone
	AF_BEEP      =  3u,  // Beep for TX
	AF_RAW 		 =  4u,  // raw (ssb without if filter = raw in sdr sharp)
	AF_DSB 		 =  5u,  // dsb (or ssb = lsb and usb at the same time)
	AF_CTCOUT    =  6u,  // ctcss/dcs (fm with narrow filters for ctcss/dcs)
	AF_AM        =  7u,  // AM
	AF_FSKOUT    =  8u,  // fsk out test with special fsk filters (need reg58 fsk on to give sound on speaker ) // nothing
	AF_BYPASS  	 =  9u,  // bypass (fm without filter = discriminator output) // distorted
	AF_UNKNOWN4  = 10u,  // nothing at all
	AF_UNKNOWN5  = 11u,  // distorted
	AF_UNKNOWN6  = 12u,  // distorted
	AF_UNKNOWN7  = 13u,  // interesting
	AF_UNKNOWN8  = 14u,  // interesting
	AF_UNKNOWN9  = 15u   // not a lot					
};
typedef enum AF_Type_t AF_Type_t;

// ----------------------------------------------------
enum BK4819_Filter_Bandwidth_t
{
    BK4819_FILTER_BW_6k = 0,        // "U 6K",      //0
    BK4819_FILTER_BW_7k,            // "U 7K",      //1
    BK4819_FILTER_BW_9k,            // "N 9k",      //2
    BK4819_FILTER_BW_10k,           // "N 10k",     //3
    BK4819_FILTER_BW_12k,           // "W 12k",     //4
    BK4819_FILTER_BW_14k,           // "W 14k",     //5
    BK4819_FILTER_BW_17k,           // "W 17k",     //6
    BK4819_FILTER_BW_20k,           // "W 20k",     //7
    BK4819_FILTER_BW_23k,           // "W 23k",     //8
    BK4819_FILTER_BW_26k            // "W 26k"      //9
};
typedef enum BK4819_Filter_Bandwidth_t BK4819_Filter_Bandwidth_t;

// ----------------------------------------------------
enum BK4819_Css_Scan_Result_t
{
	BK4819_CSS_RESULT_NOT_FOUND = 0,
	BK4819_CSS_RESULT_CTCSS,
	BK4819_CSS_RESULT_CDCSS
};
typedef enum BK4819_Css_Scan_Result_t BK4819_Css_Scan_Result_t;

// ----------------------------------------------------
enum BK4819_AGC_t
{
	AGC_AUTO = 0,
	AGC_MAN,
	AGC_FAST,
	AGC_NOR, 
	AGC_SLOW,
};
typedef enum BK4819_AGC_t BK4819_AGC_t;

// ----------------------------------------------------
enum BK4819_Mode_t
{
	MODE_FM = 0,
	MODE_AM,
	MODE_LSB,
	MODE_USB,
	MODE_CW,

};
typedef enum BK4819_Mode_t BK4819_Mode_t;

// ----------------------------------------------------
typedef struct
{
    uint16_t reg_val;
	uint8_t  gain_dB;
	
} t_gain_table;

// ----------------------------------------------------
typedef struct
{
	uint8_t AFmode;
} t_Vfo_Data;	

// ----------------------------------------------------
enum BK4819_Xtal_t
{
	XTAL26M = 0,
	XTAL13M,
	XTAL19M2,
	XTAL12M8,
	XTAL25M6,
	XTAL38M4
};
typedef enum BK4819_Xtal_t BK4819_Xtal_t;

// ----------------------------------------------------
enum BK4819_IRQType_t
{
	FSK_RX_Sync = 2,
	Squelch_Lost = 4,
	Squelch_Found = 8,
	VOX_Lost = 0x10,
	VOX_Found = 0x20,
	CTCSS_Lost = 0x40,
	CTCSS_Found = 0x80,
	CDCSS_Lost = 0x100, 
	CDCSS_Found = 0x200, 
	TONE_Tail_Found = 0x400, 
	DTMF_TONE_Found = 0x800, 
	FSK_FIFO_Full = 0x1000, 
	FSK_RX_Finished = 0x2000, 
	FSK_FIFO_Empty = 0x4000 ,
	FSK_TX_Finished = 0x8000

};
typedef enum BK4819_IRQType_t BK4819_IRQType_t;	
	
// ----------------------------------------------------	
enum BK4819_SquelchMode_t
{
	RSSI_NOISE_GLITCH = 0x88,
	RSSI_GLITCH = 0xAA,
	RSSI_NOISE = 0XCC,
	RSSI = 0XFF
};
typedef BK4819_SquelchMode_t BK4819_SquelchMode_t;	


typedef struct 
{
    uint32_t Frequency;   				// Frequenza corrente (es. 145 MHz)
	uint32_t StartFrequency;			// frequenza inizio scansione
	uint32_t EndFrequency;				// frequenza fine scansione
	uint8_t	Step;						// step Frequenza	
    BK4819_Mode_t Mode;         		// Modalità corrente (FM, AM, SSB, ecc.)
	uint8_t AGC;						// controllo AGC		
	uint8_t AFC;						// controllo AFC
    uint8_t Gain;         				// Guadagno RF corrente
    uint16_t ExtGain;         			// Attenuazione RF 
    uint16_t Sql;          				// Livello Squelch corrente
	uint8_t Volume;					    // percentuale volume corrente
	BK4819_Filter_Bandwidth_t bw;		// Bandwith
	
	uint8_t	micGain;					// guadagno microfono
	uint8_t txp;						// potenza trasmissione
	uint8_t Tx_Dev;						
	uint8_t Spcan;						// velocita' scansione * 50 mS		
	uint8_t Spres;						// velocita' ripresa scansione * 50 mS		
	uint8_t Scres;
	
	uint8_t scrambler;
	uint8_t compander;
	uint8_t NB;

	uint8_t beken_module;				// Modulo beken 0 o 1
	uint8_t volume_port;				// porta DAC per volume

	uint8_t Upconverter;
	
    union 
	{
        struct 
		{
            bool monitor:1;		// 0x0001
            bool rx:1;			// 0x0002
            bool tx:1;			// 0x0004
            bool scan:1;		// 0x0008
			bool ctcss:1;		// 0x0010
            bool dcs:1;			// 0x0020
            bool tones:1;       // 0x0040
			bool vuoto:1;		// 0x0080	
            bool flag9:1;		// 0x0100
            bool flag10:1;		// 0x0200
            bool flag11:1;		// 0x0400
            bool flag12:1;		// 0x0800
            bool flag13:1;		// 0x1000
            bool flag14:1;		// 0x2000
            bool flag15:1;		// 0x4000
            bool flag16:1;		// 0x8000

        } bits;  
        uint16_t Flags; 
    } Flag;
	
} VfoData_t;


#define INFO_STEP_SIZE 17 									// Numero di elementi in info_step e freq_step

extern const char* info_step[];   
extern const size_t info_step_size;
extern const uint32_t freq_step[]; 
extern const t_gain_table gain_table[];

// ----------------------------------------------------
// Definizione della classe BK4819 per Arduino
// ----------------------------------------------------
	class BK4819 
	{
		public:
			BK4819(int csPin, int MosiPin, int MisoPin, int sckPin);
			
			BK4819_IRQType_t BK4819_Check_Irq_type( bool direct );
			
			uint16_t BK4819_Read_Register(uint16_t address);  
			
			int16_t BK4819_Get_RSSI (void);
			
			void BK4819_SCN_select( uint8_t csPin );
			
			bool BK4819_Write_Register(uint16_t address, uint16_t data, bool direct);
			void BK4819_Init();  // Dichiara qui la funzione
			void BK4819_RX_TurnOff(bool direct);
			void BK4819_RX_TurnOn(bool direct);
			void BK4819_Sleep(bool direct);
			void BK4819_SoftReset();

			void BK4819_Clear_Interrupt( bool direct);
			void BK4819_IRQ_Set (BK4819_IRQType_t InterruptMask, bool direct);
			void BK4819_Set_Filter_Bandwidth(const BK4819_Filter_Bandwidth_t bandwidth, bool direct);
			
			void BK4819_Init_AGC(bool direct);
			void BK4819_Set_AF(AF_Type_t af, bool direct);
			void BK4819_Set_Frequency(uint32_t Frequency, bool direct);
			void BK4819_Set_AGC_Gain(uint8_t Agc, uint8_t Value, bool direct);
			void BK4819_Set_Xtal(BK4819_Xtal_t mode, bool direct);
			void BK4819_RF_Set_Agc(uint8_t mode, bool direct);
			void BK4819_Set_AFC(uint8_t value, bool direct);
			void BK4819_Squelch_Mode ( BK4819_SquelchMode_t mode, bool direct);
			
			void BK4819_Disable_DTMF(bool direct);
			
			void BK4819_Enable_Mic ( uint8_t MIC_SENSITIVITY_TUNING, bool direct);
			void BK4819_Set_Power_TX(uint8_t level, bool direct);
			void BK4819_Set_Power_Amplifier(const uint8_t bias, uint8_t gain1, uint8_t gain2, bool enable, bool direct);
			void BK4819_Enable_TXLink(bool direct);
			void BK4819_Disable_TXLink(bool direct);
			void BK4819_Mute_Tx(bool mute, bool direct);
			void BK4819_Set_TxDeviation ( uint8_t value, bool direct);
			
			void BK4819_Prepare_Transmit(bool direct);
			void BK4819_TxOn(bool direct);
			
			void BK4819_processRegisterWriteQueue(void);
			void BK4819_Set_Modulation(BK4819_Mode_t Modul, bool direct);
			void BK4819_Set_GPIO_Output(uint8_t gpio_num, bool enable);
			uint8_t BK4819_Get_GPIO(uint8_t gpio_num);

			void BK4819_Set_Squelch	(
				uint8_t Squelch_Open_RSSI,
				uint8_t Squelch_Close_RSSI,
				uint8_t Squelch_Open_Noise,
				uint8_t Squelch_Close_Noise,
				uint8_t Squelch_Close_Glitch,
				uint8_t Squelch_Open_Glitch,
				uint8_t Delay_Open,
				uint8_t Delay_Close,
				bool direct	);
				
			void BK4819_Disable_Scramble(bool direct);				
			void BK4819_Enable_Scramble(uint8_t Type,bool direct);
			

		private:
			int _MosiPin;
			int _csPin;
			int _sckPin;
			int _MisoPin;
		
			bool spiInUse;
			uint16_t GPIO_reg;
	};

#endif