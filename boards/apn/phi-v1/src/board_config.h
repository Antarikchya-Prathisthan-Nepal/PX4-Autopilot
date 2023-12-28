/****************************************************************************
 *
 *   Copyright (c) 2013-2016 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file board_config.h
 *
 * APNPHIv1 internal definitions
 */

#pragma once

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <px4_platform_common/px4_config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

/****************************************************************************************************
 * Definitions
 ****************************************************************************************************/
/* Configuration ************************************************************************************/

/* APNPHIv1 GPIOs ***********************************************************************************/
/**
 * ADC channels:
 * These are the channel numbers of the ADCs of the microcontroller that can be used by the Px4 Firmware in the adc driver.
 */
#define ADC_CHANNELS (1 << 3) | (1 << 4) | (1 << 5) | (1 << 6 ) | \
			 (1 << 7) | (1 << 8) | (1 << 9) | (1 << 10 ) | \
			  (1 << 11) | (1 << 12) | (1 << 13) | (1 << 15 )

/* ADC defines to be used in sensors.cpp to read from a particular channel. */
#define ADC_BATTERY_VOLTAGE_CHANNEL  2
#define ADC_BATTERY_CURRENT_CHANNEL  3
#define ADC_5V_RAIL_SENSE            4
#define ADC_RC_RSSI_CHANNEL          11

/* Watchdog and Reset control GPIOs */
// #define GPIO_WD_WDI			(GPIO_OUTPUT|GPIO_FLOAT|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN8)
#define GPIO_GBL_RST			(GPIO_OUTPUT|GPIO_FLOAT|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN4)

/* KILL SWITCH control and monitoring GPIOs*/
#define GPIO_KILL_SW_EN			(GPIO_OUTPUT|GPIO_FLOAT|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN10)
/* KILL Switch 1 GPIOs */
#define GPIO_KILL_SW1_STAT		(GPIO_INPUT|GPIO_FLOAT|GPIO_PORTC|GPIO_PIN13)
#define GPIO_KILL_SW1_NEG		(GPIO_OUTPUT|GPIO_FLOAT|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN4)
#define GPIO_KILL_SW1_POS		(GPIO_OUTPUT|GPIO_FLOAT|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN5)

/* KILL Switch 2 GPOIs */  //(GPIO_MODE|GPIO_PULL_DEF|GPIO_SPEED|GPIO_OUTPUT|GPIO_PORT|GPIO_PIN|)
#define GPIO_KILL_SW2_STAT		(GPIO_INPUT|GPIO_FLOAT|GPIO_PORTE|GPIO_PIN3)
#define GPIO_KILL_SW2_NEG		(GPIO_OUTPUT|GPIO_FLOAT|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN7)
#define GPIO_KILL_SW2_POS		(GPIO_OUTPUT|GPIO_FLOAT|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN6)


/* Power supply control GPIOs. */
#define GPIO_DCDC_IMU_EN		(GPIO_OUTPUT|GPIO_FLOAT|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN9)
#define GPIO_DCDC_MSN			(GPIO_OUTPUT|GPIO_FLOAT|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN8)

#define GPIO_MSN_3V3_EN			(GPIO_OUTPUT|GPIO_FLOAT|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN10)
#define GPIO_MSN_5V_EN			(GPIO_OUTPUT|GPIO_FLOAT|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN11)
#define GPIO_BURNER_EN			(GPIO_OUTPUT|GPIO_FLOAT|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN12)
#define GPIO_UNREG_EN			(GPIO_OUTPUT|GPIO_FLOAT|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN13)

/* MSN Enable GPIOs */
#define GPIO_MSN1_EN			(GPIO_OUTPUT|GPIO_FLOAT|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN15)

/* Flash Memory Control GPIOs */
#define GPIO_MFM_RST			(GPIO_OUTPUT|GPIO_FLOAT|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTD|GPIO_PIN2)
#define GPIO_MFM_HOLD			(GPIO_OUTPUT|GPIO_FLOAT|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTD|GPIO_PIN1)
#define GPIO_MFM_WP			(GPIO_OUTPUT|GPIO_FLOAT|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTD|GPIO_PIN3)

#define GPIO_SFM_RST			(GPIO_OUTPUT|GPIO_FLOAT|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN2)
#define GPIO_SFM_HOLD			(GPIO_OUTPUT|GPIO_FLOAT|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN4)
#define GPIO_SFM_WP			(GPIO_OUTPUT|GPIO_FLOAT|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN8)

#define GPIO_SFM_MODE			(GPIO_OUTPUT|GPIO_FLOAT|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN9)
#define GPIO_MUX_EN			(GPIO_OUTPUT|GPIO_FLOAT|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN10)
/* Tone alarm output. */
// #define TONE_ALARM_TIMER             2    /* timer 2 */
// #define TONE_ALARM_CHANNEL           1    /* channel 1 */
// #define GPIO_TONE_ALARM_IDLE         (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN15)
// #define GPIO_TONE_ALARM              (GPIO_ALT|GPIO_AF1|GPIO_SPEED_2MHz|GPIO_PUSHPULL|GPIO_PORTA|GPIO_PIN15)

/**
 * PWM:
 *
 * Six PWM outputs are configured.
 */
#define DIRECT_PWM_OUTPUT_CHANNELS   6

// /**
//  * USB OTG FS:
//  * PA9  OTG_FS_VBUS VBUS sensing.
//  */
// #define GPIO_OTGFS_VBUS              (GPIO_INPUT|GPIO_FLOAT|GPIO_SPEED_100MHz|GPIO_OPENDRAIN|GPIO_PORTA|GPIO_PIN9)
#define BOARD_USB_VBUS_SENSE_DISABLED
/* High-resolution timer */
#define HRT_TIMER                    3  /* use timer 3 for the HRT */
#define HRT_TIMER_CHANNEL            4  /* use capture/compare channel 4 */

// #define HRT_PPM_CHANNEL              3  /* use capture/compare channel 3 */
// #define GPIO_PPM_IN                  (GPIO_ALT|GPIO_AF2|GPIO_PULLUP|GPIO_PORTD|GPIO_PIN11)

/* RC Serial port */

#define RC_SERIAL_PORT               "/dev/ttyS2"

/* PWM input driver. Use FMU AUX5 pins attached to timer4 channel 2. */
// #define PWMIN_TIMER                  4
// #define PWMIN_TIMER_CHANNEL          2
// #define GPIO_PWM_IN                  GPIO_TIM4_CH2IN_2

/* For R12, this signal is active high. */
// #define GPIO_SBUS_INV                (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN13)
// #define RC_INVERT_INPUT(_invert_true) px4_arch_gpiowrite(GPIO_SBUS_INV, _invert_true)

/**
 * By Providing BOARD_ADC_USB_CONNECTED (using the px4_arch abstraction)
 * this board support the ADC system_power interface, and therefore
 * provides the true logic GPIO BOARD_ADC_xxxx macros.
 */
#define BOARD_ADC_BRICK_VALID        (1)
#define BOARD_ADC_SERVO_VALID        (1)
#define BOARD_ADC_PERIPH_5V_OC       (0)
#define BOARD_ADC_HIPOWER_5V_OC      (0)


/* This board provides a DMA pool and APIs */
#define BOARD_DMA_ALLOC_POOL_SIZE 5120

#define BOARD_HAS_ON_RESET 1


__BEGIN_DECLS

/****************************************************************************************************
 * Public Types
 ****************************************************************************************************/

/****************************************************************************************************
 * Public data
 ****************************************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************************************
 * Public Functions
 ****************************************************************************************************/

/****************************************************************************************************
 * Name: stm32_spiinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the PX4FMU board.
 *
 ****************************************************************************************************/

extern void stm32_spiinitialize(void);

extern void stm32_usbinitialize(void);

extern void board_peripheral_reset(int ms);

#include <px4_platform_common/board_common.h>

#endif /* __ASSEMBLY__ */

__END_DECLS
