/*
Copyright (c) 2014-2015 Arduino LLC.  All right reserved.

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
See the GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef _VARIANT_ES_SAMD21_WB
#define _VARIANT_ES_SAMD21_WB

// The definitions here needs a SAMD core >=1.6.10
#define ARDUINO_SAMD_VARIANT_COMPLIANCE 10610

/*----------------------------------------------------------------------------
*        Definitions
*----------------------------------------------------------------------------*/

/** Frequency of the board main oscillator */
#define VARIANT_MAINOSC		(32768ul)

/** Master clock frequency */
#define VARIANT_MCK			  (48000000ul)

/*----------------------------------------------------------------------------
*        Headers
*----------------------------------------------------------------------------*/

#include "WVariant.h"

#ifdef __cplusplus
#include "SERCOM.h"
#include "Uart.h"
#endif // __cplusplus

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

	/*----------------------------------------------------------------------------
	*        Pins
	*----------------------------------------------------------------------------*/

	// Number of pins defined in PinDescription array
#ifdef __cplusplus
	extern "C" unsigned int PINCOUNT_fn();
#endif
#define PINS_COUNT           (PINCOUNT_fn())
#define NUM_DIGITAL_PINS     (36u)
#define NUM_ANALOG_INPUTS    (14u)
#define NUM_ANALOG_OUTPUTS   (1u)
#define analogInputToDigitalPin(p)  ((p < 14u) ? (p) + 14u : -1)

#define digitalPinToPort(P)        ( &(PORT->Group[g_APinDescription[P].ulPort]) )
#define digitalPinToBitMask(P)     ( 1 << g_APinDescription[P].ulPin )
	//#define analogInPinToBit(P)        ( )
#define portOutputRegister(port)   ( &(port->OUT.reg) )
#define portInputRegister(port)    ( &(port->IN.reg) )
#define portModeRegister(port)     ( &(port->DIR.reg) )
#define digitalPinHasPWM(P)        ( g_APinDescription[P].ulPWMChannel != NOT_ON_PWM || g_APinDescription[P].ulTCChannel != NOT_ON_TIMER )

	/*
	* digitalPinToTimer(..) is AVR-specific and is not defined for SAMD
	* architecture. If you need to check if a pin supports PWM you must
	* use digitalPinHasPWM(..).
	*
	* https://github.com/arduino/Arduino/issues/1833
	*/
	// #define digitalPinToTimer(P)

	// LEDs
#define PIN_LED_13           (13u)
#define PIN_LED              PIN_LED_13
#define LED_BUILTIN          PIN_LED

	/*
	* Analog pins
	*/
#define PIN_A0               (14ul)
#define PIN_A1               (PIN_A0 + 1)
#define PIN_A2               (PIN_A0 + 2)
#define PIN_A3               (PIN_A0 + 3)
#define PIN_A4               (PIN_A0 + 4)
#define PIN_A5               (PIN_A0 + 5)
#define PIN_A6               (PIN_A0 + 6)
#define PIN_A7               (PIN_A0 + 7)
#define PIN_A8               (PIN_A0 + 8)
#define PIN_A9               (PIN_A0 + 9)
#define PIN_A10              (PIN_A0 + 10)
#define PIN_A11              (PIN_A0 + 11)
#define PIN_A12              (PIN_A0 + 12)
#define PIN_A13              (PIN_A0 + 13)
#define PIN_DAC0             PIN_A1

static const uint8_t A0 = PIN_A0;
static const uint8_t A1 = PIN_A1;
static const uint8_t A2 = PIN_A2;
static const uint8_t A3 = PIN_A3;
static const uint8_t A4 = PIN_A4;
static const uint8_t A5 = PIN_A5;
static const uint8_t A6 = PIN_A6;
static const uint8_t A7 = PIN_A7;
static const uint8_t A8 = PIN_A8;
static const uint8_t A9 = PIN_A9;
static const uint8_t A10 = PIN_A10;
static const uint8_t A11 = PIN_A11;
static const uint8_t A12 = PIN_A12;
static const uint8_t A13 = PIN_A13;

static const uint8_t DAC0 = PIN_DAC0;
#define ADC_RESOLUTION		12

	/*
	* Serial interfaces
	*/
	// Serial1
#define PIN_SERIAL1_TX       (7ul)
#define PIN_SERIAL1_RX       (6ul)
#define PAD_SERIAL1_TX       (UART_TX_PAD_2)
#define PAD_SERIAL1_RX       (SERCOM_RX_PAD_3)

	// Serial2
#define PIN_SERIAL2_TX       (9ul)
#define PIN_SERIAL2_RX       (8ul)
#define PAD_SERIAL2_TX       (UART_TX_PAD_2)
#define PAD_SERIAL2_RX       (SERCOM_RX_PAD_3)

	/*
	* SPI Interfaces
	*/
#define SPI_INTERFACES_COUNT 2

// SPI

#define PIN_SPI_MISO		(25u)
#define PIN_SPI_MOSI		(22u)
#define PIN_SPI_SCK			(23u)
#define PIN_SPI_CS			(24u)

static const uint8_t CS = PIN_SPI_CS;
static const uint8_t SS = PIN_SPI_CS;
static const uint8_t MOSI = PIN_SPI_MOSI;
static const uint8_t MISO = PIN_SPI_MISO;
static const uint8_t SCK = PIN_SPI_SCK;

#define PERIPH_SPI			sercom0
#define SPI_DMA_TX_ID		SERCOM0_DMAC_ID_TX
#define SPI_DMA_RX_ID		SERCOM0_DMAC_ID_RX
#define SPI_SERCOM			SERCOM0
#define SPI_HANDLER			SERCOM0_Handler
#define PORT_SPI_CONF		PIO_SERCOM
#define PAD_SPI_TX			SPI_PAD_0_SCK_1		// DOPO : MOSI SCK SS
#define PAD_SPI_RX			SERCOM_RX_PAD_3		// DIPO : MISO

#define PAD_SPISLAVE_TX		SPI_PAD_3_SCK_1		// DOPO : MISO SCK SS
#define PAD_SPISLAVE_RX		SERCOM_RX_PAD_0		// DIPO : MOSI

// SPI1

#define PIN_SPI1_MOSI		(32u)
#define PIN_SPI1_MISO		(30u)
#define PIN_SPI1_SCK		(33u)
#define PIN_SPI1_CS			(31u)

static const uint8_t CS1 = PIN_SPI1_CS;
static const uint8_t SS1 = PIN_SPI1_CS;
static const uint8_t MOSI1 = PIN_SPI1_MOSI;
static const uint8_t MISO1 = PIN_SPI1_MISO;
static const uint8_t SCK1 = PIN_SPI1_SCK;

#define PERIPH_SPI1			sercom4
#define SPI1_DMA_TX_ID		SERCOM4_DMAC_ID_TX
#define SPI1_DMA_RX_ID		SERCOM4_DMAC_ID_RX
#define SPI1_SERCOM			SERCOM4
#define SPI1_HANDLER		SERCOM4_Handler
#define PORT_SPI1_CONF		PIO_SERCOM_ALT
#define PAD_SPI1_TX			SPI_PAD_2_SCK_3		// DOPO : MOSI SCK SS
#define PAD_SPI1_RX			SERCOM_RX_PAD_0		// DIPO : MISO

#define PAD_SPISLAVE1_TX	SPI_PAD_0_SCK_3		// DOPO : MISO SCK SS
#define PAD_SPISLAVE1_RX	SERCOM_RX_PAD_2		// DIPO : MOSI

/*
* Wire Interfaces
*/
#define WIRE_INTERFACES_COUNT 3

#define PIN_WIRE_SDA         (1u)
#define PIN_WIRE_SCL         (0u)
#define PERIPH_WIRE          sercom5
#define PORT_WIRE_CONF	     PIO_SERCOM_ALT
#define WIRE_IT_HANDLER      SERCOM5_Handler

static const uint8_t SDA = PIN_WIRE_SDA;
static const uint8_t SCL = PIN_WIRE_SCL;

#define PIN_WIRE1_SDA        (30u)
#define PIN_WIRE1_SCL        (31u)
#define PERIPH_WIRE1         sercom2
#define PORT_WIRE1_CONF	     PIO_SERCOM
#define WIRE1_IT_HANDLER     SERCOM2_Handler

static const uint8_t SDA1 = PIN_WIRE1_SDA;
static const uint8_t SCL1 = PIN_WIRE1_SCL;

#define PIN_WIRE2_SDA        (22u)
#define PIN_WIRE2_SCL        (23u)
#define PERIPH_WIRE2         sercom0
#define PORT_WIRE2_CONF	     PIO_SERCOM
#define WIRE2_IT_HANDLER     SERCOM0_Handler

static const uint8_t SDA2 = PIN_WIRE2_SDA;
static const uint8_t SCL2 = PIN_WIRE2_SCL;

	/*
	* USB
	*/
#define PIN_USB_HOST_ENABLE (2ul)
#define PIN_USB_DM          (36ul)
#define PIN_USB_DP          (37ul)

	/*
	* I2S Interfaces
	*/
#define I2S_INTERFACES_COUNT 0

#define I2S_DEVICE          0
#define I2S_CLOCK_GENERATOR 3
#define PIN_I2S_SD          (9u)
#define PIN_I2S_SCK         (1u)
#define PIN_I2S_FS          (0u)

#ifdef __cplusplus
}
#endif

/*----------------------------------------------------------------------------
*        Arduino objects - C++ only
*----------------------------------------------------------------------------*/

#ifdef __cplusplus

/*	=========================
*	===== SERCOM DEFINITION
*	=========================
*/
extern SERCOM sercom0;
extern SERCOM sercom1;
extern SERCOM sercom2;
extern SERCOM sercom3;
extern SERCOM sercom4;
extern SERCOM sercom5;

//extern Uart Serial;
extern Uart Serial1;
extern Uart Serial2;

#endif

#ifdef __cplusplus
extern "C" {
#endif
	unsigned int PINCOUNT_fn();
#ifdef __cplusplus
}
#endif

// These serial port names are intended to allow libraries and architecture-neutral
// sketches to automatically default to the correct port name for a particular type
// of use.  For example, a GPS module would normally connect to SERIAL_PORT_HARDWARE_OPEN,
// the first hardware serial port whose RX/TX pins are not dedicated to another use.
//
// SERIAL_PORT_MONITOR        Port which normally prints to the Arduino Serial Monitor
//
// SERIAL_PORT_USBVIRTUAL     Port which is USB virtual serial
//
// SERIAL_PORT_LINUXBRIDGE    Port which connects to a Linux system via Bridge library
//
// SERIAL_PORT_HARDWARE       Hardware serial port, physical RX & TX pins.
//
// SERIAL_PORT_HARDWARE_OPEN  Hardware serial ports which are open for use.  Their RX & TX
//                            pins are NOT connected to anything by default.
#define SERIAL_PORT_USBVIRTUAL      SerialUSB
#define SERIAL_PORT_MONITOR         Serial
// Serial has no physical pins broken out, so it's not listed as HARDWARE port
#define SERIAL_PORT_HARDWARE        Serial1
#define SERIAL_PORT_HARDWARE_OPEN   Serial1

#endif /* _VARIANT_ES_SAMD21_WB */
