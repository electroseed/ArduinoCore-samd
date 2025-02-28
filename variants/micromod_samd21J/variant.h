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

#ifndef _VARIANT_MICROMOD_SAMD21
#define _VARIANT_MICROMOD_SAMD21

// The definitions here needs a SAMD core >=1.6.10
#define ARDUINO_SAMD_VARIANT_COMPLIANCE 10610

/*----------------------------------------------------------------------------
 *        Definitions
 *----------------------------------------------------------------------------*/

/** Frequency of the board main oscillator */
#define VARIANT_MAINOSC		(32768ul)

/** Master clock frequency */
#define VARIANT_MCK	(F_CPU)

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
#define NUM_DIGITAL_PINS     (50u)
#define NUM_ANALOG_INPUTS    (20u)
#define NUM_ANALOG_OUTPUTS   (1u)
#define analogInputToDigitalPin(p)  ((p < NUM_ANALOG_INPUTS) ? (p) + 14u : -1)

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

#define M4		(23ul)	// A9 / 3V3 EN
#define M8		(45ul)	// G11
#define M10		(26ul)	// A12 / D0
#define M12		(9ul)	// SDA
#define M11		(36ul)	// RTS1
#define M13		(36ul)	// RTS1
#define M14		(10ul)	// SCL
#define M15		(35ul)	// CTS1
#define M16		(6ul)	// I2C INT
#define M17		(7ul)	// TX1
#define M18		(22ul)	// A8 / D1
#define M19		(8ul)	// RX1
#define M20		(40ul)	// RX2
#define M21		(37ul)	// SWDCK
#define M22		(39ul)	// TX2
#define M23		(38ul)	// SWDIO
#define M32		(49ul)	// PWM0
#define M34		(14ul)	// A0
#define M38		(15ul)	// A1
#define M40		(20ul)	// A6 / G0
#define M41		(24ul)	// A10 / CAN RX
#define M42		(19ul)	// A5 / G1
#define M43		(25ul)	// A13 / CAN TX
#define M44		(27ul)	// A13 / G2
#define M46		(28ul)	// A14 / G3
#define M47		(48ul)	// PWM1
#define M48		(0ul)	// G4
#define M49		(16ul)	// A2 / BATT VIN/3
#define M50		(34ul)	// AUD_BCLK
#define M51		(12ul)	// SCK1
#define M52		(13ul)	// G11 / Led
#define M53		(11ul)	// MOSI1
#define M54		(47ul)	// AUD IN
#define M55		(29ul)	// A15 / CS
#define M56		(44ul)	// AUD OUT
#define M57		(17ul)	// A3 / SCK
#define M58		(43ul)	// AUD MCLK
#define M59		(21ul)	// A7 / MOSI
#define M60		(42ul)	// SDIO SCK
#define M61		(18ul)	// A4 / MISO
#define M62		(41ul)	// SDIO CMD
#define M63		(3ul)	// G10
#define M64		(33ul)	// A19 / SDIO DATA0
#define M65		(5ul)	// MISO1 / G9
#define M66		(32ul)	// A18 / SDIO DATA1
#define M67		(4ul)	// CS1 / G8
#define M68		(31ul)	// A17 / SDIO DATA2
#define M69		(46ul)	// A14 / G7
#define M70		(30ul)	// A16 / SDIO DATA3
#define M71		(2ul)	// G6
#define M73		(1ul)	// G5

	// LEDs
#define PIN_LED_13           (13u)
#define PIN_LED              PIN_LED_13
#define LED_BUILTIN          PIN_LED_13

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
#define PIN_A14              (PIN_A0 + 14)
#define PIN_A15              (PIN_A0 + 15)
#define PIN_A16              (PIN_A0 + 16)
#define PIN_A17              (PIN_A0 + 17)
#define PIN_A18              (PIN_A0 + 18)
#define PIN_A19              (PIN_A0 + 19)

#define PIN_DAC0             PIN_A0

static const uint8_t A0  = PIN_A0;
static const uint8_t A1  = PIN_A1;
static const uint8_t A2  = PIN_A2;
static const uint8_t A3  = PIN_A3;
static const uint8_t A4  = PIN_A4;
static const uint8_t A5  = PIN_A5;
static const uint8_t A6  = PIN_A6;
static const uint8_t A7  = PIN_A7;
static const uint8_t A8  = PIN_A8;
static const uint8_t A9  = PIN_A9;
static const uint8_t A10 = PIN_A10;
static const uint8_t A12 = PIN_A12;
static const uint8_t A13 = PIN_A13;
static const uint8_t A14 = PIN_A14;
static const uint8_t A15 = PIN_A15;
static const uint8_t A16 = PIN_A16;
static const uint8_t A17 = PIN_A17;
static const uint8_t A18 = PIN_A18;
static const uint8_t A19 = PIN_A19;

static const uint8_t DAC0 = PIN_DAC0;
#define ADC_RESOLUTION		12

/*
* Serial interfaces
*/
// Serial1
#define PIN_SERIAL1_RX       (8ul)
#define PIN_SERIAL1_TX       (7ul)
#define PAD_SERIAL1_TX       (UART_TX_PAD_2)
#define PAD_SERIAL1_RX       (SERCOM_RX_PAD_3)

// Serial2
#define PIN_SERIAL2_RX       (10ul)
#define PIN_SERIAL2_TX       (9ul)
#define PAD_SERIAL2_TX       (UART_TX_PAD_2)
#define PAD_SERIAL2_RX       (SERCOM_RX_PAD_3)

// Serial3
#define PIN_SERIAL3_RX       (31ul)
#define PIN_SERIAL3_TX       (30ul)
#define PAD_SERIAL3_TX       (UART_TX_PAD_2)
#define PAD_SERIAL3_RX       (SERCOM_RX_PAD_0)

/*
* SPI Interfaces
*/
#define SPI_INTERFACES_COUNT 2

// SPI
//-------------------------------------------
// SPI
#define PIN_SPI_MISO		M61
#define PIN_SPI_MOSI		M59
#define PIN_SPI_SCK			M57
#define PIN_SPI_CS			M55

static const uint8_t SS = PIN_SPI_CS;
static const uint8_t CS = PIN_SPI_CS;
static const uint8_t MOSI = PIN_SPI_MOSI;
static const uint8_t MISO = PIN_SPI_MISO;
static const uint8_t SCK = PIN_SPI_SCK;

#define PERIPH_SPI			sercom2
#define SPI_DMA_TX_ID		SERCOM2_DMAC_ID_TX
#define SPI_DMA_RX_ID		SERCOM2_DMAC_ID_RX
#define SPI_SERCOM			SERCOM2
#define PORT_SPI_CONF		PIO_SERCOM_ALT
#define SPI_HANDLER			SERCOM2_Handler
#define PAD_SPI_TX			SPI_PAD_0_SCK_1		// DOPO : MOSI SCK SS
#define PAD_SPI_RX			SERCOM_RX_PAD_3		// DIPO : MISO

#define PAD_SPISLAVE_TX		SPI_PAD_3_SCK_1		// DOPO : MISO SCK SS
#define PAD_SPISLAVE_RX		SERCOM_RX_PAD_0		// DIPO : MOSI

// SPI1

#define PIN_SPI1_MISO	M65	// 5		PA12	PAD0
#define PIN_SPI1_MOSI	M53	// 11		PA14	PAD2
#define PIN_SPI1_SCK	M51	// 12		PA15	PAD3
#define PIN_SPI1_CS		M67	// 4		PA13	PAD1

static const uint8_t CS1 = PIN_SPI1_CS;
static const uint8_t SS1 = PIN_SPI1_CS;
static const uint8_t MOSI1 = PIN_SPI1_MOSI;
static const uint8_t MISO1 = PIN_SPI1_MISO;
static const uint8_t SCK1 = PIN_SPI1_SCK;

#define PERIPH_SPI1			sercom4
#define SPI1_DMA_TX_ID		SERCOM4_DMAC_ID_TX
#define SPI1_DMA_RX_ID		SERCOM4_DMAC_ID_RX
#define SPI1_SERCOM			SERCOM4
#define PORT_SPI1_CONF		PIO_SERCOM_ALT
#define SPI1_HANDLER		SERCOM4_Handler
#define PAD_SPI1_TX			SPI_PAD_2_SCK_3		// DOPO : MOSI SCK SS
#define PAD_SPI1_RX			SERCOM_RX_PAD_0		// DIPO : MISO

#define PAD_SPISLAVE1_TX	SPI_PAD_0_SCK_3		// DOPO : MISO SCK SS
#define PAD_SPISLAVE1_RX	SERCOM_RX_PAD_2		// DIPO : MOSI

// Wire Interfaces
#define WIRE_INTERFACES_COUNT 1

#define PIN_WIRE_SDA         M12 		//9
#define PIN_WIRE_SCL         M14		//10
#define PERIPH_WIRE          sercom3
#define PORT_WIRE_CONF	     PIO_SERCOM
#define WIRE_IT_HANDLER      SERCOM3_Handler

	static const uint8_t SDA = PIN_WIRE_SDA;
	static const uint8_t SCL = PIN_WIRE_SCL;

	/*
	* USB
	*/
#define PIN_USB_HOST_ENABLE (6ul)
#define PIN_USB_DM          (50ul)
#define PIN_USB_DP          (51ul)

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
extern Uart Serial3;

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
#define SERIAL_PORT_MONITOR         SerialUSB
#define SERIAL_PORT_HARDWARE        Serial1
#define SERIAL_PORT_HARDWARE_OPEN   Serial1

// Alias Serial to SerialUSB
//#define Serial                      SerialUSB

#endif /* _VARIANT_MICROMOD_SAMD21 */
