#ifndef _VARIANT_TINY_SAMD21_M2
#define _VARIANT_TINY_SAMD21_M2

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
#define NUM_DIGITAL_PINS     (20u)
#define NUM_ANALOG_INPUTS    (6u)
#define NUM_ANALOG_OUTPUTS   (1u)
#define analogInputToDigitalPin(p)  ((p < 6u) ? (p) + 14u : -1)

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

static const uint8_t DAC0 = PIN_DAC0;

#define ADC_RESOLUTION		12

/*
* Serial interfaces
*/
// Serial1
#define PIN_SERIAL1_RX       (1ul)
#define PIN_SERIAL1_TX       (0ul)
#define PAD_SERIAL1_TX       (UART_TX_PAD_2)
#define PAD_SERIAL1_RX       (SERCOM_RX_PAD_3)

// Serial2
#define PIN_SERIAL2_RX       (23ul)
#define PIN_SERIAL2_TX       (24ul)
#define PAD_SERIAL2_TX       (UART_TX_PAD_2)
#define PAD_SERIAL2_RX       (SERCOM_RX_PAD_3)

// Serial3
#define PIN_SERIAL3_RX       (3ul)
#define PIN_SERIAL3_TX       (2ul)
#define PAD_SERIAL3_TX       (UART_TX_PAD_0)
#define PAD_SERIAL3_RX       (SERCOM_RX_PAD_1)

/*
* SPI Interfaces
*/
#define SPI_INTERFACES_COUNT 1

// SPI

#define PIN_SPI_MISO         PIN_A9
#define PIN_SPI_MOSI         PIN_A12
#define PIN_SPI_SCK          PIN_A11
#define PIN_SPI_SS           PIN_A10

static const uint8_t SS = PIN_SPI_SS;
static const uint8_t MOSI = PIN_SPI_MOSI;
static const uint8_t MISO = PIN_SPI_MISO;
static const uint8_t SCK = PIN_SPI_SCK;

#define PERIPH_SPI           sercom2
#define PORT_SPI_CONF	     PIO_SERCOM_ALT
#define PAD_SPI_TX           SPI_PAD_0_SCK_1		// DOPO : MOSI SCK SS
#define PAD_SPI_RX           SERCOM_RX_PAD_3		// DIPO : MISO

#define PAD_SPISLAVE_TX      SPI_PAD_3_SCK_1		// DOPO : MISO SCK SS
#define PAD_SPISLAVE_RX      SERCOM_RX_PAD_0		// DIPO : MOSI

	/*
	* Wire Interfaces
	*/
#define WIRE_INTERFACES_COUNT 2

#define PIN_WIRE_SDA         (2u)
#define PIN_WIRE_SCL         (3u)
#define PERIPH_WIRE          sercom3
#define PORT_WIRE_CONF	     PIO_SERCOM
#define WIRE_IT_HANDLER      SERCOM3_Handler

static const uint8_t SDA = PIN_WIRE_SDA;
static const uint8_t SCL = PIN_WIRE_SCL;

#define PIN_WIRE1_SDA        (26u)
#define PIN_WIRE1_SCL        (25u)
#define PERIPH_WIRE1         sercom2
#define PORT_WIRE1_CONF	     PIO_SERCOM_ALT
#define WIRE1_IT_HANDLER     SERCOM2_Handler

static const uint8_t SDA1 = PIN_WIRE1_SDA;
static const uint8_t SCL1 = PIN_WIRE1_SCL;

	/*
	* USB
	*/
#define PIN_USB_HOST_ENABLE (27ul)
#define PIN_USB_DM          (28ul)
#define PIN_USB_DP          (29ul)

	/*
	* I2S Interfaces
	*/
#define I2S_INTERFACES_COUNT 1

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

#endif /*_VARIANT_TINY_SAMD21_M2 */
