/*
  Copyright (c) 2015 Arduino LLC.  All right reserved.

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

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * \brief SAMD products have only one reference for ADC
 */
 // add internal voltages for ATSAMD51 SUPC VREF register
typedef enum _eAnalogReference
{
  AR_DEFAULT,
  AR_INTERNAL1V0,
  AR_INTERNAL1V1,
  AR_INTERNAL1V2,
  AR_INTERNAL1V25,
  AR_INTERNAL2V0,
  AR_INTERNAL2V2,
  AR_INTERNAL2V23,
  AR_INTERNAL2V4,
  AR_INTERNAL2V5,
  AR_INTERNAL1V65,
  AR_EXTERNAL
} eAnalogReference ;

#define DEFAULT	AR_DEFAULT


/*
 * \brief Configures the reference voltage used for analog input (i.e. the value used as the top of the input range).
 * This function is kept only for compatibility with existing AVR based API.
 *
 * \param ulMmode Should be set to AR_DEFAULT.
 */
extern void analogReference( eAnalogReference ulMode ) ;

extern void initAnalog();

/*
 * \brief Writes an analog value (PWM wave) to a pin.
 *
 * \param ulPin
 * \param ulValue
 */
extern void analogWrite( uint32_t ulPin, uint32_t ulValue ) ;


/*
 * \brief selects PWM frequency for a pin (resolution will be scaled acordingly)
 *
 * \param ulPin
 * \param frequency
 */
extern void analogWriteFrequency( uint32_t ulPin, float frequency);

/*
 * \brief Reads the value from the specified analog pin.
 *
 * \param ulPin
 *
 * \return Read value from selected pin, if no error.
 */
extern uint32_t analogRead( uint32_t ulPin ) ;

extern int16_t analogReadDifferential(uint8_t pin_pos,uint8_t pin_neg);
extern int16_t analogReadDifferentialRaw(uint8_t mux_pos,uint8_t mux_neg);

/*
 * \brief Set the resolution of analogRead return values. Default is 10 bits (range from 0 to 1023).
 *
 * \param res
 */
extern void analogReadResolution(int res);

// sets the gain of the ADC. See page 868. All values defined above. 
void analogGain(uint8_t gain);

// calibrates the bias and linearity based on the nvm register. 
// NVM register access code modified from https://github.com/arduino/ArduinoCore-samd/blob/master/cores/arduino/USB/samd21_host.c
// datasheet pages 32 and 882
void analogCalibrate();

// set the analog reference voltage, but with all available options
// (the Arduino IDE neglects some). The Arduino IDE also changes
// the gain when analogReference() is used, but this won't. pg 861
void analogReference2(uint8_t ref);

// increases accuracy of gain stage by enabling the reference buffer
// offset compensation. Takes longer to start. pg 861
void analogReferenceCompensation(uint8_t val);

// sets the ADC clock relative to the peripheral clock. pg 864
void analogPrescaler(uint8_t val);

// resets the ADC. pg 860
// note that this doesn't put back the default values set by the 
// Arduino IDE. 
void analogReset();

/*
 * \brief Set the resolution of analogWrite parameters. Default is 8 bits (range from 0 to 255).
 *
 * \param res
 */
extern void analogWriteResolution(int res);

extern void analogReadAveraging(uint16_t num);

extern void analogOutputInit( void ) ;

#ifndef ADC_CTRLB_RESSEL_12BIT_Val 
#define ADC_CTRLB_RESSEL_8BIT_Val   0x03
#define ADC_CTRLB_RESSEL_10BIT_Val  0x02 // default by Arduino
#define ADC_CTRLB_RESSEL_12BIT_Val  0x00
#define ADC_CTRLB_RESSEL_16BIT_Val  0x01 // used for averaging mode output
#endif

#define ADC_PIN_TEMP                0x18 // positive mux, pg 870
#define ADC_PIN_BANDGAP             0x19
#define ADC_PIN_SCALEDCOREVCC       0x1A
#define ADC_PIN_SCALEDIOVCC         0x1B
#define ADC_PIN_DAC                 0x1C

#define ADC_PIN_GND                 0x18 // negative mux, pg 869
#define ADC_PIN_IOGND               0x19

#define ADC_GAIN_1                  0x00 // pg 868
#define ADC_GAIN_2                  0x01
#define ADC_GAIN_4                  0x02
#define ADC_GAIN_8                  0x03
#define ADC_GAIN_16                 0x04
#define ADC_GAIN1_DIV2              0x0F // default by Arduino

#define ADC_REF_INT1V               0x00 // 1.0V reference, pg 861
#define ADC_REF_INTVCC0             0x01 // 1/1.48 VDDANA
#define ADC_REF_INTVCC1             0x02 // 1/2 VDDANA (only for VDDANA > 2.0V) // default
#define ADC_REF_VREFA               0x03 // external reference
#define ADC_REF_VREFB               0x04 // external reference

#define ADC_PRESCALER_DIV4          0x00 // pg 864
#define ADC_PRESCALER_DIV8          0x01
#define ADC_PRESCALER_DIV16         0x02
#define ADC_PRESCALER_DIV32         0x03
#define ADC_PRESCALER_DIV64         0x04
#define ADC_PRESCALER_DIV128        0x05
#define ADC_PRESCALER_DIV256        0x06
#define ADC_PRESCALER_DIV512        0x07 // Arduino default

// NVM Software Calibration Area Mapping, pg 32. Address starting at NVMCTRL_OTP4. 
// NVM register access code modified from https://github.com/arduino/ArduinoCore-samd/blob/master/cores/arduino/USB/samd21_host.c
// ADC Linearity Calibration value. Should be written to the CALIB register.
#define NVM_ADC_LINEARITY_POS         27
#define NVM_ADC_LINEARITY_SIZE         8
// ADC Bias Calibration value. Should be written to the CALIB register.
#define NVM_ADC_BIASCAL_POS           35
#define NVM_ADC_BIASCAL_SIZE           3


#ifdef __cplusplus
}
#endif
