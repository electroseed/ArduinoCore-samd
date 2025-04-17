/*
  Copyright (c) 2014 Arduino LLC.  All right reserved.
  SAMD51 support added by Adafruit - Copyright (c) 2018 Dean Miller for Adafruit Industries

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

#include "Arduino.h"
#include "wiring_private.h"

#ifdef __cplusplus
extern "C" {
#endif

static int _readResolution = 0;					// Force first time settings
 int _ADCResolution = 12;
static uint16_t num_averages = 0xffff;			// Force first time settings
 uint16_t num_averages_pow2 = 0;
uint16_t pwm_per[8];
uint16_t pwm_val[8];
static bool tcEnabled[TCC_INST_NUM+TC_INST_NUM];

#if defined(__SAMD51__)
static int _writeResolution = 12;
static int _dacResolution = 12;
#else
static int _writeResolution = 8;
//static int _dacResolution = 10;
#endif

void initAnalog()
{
	for (uint8_t i = 0; i < 8; i++)
		pwm_per[i] = 0xFFFF;
	for (uint8_t i = 0; i < TCC_INST_NUM+TC_INST_NUM; i++)
		tcEnabled[i] = false;
	analogReadAveraging(1);
}

static __inline__ void syncDAC() __attribute__((always_inline, unused));
// Wait for synchronization of registers between the clock domains
static void syncADC() 
{
#if defined(__SAMD51__)
	while(ADC0->SYNCBUSY.reg & ADC_SYNCBUSY_REFCTRL); //wait for sync
	while(ADC1->SYNCBUSY.reg & ADC_SYNCBUSY_REFCTRL); //wait for sync
#else
  while (ADC->STATUS.bit.SYNCBUSY == 1);
#endif
}

 // ATSAMR, for example, doesn't have a DAC
#ifdef DAC
// Wait for synchronization of registers between the clock domains
static __inline__ void syncDAC() __attribute__((always_inline, unused));
static void syncDAC()
{
#if defined(__SAMD51__)
	while (DAC->SYNCBUSY.bit.ENABLE || DAC->SYNCBUSY.bit.SWRST);
#else
	while (DAC->STATUS.bit.SYNCBUSY == 1);
#endif
}

#if !defined(__SAMD51__)
// Wait for synchronization of registers between the clock domains
static __inline__ void syncTC_16(Tc* TCx) __attribute__((always_inline, unused));
static void syncTC_16(Tc* TCx) {
  while (TCx->COUNT16.STATUS.bit.SYNCBUSY);
}

// Wait for synchronization of registers between the clock domains
static __inline__ void syncTCC(Tcc* TCCx) __attribute__((always_inline, unused));
static void syncTCC(Tcc* TCCx) {
  while (TCCx->SYNCBUSY.reg & TCC_SYNCBUSY_MASK);
}
#endif
#endif

#ifdef DAC
#if defined(__SAMD51__)
static bool dacEnabled[2];
#endif
#endif

static uint8_t res_table[] = {
	8,ADC_CTRLB_RESSEL_8BIT_Val,0,0,	// 8
	10,ADC_CTRLB_RESSEL_10BIT_Val,0,0,	// 9
	10,ADC_CTRLB_RESSEL_10BIT_Val,0,0,	// 10
	12,ADC_CTRLB_RESSEL_12BIT_Val,0,0,	// 11
	12,ADC_CTRLB_RESSEL_12BIT_Val,0,0,	// 12
	13,ADC_CTRLB_RESSEL_16BIT_Val,0,1,	// 13
	14,ADC_CTRLB_RESSEL_16BIT_Val,0,2,	// 14
	16,ADC_CTRLB_RESSEL_16BIT_Val,0,3,	// 15
	16,ADC_CTRLB_RESSEL_16BIT_Val,0,4	// 16
};
typedef struct _res_info
{
	uint8_t mReadBits;
	uint8_t mRessel;
	uint8_t mAdjustRes;
	uint8_t mNumAvg;
} resInfo;
#define getResolutionInfo(m_res)		(resInfo*)&res_table[(m_res - 8) * 4]

void setAnalogReadResolution(int res)
{
	if (res < 8)
		res = 8;
	_readResolution = res;
	resInfo *res_info = getResolutionInfo(res);
	_ADCResolution = res_info->mReadBits + num_averages_pow2;
	uint8_t adj_res = res_info->mAdjustRes;// + min(num_averages_pow2,4);
	if (_ADCResolution > 16)
	{
		int diff = _ADCResolution - 16;
		_ADCResolution = 16;
//		adj_res += diff;
	}
	uint8_t ressel = res_info->mRessel;
	if (num_averages_pow2 > 0)
		ressel = ADC_CTRLB_RESSEL_16BIT_Val;
#if defined(__SAMD51__)
	ADC0->CTRLB.bit.RESSEL = ressel;
	ADC1->CTRLB.bit.RESSEL = ressel;
	ADC0->AVGCTRL.bit.ADJRES = adj_res;
	ADC1->AVGCTRL.bit.ADJRES = adj_res;
	ADC0->AVGCTRL.bit.SAMPLENUM = num_averages_pow2 + res_info->mNumAvg;
	ADC1->AVGCTRL.bit.SAMPLENUM = num_averages_pow2 + res_info->mNumAvg;
#else
	ADC->CTRLB.bit.RESSEL = ressel;
	ADC->AVGCTRL.bit.ADJRES = adj_res;
	ADC->AVGCTRL.bit.SAMPLENUM = num_averages_pow2 + res_info->mNumAvg;
#endif
	syncADC();
}

void analogReadResolution(int res)
{
	if (_readResolution == res)
		return;
	setAnalogReadResolution(res);
}

uint16_t FastLog2(uint16_t val)
{
    uint16_t result = 0;
    uint16_t step = 8;
	uint16_t mask = 0xff00;

    while ( val && step )
    {
      if ( val & mask ) { result += step ;  val >>= step ; }	// In the upper half bits
      mask >>= ( step + 1) >> 1 ;
      step >>= 1 ;
    }
    return result;
}

void analogReadAveraging(uint16_t num)
{
	if (num_averages == num)
		return;
	num_averages = num;
	num_averages_pow2 = FastLog2(num);
	setAnalogReadResolution(_readResolution);
}

void analogWriteResolution(int res)
{
	_writeResolution = res;
}

static inline uint32_t mapResolution(uint32_t value, uint32_t from, uint32_t to)
{
  if (from == to) {
    return value;
  }
  if (from > to) {
    return value >> (from-to);
  }
  return value << (to-from);
}

/*
 * Internal Reference is at 1.0v
 * External Reference should be between 1v and VDDANA-0.6v=2.7v
 *
 * Warning : On Arduino Zero board the input/output voltage for SAMD21G18 is 3.3 volts maximum
 */
void analogReference(eAnalogReference mode)
{
	syncADC();
#if defined(__SAMD51__)	
	//TODO: fix gains
	switch (mode)
	{
		case AR_INTERNAL1V0:
		//ADC0->GAINCORR.reg = ADC_GAINCORR_GAINCORR();      // Gain Factor Selection
		SUPC->VREF.bit.SEL = SUPC_VREF_SEL_1V0_Val;		// select 1.0V
		SUPC->VREF.bit.VREFOE = 1;	//	Turn on for use with ADC
		ADC0->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INTREF_Val; // Use SUPC.VREF
		ADC1->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INTREF_Val; // 
		break;
		
		case AR_INTERNAL1V1:
		//ADC0->GAINCORR.reg = ADC_GAINCORR_GAINCORR();      // Gain Factor Selection
		SUPC->VREF.bit.SEL = SUPC_VREF_SEL_1V1_Val;		// select 1.1V
		SUPC->VREF.bit.VREFOE = 1;	//	Turn on for use with ADC
		ADC0->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INTREF_Val; // Use SUPC.VREF
		ADC1->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INTREF_Val; // 
		break;
		
		case AR_INTERNAL1V2:
		//ADC0->GAINCORR.reg = ADC_GAINCORR_GAINCORR();      // Gain Factor Selection
		SUPC->VREF.bit.SEL = SUPC_VREF_SEL_1V2_Val;		// select 1V2
		SUPC->VREF.bit.VREFOE = 1;	//	Turn on for use with ADC
		ADC0->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INTREF_Val; // Use SUPC.VREF
		ADC1->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INTREF_Val; // 
		break;

		case AR_INTERNAL1V25:
		//ADC0->GAINCORR.reg = ADC_GAINCORR_GAINCORR();      // Gain Factor Selection
		SUPC->VREF.bit.SEL = SUPC_VREF_SEL_1V25_Val;		// select 1.25V
		SUPC->VREF.bit.VREFOE = 1;	//	Turn on for use with ADC
		ADC0->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INTREF_Val; // Use SUPC.VREF
		ADC1->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INTREF_Val; // 
		break;
		
		case AR_INTERNAL2V0:
		//ADC0->GAINCORR.reg = ADC_GAINCORR_GAINCORR();      // Gain Factor Selection
		SUPC->VREF.bit.SEL = SUPC_VREF_SEL_2V0_Val;		// select 2.0V
		SUPC->VREF.bit.VREFOE = 1;	//	Turn on for use with ADC
		ADC0->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INTREF_Val; // Use SUPC.VREF
		ADC1->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INTREF_Val; // 
		break;
		
		case AR_INTERNAL2V2:
		//ADC0->GAINCORR.reg = ADC_GAINCORR_GAINCORR();      // Gain Factor Selection
		SUPC->VREF.bit.SEL = SUPC_VREF_SEL_2V2_Val;		// select 2.2V
		SUPC->VREF.bit.VREFOE = 1;	//	Turn on for use with ADC
		ADC0->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INTREF_Val; // Use SUPC.VREF
		ADC1->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INTREF_Val; // 
		break;
		
		case AR_INTERNAL2V4:
		//ADC0->GAINCORR.reg = ADC_GAINCORR_GAINCORR();      // Gain Factor Selection
		SUPC->VREF.bit.SEL = SUPC_VREF_SEL_2V4_Val;		// select 2.4V
		SUPC->VREF.bit.VREFOE = 1;	//	Turn on for use with ADC
		ADC0->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INTREF_Val; // Use SUPC.VREF
		ADC1->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INTREF_Val; // 
		break;
		
		case AR_INTERNAL2V5:
		//ADC0->GAINCORR.reg = ADC_GAINCORR_GAINCORR();      // Gain Factor Selection
		SUPC->VREF.bit.SEL = SUPC_VREF_SEL_2V5_Val;		// select 2.5V
		SUPC->VREF.bit.VREFOE = 1;	//	Turn on for use with ADC
		ADC0->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INTREF_Val; // Use SUPC.VREF
		ADC1->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INTREF_Val; // 
		break;
		
		case AR_EXTERNAL:
		//ADC0->INPUTCTRL.bit.GAIN = ADC_INPUTCTRL_GAIN_1X_Val;      // Gain Factor Selection
		ADC0->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_AREFA_Val;	// AREF is jumpered to VCC, so 3.3V
		ADC1->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_AREFA_Val;
		break;

		case AR_INTERNAL1V65:
		//ADC0->INPUTCTRL.bit.GAIN = ADC_INPUTCTRL_GAIN_DIV2_Val;
		ADC0->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INTVCC0_Val; // 1/2 VDDANA = 1.65
		ADC1->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INTVCC0_Val; // 
		break;
		
		case AR_DEFAULT:
		default:
		//ADC0->INPUTCTRL.bit.GAIN = ADC_INPUTCTRL_GAIN_DIV2_Val;
		ADC0->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INTVCC1_Val; // VDDANA = 3V3
		ADC1->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INTVCC1_Val; // 
		break;
	}
	
#else
  switch (mode)
  {
    case AR_INTERNAL2V23:
      ADC->INPUTCTRL.bit.GAIN = ADC_INPUTCTRL_GAIN_1X_Val;      // Gain Factor Selection
      ADC->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INTVCC0_Val; // 1/1.48 VDDANA = 1/1.48* 3V3 = 2.2297
      break;

    case AR_EXTERNAL:
      ADC->INPUTCTRL.bit.GAIN = ADC_INPUTCTRL_GAIN_1X_Val;      // Gain Factor Selection
      ADC->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_AREFA_Val;
      break;

    case AR_INTERNAL1V0:
      ADC->INPUTCTRL.bit.GAIN = ADC_INPUTCTRL_GAIN_1X_Val;      // Gain Factor Selection
      ADC->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INT1V_Val;   // 1.0V voltage reference
      break;

    case AR_INTERNAL1V65:
      ADC->INPUTCTRL.bit.GAIN = ADC_INPUTCTRL_GAIN_1X_Val;      // Gain Factor Selection
      ADC->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INTVCC1_Val; // 1/2 VDDANA = 0.5* 3V3 = 1.65V
      break;

    case AR_DEFAULT:
    default:
      ADC->INPUTCTRL.bit.GAIN = ADC_INPUTCTRL_GAIN_DIV2_Val;
      ADC->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INTVCC1_Val; // 1/2 VDDANA = 0.5* 3V3 = 1.65V
      break;
  }
#endif
}

void analogGain(uint8_t gain) 
{
#if defined(__SAMD51__)
	// No gain in Samd51
#else
// sets the gain of the ADC. See page 868. All values defined above. 
	syncADC();
	ADC->INPUTCTRL.bit.GAIN = gain;
	syncADC();
#endif
}

// calibrates the bias and linearity based on the nvm register. 
// NVM register access code modified from https://github.com/arduino/ArduinoCore-samd/blob/master/cores/arduino/USB/samd21_host.c
// datasheet pages 32 and 882
void analogCalibrate() 
{
	syncADC();
#if defined(__SAMD51__)
	// TODO
#else
	// read NVM register
	uint32_t adc_linearity = (*((uint32_t *)(NVMCTRL_OTP4) // original position
		+(NVM_ADC_LINEARITY_POS / 32)) // move to the correct 32 bit window, read value
		>> (NVM_ADC_LINEARITY_POS % 32)) // shift value to match the desired position
		& ((1 << NVM_ADC_LINEARITY_SIZE) - 1); // apply a bitmask for the desired size

	uint32_t adc_biascal = (*((uint32_t *)(NVMCTRL_OTP4)
		+(NVM_ADC_BIASCAL_POS / 32))
		>> (NVM_ADC_BIASCAL_POS % 32))
		& ((1 << NVM_ADC_BIASCAL_SIZE) - 1);

	// write values to CALIB register
	ADC->CALIB.bit.LINEARITY_CAL = adc_linearity;
	ADC->CALIB.bit.BIAS_CAL = adc_biascal;
	syncADC();
#endif
}

// set the analog reference voltage, but with all available options
// (the Arduino IDE neglects some). The Arduino IDE also changes
// the gain when analogReference() is used, but this won't. pg 861
void analogReference2(uint8_t ref) 
{
	syncADC();
#if defined(__SAMD51__)
	ADC0->REFCTRL.bit.REFSEL = ref;
	ADC1->REFCTRL.bit.REFSEL = ref;
#else
	ADC->REFCTRL.bit.REFSEL = ref;
#endif
	syncADC();
}

// increases accuracy of gain stage by enabling the reference buffer
// offset compensation. Takes longer to start. pg 861
void analogReferenceCompensation(uint8_t val) 
{
	if (val > 0) val = 1;
	syncADC();
#if defined(__SAMD51__)
	ADC0->REFCTRL.bit.REFCOMP = val;
	ADC1->REFCTRL.bit.REFCOMP = val;
#else
	ADC->REFCTRL.bit.REFCOMP = val;
#endif
	syncADC();
}

// sets the ADC clock relative to the peripheral clock. pg 864
void analogPrescaler(uint8_t val) 
{
	syncADC();
#if defined(__SAMD51__)
	ADC0->CTRLA.bit.PRESCALER = val;
	ADC1->CTRLA.bit.PRESCALER = val;
#else
	ADC->CTRLB.bit.PRESCALER = val;
#endif
	syncADC();
}

// resets the ADC. pg 860
// note that this doesn't put back the default values set by the 
// Arduino IDE. 
void analogReset() 
{
	syncADC();
#if defined(__SAMD51__)
	ADC0->CTRLA.bit.SWRST = 1; // set reset bit
	while (ADC0->CTRLA.bit.SWRST == 1); // wait until it's finished
	ADC1->CTRLA.bit.SWRST = 1; // set reset bit
	while (ADC1->CTRLA.bit.SWRST == 1); // wait until it's finished
#else
	ADC->CTRLA.bit.SWRST = 1; // set reset bit
	while (ADC->CTRLA.bit.SWRST == 1); // wait until it's finished
#endif
	syncADC();
}

uint32_t analogRead(uint32_t pin)
{
  uint32_t valueRead = 0;
  pinPeripheral(pin, PIO_ANALOG);

#ifdef DAC								 //ATSAMR, for example, doesn't have a DAC
	#if defined(__SAMD51__)
		if (pin == PIN_DAC0 || pin == PIN_DAC1) // Disable DAC, if analogWrite(A0,dval) used previously the DAC is enabled
		{
			uint8_t channel = (pin == PIN_DAC0 ? 0 : 1);

			if(dacEnabled[channel])
			{
				dacEnabled[channel] = false;
				syncDAC();
				DAC->CTRLA.bit.ENABLE = 0;     // disable DAC
				syncDAC();
				DAC->DACCTRL[channel].bit.ENABLE = 0;
				syncDAC();
				DAC->CTRLA.bit.ENABLE = 1;     // enable DAC
			}
			while (DAC->SYNCBUSY.bit.ENABLE);
		}
	#else
		if (pin == PIN_DAC0) // Disable DAC, if analogWrite(A0,dval) used previously the DAC is enabled
		{
			syncDAC();
			DAC->CTRLA.bit.ENABLE = 0x00; // Disable DAC
			//DAC->CTRLB.bit.EOEN = 0x00; // The DAC output is turned off.
			syncDAC();
		}
	#endif
#endif

#if defined(__SAMD51__)
  Adc *adc;
  if(g_APinDescription[pin].ulPinAttribute & PIN_ATTR_ANALOG) adc = ADC0;
  else if(g_APinDescription[pin].ulPinAttribute & PIN_ATTR_ANALOG_ALT) adc = ADC1;
  else return 0;

  while( adc->SYNCBUSY.reg & ADC_SYNCBUSY_INPUTCTRL ); //wait for sync
  adc->INPUTCTRL.bit.MUXPOS = g_APinDescription[pin].ulADCChannelNumber; // Selection for the positive ADC input
  
  // Control A
  /*
   * Bit 1 ENABLE: Enable
   *   0: The ADC is disabled.
   *   1: The ADC is enabled.
   * Due to synchronization, there is a delay from writing CTRLA.ENABLE until the peripheral is enabled/disabled. The
   * value written to CTRL.ENABLE will read back immediately and the Synchronization Busy bit in the Status register
   * (STATUS.SYNCBUSY) will be set. STATUS.SYNCBUSY will be cleared when the operation is complete.
   *
   * Before enabling the ADC, the asynchronous clock source must be selected and enabled, and the ADC reference must be
   * configured. The first conversion after the reference is changed must not be used.
   */
  while( adc->SYNCBUSY.reg & ADC_SYNCBUSY_ENABLE ); //wait for sync
  adc->CTRLA.bit.ENABLE = 0x01;             // Enable ADC

  // Start conversion
  while( adc->SYNCBUSY.reg & ADC_SYNCBUSY_ENABLE ); //wait for sync
  
  adc->SWTRIG.bit.START = 1;

  // Clear the Data Ready flag
  adc->INTFLAG.reg = ADC_INTFLAG_RESRDY;

  // Start conversion again, since The first conversion after the reference is changed must not be used.
  adc->SWTRIG.bit.START = 1;

  // Store the value
  while (adc->INTFLAG.bit.RESRDY == 0);   // Waiting for conversion to complete
  valueRead = adc->RESULT.reg;

  while( adc->SYNCBUSY.reg & ADC_SYNCBUSY_ENABLE ); //wait for sync
  adc->CTRLA.bit.ENABLE = 0x00;             // Disable ADC
  while( adc->SYNCBUSY.reg & ADC_SYNCBUSY_ENABLE ); //wait for sync
  
#else
  syncADC();
  ADC->INPUTCTRL.bit.MUXPOS = g_APinDescription[pin].ulADCChannelNumber; // Selection for the positive ADC input
  
  // Control A
  /*
   * Bit 1 ENABLE: Enable
   *   0: The ADC is disabled.
   *   1: The ADC is enabled.
   * Due to synchronization, there is a delay from writing CTRLA.ENABLE until the peripheral is enabled/disabled. The
   * value written to CTRL.ENABLE will read back immediately and the Synchronization Busy bit in the Status register
   * (STATUS.SYNCBUSY) will be set. STATUS.SYNCBUSY will be cleared when the operation is complete.
   *
   * Before enabling the ADC, the asynchronous clock source must be selected and enabled, and the ADC reference must be
   * configured. The first conversion after the reference is changed must not be used.
   */
  syncADC();
  ADC->CTRLA.bit.ENABLE = 0x01;             // Enable ADC

  // Start conversion
  syncADC();
  ADC->SWTRIG.bit.START = 1;
  syncADC();
  // Clear the Data Ready flag
  ADC->INTFLAG.reg = ADC_INTFLAG_RESRDY;

  // Start conversion again, since The first conversion after the reference is changed must not be used.
  syncADC();
  ADC->SWTRIG.bit.START = 1;

  // Store the value
  while (ADC->INTFLAG.bit.RESRDY == 0);   // Waiting for conversion to complete
  valueRead = ADC->RESULT.reg;

  syncADC();
  ADC->CTRLA.bit.ENABLE = 0x00;             // Disable ADC
  syncADC();
#endif

  return mapResolution(valueRead, _ADCResolution,_readResolution);
}

#if defined(__SAMD21__)
// modified from Arduino analogRead, can be used in conjunction with analogRead:
int16_t analogReadDifferential(uint8_t pin_pos,uint8_t pin_neg) 
{
	if (pin_pos < A0) pin_pos += A0;
	if (pin_neg < A0) pin_neg += A0;

	if ((g_APinDescription[pin_neg].ulADCChannelNumber>0x07) && (pin_neg < ADC_PIN_GND)) { // if the negative pin is out of bounds
		return 0;
	}

	uint32_t value_read = 0;

	pinPeripheral(pin_pos,PIO_ANALOG); // set pins to analog mode
	pinPeripheral(pin_neg,PIO_ANALOG);

	if ((pin_pos == A0) || (pin_neg == A0)) { // Disable DAC
		syncDAC();
		DAC->CTRLA.bit.ENABLE = 0x00; // Disable DAC
		syncDAC();
	}

	syncADC();
	ADC->INPUTCTRL.bit.MUXPOS = g_APinDescription[pin_pos].ulADCChannelNumber; // Selection for the positive ADC input
	ADC->INPUTCTRL.bit.MUXNEG = g_APinDescription[pin_neg].ulADCChannelNumber; // negative ADC input

	syncADC();
	ADC->CTRLA.bit.ENABLE = 0x01; // enable adc
	ADC->CTRLB.bit.DIFFMODE = 1; // set to differential mode

	syncADC();
	ADC->SWTRIG.bit.START = 1; // start conversion

	ADC->INTFLAG.reg = ADC_INTFLAG_RESRDY; // clear the data ready flag
	syncADC();

	ADC->SWTRIG.bit.START = 1; // restart conversion, as changing inputs messes up first conversion

	while (ADC->INTFLAG.bit.RESRDY == 0);   // Wait for conversion to complete
	value_read = ADC->RESULT.reg; // read the value

	syncADC();
	ADC->CTRLA.bit.ENABLE = 0x00; // disable adc
	ADC->CTRLB.bit.DIFFMODE = 0; // put back into single-ended mode
	ADC->INPUTCTRL.bit.MUXNEG = ADC_PIN_GND; // set back muxneg to internal ground
	syncADC();

	return value_read;
}
#endif

#if defined(__SAMD21__)
// same as the above function, but no error checking, no pin types are changed, and the positive and negative
// inputs are the raw values being input. The DAC is not automatically shut off either. See datasheet page
int16_t analogReadDifferentialRaw(uint8_t mux_pos,uint8_t mux_neg)
{
	uint32_t value_read = 0;

	syncADC();
	ADC->INPUTCTRL.bit.MUXPOS = mux_pos; // Selection for the positive ADC input
	ADC->INPUTCTRL.bit.MUXNEG = mux_neg; // negative ADC input

	syncADC();
	ADC->CTRLA.bit.ENABLE = 0x01; // enable adc
	ADC->CTRLB.bit.DIFFMODE = 1; // set to differential mode

	syncADC();
	ADC->SWTRIG.bit.START = 1; // start conversion

	ADC->INTFLAG.reg = ADC_INTFLAG_RESRDY; // clear the data ready flag
	syncADC();

	ADC->SWTRIG.bit.START = 1; // restart conversion, as changing inputs messes up first conversion

	while (ADC->INTFLAG.bit.RESRDY == 0);   // Wait for conversion to complete
	value_read = ADC->RESULT.reg; // read the value

	syncADC();
	ADC->CTRLA.bit.ENABLE = 0x00; // disable adc
	ADC->CTRLB.bit.DIFFMODE = 0; // put back into single-ended mode
	ADC->INPUTCTRL.bit.MUXNEG = ADC_PIN_GND; // set back muxneg to internal ground
	syncADC();

	return value_read;
}
#endif

void analogWriteFrequency( uint32_t pin, float frequency)
{
  PinDescription pinDesc = g_APinDescription[pin];
  uint32_t attr = pinDesc.ulPinAttribute;

#ifdef DAC
	if ((attr & PIN_ATTR_ANALOG) == PIN_ATTR_ANALOG)
	{
	    // DAC handling code
#if defined(__SAMD51__)
		if (pin == PIN_DAC0 || pin == PIN_DAC1) // 2 DACs on A0 (PA02) and A1 (PA05)
#else
	    if (pin == PIN_DAC0)  // Only 1 DAC on A0 (PA02)
#endif
		return;		// Do nothing for DAC pin
	}
#endif

	uint32_t tcNum = GetTCNumber(pinDesc.ulPWMChannel);
	float per = F_CPU/frequency;
	if (per > 0xffff)
		per = 0xffff;
	if (pwm_per[tcNum] != (uint16_t) per)
	{
		tcEnabled[tcNum] = false;
		pwm_per[tcNum] = (uint16_t) per;
	}
}

// Right now, PWM output only works on the pins with
// hardware support.  These are defined in the appropriate
// pins_*.c file.  For the rest of the pins, we default
// to digital output.
void analogWrite(uint32_t pin, uint32_t value)
{
  PinDescription pinDesc = g_APinDescription[pin];
  uint32_t attr = pinDesc.ulPinAttribute;

 // ATSAMR, for example, doesn't have a DAC
#ifdef DAC
	  if ((attr & PIN_ATTR_ANALOG) == PIN_ATTR_ANALOG)
	  {
	    // DAC handling code
#if defined(__SAMD51__)
		if (pin == PIN_DAC0 || pin == PIN_DAC1) { // 2 DACs on A0 (PA02) and A1 (PA05)
#else
	    if (pin == PIN_DAC0) { // Only 1 DAC on A0 (PA02)
#endif

#if defined(__SAMD51__)
	    value = mapResolution(value, _writeResolution, _dacResolution);

			uint8_t channel = (pin == PIN_DAC0 ? 0 : 1);

			pinPeripheral(pin, PIO_ANALOG);

			if(!dacEnabled[channel]){
				dacEnabled[channel] = true;

				syncDAC();
				DAC->CTRLA.bit.ENABLE = 0;     // disable DAC

				syncDAC();
				DAC->DACCTRL[channel].bit.ENABLE = 1;

				syncDAC();
				DAC->CTRLA.bit.ENABLE = 1;     // enable DAC

				if(channel == 0){

					while ( !DAC->STATUS.bit.READY0 );

					while (DAC->SYNCBUSY.bit.DATA0);
					DAC->DATA[0].reg = value;
				}
				else if(channel == 1){
					while ( !DAC->STATUS.bit.READY1 );

					while (DAC->SYNCBUSY.bit.DATA1);
					DAC->DATA[1].reg = value;
				}

				delayMicroseconds(10000);
			}

			//ERROR!
			while(!DAC->DACCTRL[channel].bit.ENABLE);

			if(channel == 0){

				while ( !DAC->STATUS.bit.READY0 );

				while (DAC->SYNCBUSY.bit.DATA0);
				DAC->DATA[0].reg = value;  // DAC on 10 bits.
			}
			else if(channel == 1){
				while ( !DAC->STATUS.bit.READY1 );

				while (DAC->SYNCBUSY.bit.DATA1);
				DAC->DATA[1].reg = value;  // DAC on 10 bits.
			}
#else
			syncDAC();
			DAC->DATA.reg = value & 0x3FF;  // DAC on 10 bits.
			syncDAC();
			DAC->CTRLA.bit.ENABLE = 0x01;     // Enable DAC
			syncDAC();
#endif // __SAMD51__
				return;
	  }
	}
#endif // DAC

#if defined(__SAMD51__)
	if(attr & (PIN_ATTR_PWM_E|PIN_ATTR_PWM_F|PIN_ATTR_PWM_G)){

		uint32_t tcNum = GetTCNumber(pinDesc.ulPWMChannel);
		uint8_t tcChannel = GetTCChannelNumber(pinDesc.ulPWMChannel);
		static bool tcEnabled[TCC_INST_NUM+TC_INST_NUM];

		if(attr & PIN_ATTR_PWM_E)
			pinPeripheral(pin, PIO_TIMER);
		else if(attr & PIN_ATTR_PWM_F)
			pinPeripheral(pin, PIO_TIMER_ALT);
		else if(attr & PIN_ATTR_PWM_G)
			pinPeripheral(pin, PIO_TCC_PDEC);

		if (!tcEnabled[tcNum]) {
		  tcEnabled[tcNum] = true;
	      GCLK->PCHCTRL[GCLK_CLKCTRL_IDs[tcNum]].reg = GCLK_PCHCTRL_GEN_GCLK0_Val | (1 << GCLK_PCHCTRL_CHEN_Pos); //use clock generator 0

	      // Set PORT
	      if (tcNum >= TCC_INST_NUM) {
				// -- Configure TC
				Tc* TCx = (Tc*) GetTC(pinDesc.ulPWMChannel);

				//reset
				TCx->COUNT8.CTRLA.bit.SWRST = 1;
				while (TCx->COUNT8.SYNCBUSY.bit.SWRST);

				// Disable TCx
				TCx->COUNT8.CTRLA.bit.ENABLE = 0;
				while (TCx->COUNT8.SYNCBUSY.bit.ENABLE);
				// Set Timer counter Mode to 8 bits, normal PWM, prescaler 1/256
				TCx->COUNT8.CTRLA.reg = TC_CTRLA_MODE_COUNT8 | TC_CTRLA_PRESCALER_DIV256;
				TCx->COUNT8.WAVE.reg = TC_WAVE_WAVEGEN_NPWM;

				while (TCx->COUNT8.SYNCBUSY.bit.CC0);
				// Set the initial value
				TCx->COUNT8.CC[tcChannel].reg = (uint8_t) value;
				while (TCx->COUNT8.SYNCBUSY.bit.CC0);
				// Set PER to maximum counter value (resolution : 0xFF)
				TCx->COUNT8.PER.reg = 0xFF;
				while (TCx->COUNT8.SYNCBUSY.bit.PER);
				// Enable TCx
				TCx->COUNT8.CTRLA.bit.ENABLE = 1;
				while (TCx->COUNT8.SYNCBUSY.bit.ENABLE);
			} else {
				// -- Configure TCC
				Tcc* TCCx = (Tcc*) GetTC(pinDesc.ulPWMChannel);

				TCCx->CTRLA.bit.SWRST = 1;
				while (TCCx->SYNCBUSY.bit.SWRST);

				// Disable TCCx
				TCCx->CTRLA.bit.ENABLE = 0;
				while (TCCx->SYNCBUSY.bit.ENABLE);
				// Set prescaler to 1/256
				TCCx->CTRLA.reg = TCC_CTRLA_PRESCALER_DIV256 | TCC_CTRLA_PRESCSYNC_GCLK;

				// Set TCx as normal PWM
				TCCx->WAVE.reg = TCC_WAVE_WAVEGEN_NPWM;
				while ( TCCx->SYNCBUSY.bit.WAVE );

				while (TCCx->SYNCBUSY.bit.CC0 || TCCx->SYNCBUSY.bit.CC1);
				// Set the initial value
				TCCx->CC[tcChannel].reg = (uint32_t) value;
				while (TCCx->SYNCBUSY.bit.CC0 || TCCx->SYNCBUSY.bit.CC1);
				// Set PER to maximum counter value (resolution : 0xFF)
				TCCx->PER.reg = 0xFF;
				while (TCCx->SYNCBUSY.bit.PER);
				// Enable TCCx
				TCCx->CTRLA.bit.ENABLE = 1;
				while (TCCx->SYNCBUSY.bit.ENABLE);
			}
		}
		else {
			if (tcNum >= TCC_INST_NUM) {
				Tc* TCx = (Tc*) GetTC(pinDesc.ulPWMChannel);
				TCx->COUNT8.CC[tcChannel].reg = (uint8_t) value;
				while (TCx->COUNT8.SYNCBUSY.bit.CC0 || TCx->COUNT8.SYNCBUSY.bit.CC1);
				} else {
				Tcc* TCCx = (Tcc*) GetTC(pinDesc.ulPWMChannel);
				while (TCCx->SYNCBUSY.bit.CTRLB);
				while (TCCx->SYNCBUSY.bit.CC0 || TCCx->SYNCBUSY.bit.CC1);
				TCCx->CCBUF[tcChannel].reg = (uint32_t) value;
				while (TCCx->SYNCBUSY.bit.CC0 || TCCx->SYNCBUSY.bit.CC1);
				TCCx->CTRLBCLR.bit.LUPD = 1;
				while (TCCx->SYNCBUSY.bit.CTRLB);
				}
		}

		return;
	}
	  
#else

  if ((attr & PIN_ATTR_PWM) == PIN_ATTR_PWM)
  {
	  uint32_t tcNum = GetTCNumber(pinDesc.ulPWMChannel);
	  uint8_t tcChannel = GetTCChannelNumber(pinDesc.ulPWMChannel);

	    if (attr & PIN_ATTR_TIMER) 
		{
#if !(ARDUINO_SAMD_VARIANT_COMPLIANCE >= 10603)
	      // Compatibility for cores based on SAMD core <=1.6.2
	      if (pinDesc.ulPinType == PIO_TIMER_ALT) {
	        pinPeripheral(pin, PIO_TIMER_ALT);
	      } 
		  else
#endif
	      {
	        pinPeripheral(pin, PIO_TIMER);
	      }
	    } 
		else 
		{
			if ((attr & PIN_ATTR_TIMER_ALT) == PIN_ATTR_TIMER_ALT)
			{
				//this is on an alt timer
				pinPeripheral(pin, PIO_TIMER_ALT);
			}
			else
				return;
		}

		if (tcNum >= TCC_INST_NUM) 
			value = mapResolution(value, _writeResolution, 16);
		else
			value = (pwm_per[tcNum]*value) >> _writeResolution;

		pwm_val[tcNum] = value;

	    if (!tcEnabled[tcNum]) 
		{
	      tcEnabled[tcNum] = true;
		  uint16_t GCLK_CLKCTRL_IDs[] = {
			GCLK_CLKCTRL_ID(GCM_TCC0_TCC1), // TCC0
			GCLK_CLKCTRL_ID(GCM_TCC0_TCC1), // TCC1
			GCLK_CLKCTRL_ID(GCM_TCC2_TC3),  // TCC2
			GCLK_CLKCTRL_ID(GCM_TCC2_TC3),  // TC3
			GCLK_CLKCTRL_ID(GCM_TC4_TC5),   // TC4
			GCLK_CLKCTRL_ID(GCM_TC4_TC5),   // TC5
			GCLK_CLKCTRL_ID(GCM_TC6_TC7),   // TC6
			GCLK_CLKCTRL_ID(GCM_TC6_TC7),   // TC7
		  };
		  GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_IDs[tcNum]);
		  while (GCLK->STATUS.bit.SYNCBUSY == 1);

		  // Set PORT
		  if (tcNum >= TCC_INST_NUM) 
		  {
			// -- Configure TC
			Tc* TCx = (Tc*) GetTC(pinDesc.ulPWMChannel);
			// Disable TCx
			TCx->COUNT16.CTRLA.bit.ENABLE = 0;
			syncTC_16(TCx);
			// Set Timer counter Mode to 16 bits, normal PWM
			TCx->COUNT16.CTRLA.reg |= TC_CTRLA_MODE_COUNT16 | TC_CTRLA_WAVEGEN_NPWM;
			syncTC_16(TCx);
			// Set the initial value
			TCx->COUNT16.CC[tcChannel].reg = (uint32_t) value;
			syncTC_16(TCx);
			// Enable TCx
			TCx->COUNT16.CTRLA.bit.ENABLE = 1;
			syncTC_16(TCx);
		  }
		  else 
		  {
			// -- Configure TCC
			Tcc* TCCx = (Tcc*) GetTC(pinDesc.ulPWMChannel);
			// Disable TCCx
			TCCx->CTRLA.bit.ENABLE = 0;
			syncTCC(TCCx);
			// Set TCCx as normal PWM
			TCCx->WAVE.reg |= TCC_WAVE_WAVEGEN_NPWM;
			syncTCC(TCCx);
			// Set the initial value
			TCCx->CC[tcChannel].reg = (uint32_t) value;
			syncTCC(TCCx);
			// Set PER to maximum counter value (resolution : 0xFFFF)
			TCCx->PER.reg = pwm_per[tcNum];
	//		if (tcNum==0)
	//			TCCx->WEXCTRL = TCC_WEXCTRL_DTHS(100) | TCC_WEXCTRL_DTLS(100) | TCC_WEXCTRL_DTIEN0 | TCC_WEXCTRL_DTIEN1 | TCC_WEXCTRL_OTMX(0x2);
			syncTCC(TCCx);
			// Enable TCCx
			TCCx->CTRLA.bit.ENABLE = 1;
			syncTCC(TCCx);
		  }
		}
		else 
		{
		  if (tcNum >= TCC_INST_NUM) 
		  {
			Tc* TCx = (Tc*) GetTC(pinDesc.ulPWMChannel);
			TCx->COUNT16.CC[tcChannel].reg = (uint32_t) value;
			syncTC_16(TCx);
		  }
		  else
		  {
			Tcc* TCCx = (Tcc*) GetTC(pinDesc.ulPWMChannel);
			TCCx->CTRLBSET.bit.LUPD = 1;
			syncTCC(TCCx);
			TCCx->CCB[tcChannel].reg = (uint32_t) value;
			syncTCC(TCCx);
			TCCx->CTRLBCLR.bit.LUPD = 1;
			syncTCC(TCCx);
		  }
		}
	  return;
  }
#endif

  // -- Defaults to digital write
  pinMode(pin, OUTPUT);
  value = mapResolution(value, _writeResolution, 8);
  if (value < 128) {
    digitalWrite(pin, LOW);
  } else {
    digitalWrite(pin, HIGH);
  }
}

#ifdef __cplusplus
}
#endif
