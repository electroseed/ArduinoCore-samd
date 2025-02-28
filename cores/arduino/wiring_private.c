/*
  Copyright (c) 2015 Arduino LLC.  All right reserved.
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

int pinPeripheral( uint32_t ulPin, EPioType ulPeripheral )
{
  // Handle the case the pin isn't usable as PIO
	EPortType port = g_APinDescription[ulPin].ulPort;
	uint8_t pin = g_APinDescription[ulPin].ulPin;
	
	if (g_APinDescription[ulPin].ulPinType == PIO_NOT_A_PIN)
		return -1 ;

	switch ( ulPeripheral )
	{
	case PIO_DIGITAL:
	case PIO_INPUT:
	case PIO_INPUT_PULLUP:
	case PIO_OUTPUT:
		// Disable peripheral muxing, done in pinMode
	//			PORT->Group[port].PINCFG[pin].bit.PMUXEN = 0 ;

		// Configure pin mode, if requested
		if ( ulPeripheral == PIO_INPUT )
			pinMode( ulPin, INPUT ) ;
		else
		{
			if ( ulPeripheral == PIO_INPUT_PULLUP )
				pinMode( ulPin, INPUT_PULLUP ) ;
			else
			{
				if ( ulPeripheral == PIO_OUTPUT )
					pinMode( ulPin, OUTPUT ) ;
				else
				{
				// PIO_DIGITAL, do we have to do something as all cases are covered?
				}
			}
		}
	break ;

	case PIO_ANALOG:
	case PIO_SERCOM:
	case PIO_SERCOM_ALT:
	case PIO_TIMER:
	case PIO_TIMER_ALT:
	case PIO_EXTINT:
#if defined(__SAMD51__)
	case PIO_TCC_PDEC:
	case PIO_COM:
	case PIO_SDHC:
	case PIO_I2S:
	case PIO_PCC:
	case PIO_GMAC:
	case PIO_AC_CLK:
	case PIO_CCL:
#else
	case PIO_COM:
	case PIO_AC_CLK:
#endif
	#if 0
		// Is the pio pin in the lower 16 ones?
		// The WRCONFIG register allows update of only 16 pin max out of 32
		if ( pin < 16 )
		{
		PORT->Group[port].WRCONFIG.reg = PORT_WRCONFIG_WRPMUX | PORT_WRCONFIG_PMUXEN | PORT_WRCONFIG_PMUX( ulPeripheral ) |
																	PORT_WRCONFIG_WRPINCFG |
																	PORT_WRCONFIG_PINMASK( pin ) ;
		}
		else
		{
		PORT->Group[port].WRCONFIG.reg = PORT_WRCONFIG_HWSEL |
																	PORT_WRCONFIG_WRPMUX | PORT_WRCONFIG_PMUXEN | PORT_WRCONFIG_PMUX( ulPeripheral ) |
																	PORT_WRCONFIG_WRPINCFG |
																	PORT_WRCONFIG_PINMASK( pin - 16 ) ;
		}
	#else
		if ( pin & 1 ) // is pin odd?
		{
			// Get whole current setup for both odd and even pins and remove odd one
			uint32_t temp = (PORT->Group[port].PMUX[pin>> 1].reg) & PORT_PMUX_PMUXE(0xF);
			// Set new muxing
			PORT->Group[port].PMUX[pin >> 1].reg = temp|PORT_PMUX_PMUXO( ulPeripheral );
			// Enable port mux
			PORT->Group[port].PINCFG[pin].reg |= PORT_PINCFG_PMUXEN;// | PORT_PINCFG_DRVSTR;
		}
		else // even pin
		{
			uint32_t temp = (PORT->Group[port].PMUX[pin >> 1].reg) & PORT_PMUX_PMUXO( 0xF );
			PORT->Group[port].PMUX[pin >> 1].reg = temp|PORT_PMUX_PMUXE( ulPeripheral );
			PORT->Group[port].PINCFG[pin].reg |= PORT_PINCFG_PMUXEN;// | PORT_PINCFG_DRVSTR; // Enable port mux
		}
	#endif
	break ;

	case PIO_NOT_A_PIN:
		return -1l ;
	break ;
	}

	return 0l ;
}

