/*
 * ads115.h
 *
 *  Created on: Sep 18, 2023
 *      Author: sguss
 */

#ifndef INC_ADS1115_H_
#define INC_ADS1115_H_


#define ADDR_ADS1115_GND	0x48
#define ADDR_ADS1115_VDD	0x49
#define ADDR_ADS1115_SDA	0x4A
#define ADDR_ADS1115_SCL	0x4B
// Programm Gain Amplifier - PGA

// ВОЗМОЖНЫЕ ВАРИАНТЫ УСТАНОВКИ КУ:
// ads.setGain(GAIN_TWOTHIRDS); | 2/3х | +/-6.144V | 1bit = 0.1875mV    |
// ads.setGain(GAIN_ONE);       | 1х   | +/-4.096V | 1bit = 0.125mV     |
// ads.setGain(GAIN_TWO);       | 2х   | +/-2.048V | 1bit = 0.0625mV    |
// ads.setGain(GAIN_FOUR);      | 4х   | +/-1.024V | 1bit = 0.03125mV   |
// ads.setGain(GAIN_EIGHT);     | 8х   | +/-0.512V | 1bit = 0.015625mV  |
// ads.setGain(GAIN_SIXTEEN);   | 16х  | +/-0.256V | 1bit = 0.0078125mV |

#define ONE_SHOT_START	0x8000


#ifdef USE_GAIN_2_3
#define PGA 0x0000
#define LSB 0.185 / 1000
#endif

#ifdef USE_GAIN_1
#define PGA 0x0200
#define LSB 0.125 / 1000
#endif

#ifdef USE_GAIN_2
#define PGA 0x0400
#define LSB 0.0625 / 1000
#endif

#ifdef USE_GAIN_4
#define PGA 0x0600
#define LSB 0.03125 / 1000
#endif

#ifdef USE_GAIN_8
#define PGA 0x0800
#define LSB 0.015625 / 1000
#endif

#ifdef USE_GAIN_16
#define PGA 0x0A00
#define LSB 0.0078125 / 1000
#endif


// Address Point Register
#define	CONVERSION_REG 	0x00
#define	CONFIG_REG 		0x01
#define	LO_THREAD_REG 	0x02
#define	HI_THREAD_REG 	0x03

// Config Register
#define FSR_6			0x0000
#define FSR_4			0x0200
#define FSR_2			0x0400
#define FSR_1			0x0600
#define FSR_05			0x0800
#define FSR_025			0x0A00

#define ONE_SHOT		0x0100
#define SPS_8			0x0000
#define SPS_16			0x0020
#define SPS_32			0x0040
#define SPS_64			0x0060
#define SPS_128			0x0080
#define SPS_250			0x00a0
#define SPS_475			0x00c0
#define SPS_860			0x00e0

#define TIMEOUT_SPS_8	1000/8
#define TIMEOUT_SPS_16	1000/16
#define TIMEOUT_SPS_32	1000/32
#define TIMEOUT_SPS_64	1000/64
#define TIMEOUT_SPS_128	1000/128
#define TIMEOUT_SPS_250	1000/250
#define TIMEOUT_SPS_475	1000/475
#define TIMEOUT_SPS_860	1000/860

#define DISABLE_COMPARATOR 0x0003

#endif /* INC_ADS115_H_ */
