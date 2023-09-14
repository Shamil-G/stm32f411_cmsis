/*
 * mcp3421.h
 *
 *  Created on: Sep 14, 2023
 *      Author: sguss
 */

#ifndef INC_MCP3421_H_
#define INC_MCP3421_H_

#define ADDR_MCP3421	0x68
// Programm Gain Amplifier - PGA
#define GAIN_1			0x00
#define GAIN_2			0x01
#define GAIN_4			0x02
#define GAIN_8			0x03

#define USE_14b

#ifdef USE_12b
#define MCP3421_PRECISION 0x00
#define MCP3421_TIMEOUT		5
#define MCP3421_DATA_LEN	2
#define SIGN_BIT			0x08
#define LSB					1/1000
#endif

#ifdef USE_14b
#define MCP3421_PRECISION 0x04
#define MCP3421_TIMEOUT		17
#define MCP3421_DATA_LEN	2
#define LSB					250/1000000
#define SIGN_BIT			0x20
#endif

#ifdef USE_16b
#define MCP3421_PRECISION 0x08
#define MCP3421_TIMEOUT		70
#define MCP3421_DATA_LEN	2
#define LSB					62.5/1000000
#define SIGN_BIT			0x80
#endif

#ifdef USE_18b
#define MCP3421_PRECISION 0x0C
#define MCP3421_TIMEOUT	  300
#define MCP3421_DATA_LEN	3
#define LSB					15.625/1000000
#define SIGN_BIT			0x02
#endif

#define ONE_SHOT		0x00
#define COntinuous_Shot	0x10

#define ONE_SHOT_START	0x80


#endif /* INC_MCP3421_H_ */
