/*
 * Copyright 2014-18 AM Maree/KSS Technologies (Pty) Ltd.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software
 * and associated documentation files (the "Software"), to deal in the Software without restriction,
 * including without limitation the rights to use, copy, modify, merge, publish, distribute,
 * sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
 * BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */

/*
 * pca9555.h
 */

#pragma		once

#include	"hal_i2c.h"

// Device base address
#define	configPCA9555_ADDR_0			0x20		// default with A2 A1 A0 all '0'
#define	configPCA9555_ADDR_1			0x21
#define	configPCA9555_ADDR_2			0x22
#define	configPCA9555_ADDR_3			0x23
#define	configPCA9555_ADDR_4			0x24
#define	configPCA9555_ADDR_5			0x25
#define	configPCA9555_ADDR_6			0x26
#define	configPCA9555_ADDR_7			0x27

// Device registers
#define	configPCA9555_INPUT_0			0x00		// INput status registers
#define	configPCA9555_INPUT_1			0x01
#define	configPCA9555_OUTPUT_0			0x02		// OUTput control registers
#define	configPCA9555_OUTPUT_1			0x03
#define	configPCA9555_POLINV_0			0x04		// Polarity inversion
#define	configPCA9555_POLINV_1			0x05		// 0=normal, 1=Inverted
#define	configPCA9555_CONFIG_0			0x06		// Direction config
#define	configPCA9555_CONFIG_1			0x07		// 0=Out, 1=In

// ######################################## Enumerations ###########################################

enum { regPCA9555_IN, regPCA9555_OUT, regPCA9555_POL, regPCA9555_CFG, regPCA9555_NUM } ;

// Enumeration used as bit shifted position into the registers
enum {
	pinPCA9555_0,
	pinPCA9555_1,
	pinPCA9555_2,
	pinPCA9555_3,
	pinPCA9555_4,
	pinPCA9555_5,
	pinPCA9555_6,
	pinPCA9555_7,
	pinPCA9555_8,
	pinPCA9555_9,
	pinPCA9555_10,
	pinPCA9555_11,
	pinPCA9555_12,
	pinPCA9555_13,
	pinPCA9555_14,
	pinPCA9555_15,
	pinPCA9555_NUM,
} ;

// ######################################### Structures ############################################

typedef struct {
	halI2Cdev_t			sI2Cdev ;
	union {
		uint16_t		Regs[regPCA9555_NUM] ;
		struct {
			uint16_t	Reg_IN ;
			uint16_t	Reg_OUT ;
			uint16_t	Reg_POL ;
			uint16_t	Reg_CFG ;
		} ;
	} ;
	bool				Invert[pinPCA9555_NUM] ;
	bool				f_WriteIsDirty ;
} pca9555_t ;

extern	pca9555_t	sPCA9555 ;
extern	uint32_t	pcaSuccessCount, pcaResetCount, pcaCheckInterval ;

// ####################################### Global functions ########################################

void	pca9555DIG_IN_Config(uint8_t pin) ;
uint8_t	pca9555DIG_IN_GetState(uint8_t pin) ;
void	pca9555DIG_IN_Invert(uint8_t pin) ;

void	pca9555DIG_OUT_Config(uint8_t pin) ;
void	pca9555DIG_OUT_SetState(uint8_t pin, uint8_t State, uint8_t Now) ;
int32_t	pca9555DIG_OUT_GetState(uint8_t pin) ;
int32_t	pca9555DIG_OUT_WriteAll(void) ;
void	pca9555DIG_OUT_Toggle(uint8_t pin) ;

int32_t	pca9555Diagnostics(void) ;
int32_t	pca9555Identify(uint8_t eChan, uint8_t Addr) ;
int32_t	pca9555Config(void) ;
int32_t	pca9555Check(uint32_t tIntvl) ;
