/*
 * Copyright 2014-21 AM Maree/KSS Technologies (Pty) Ltd.
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

#include	"hal_i2c.h"									// +x_struct_union +stdint

#ifdef __cplusplus
extern "C" {
#endif

// ######################################## Enumerations ###########################################

enum {												// Register index enumeration
	regPCA9555_IN,									// INput status registers
	regPCA9555_OUT, 								// OUTput control registers
	regPCA9555_POL, 								// POLarity 0=normal, 1=Inverted
	regPCA9555_CFG, 								// Direction config 0=OUT 1=IN
	regPCA9555_NUM,
} ;

enum {												// bit shifted position into the registers
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


// ####################################### Public variables ########################################


// ####################################### Global functions ########################################

void	pca9555DIG_IN_Config(uint8_t pin) ;
uint8_t	pca9555DIG_IN_GetState(uint8_t pin) ;
void	pca9555DIG_IN_Invert(uint8_t pin) ;

void	pca9555DIG_OUT_Config(uint8_t pin) ;
void	pca9555DIG_OUT_SetState(uint8_t pin, uint8_t State, uint8_t Now) ;
int32_t	pca9555DIG_OUT_GetState(uint8_t pin) ;
int32_t	pca9555DIG_OUT_WriteAll(void) ;
void	pca9555DIG_OUT_Toggle(uint8_t pin) ;

int32_t	pca9555Diagnostics(i2c_dev_info_t * psI2C_DI) ;
int32_t	pca9555Identify(i2c_dev_info_t * psI2C_DI) ;
int32_t	pca9555Config(i2c_dev_info_t * psI2C_DI) ;
int32_t	pca9555Check(uint32_t tIntvl) ;

#ifdef __cplusplus
}
#endif
