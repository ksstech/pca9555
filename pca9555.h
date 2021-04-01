/*
 * Copyright 2014-21 AM Maree/KSS Technologies (Pty) Ltd.
 */

#pragma		once

#include	"hal_i2c.h"									// +x_struct_union +stdint

#ifdef __cplusplus
extern "C" {
#endif


// ######################################## Enumerations ###########################################


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
