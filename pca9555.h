/*
 * Copyright 2014-23 Andre M. Maree/KSS Technologies (Pty) Ltd.
 */

#pragma		once

#include	"hal_i2cm.h"									// +x_struct_union +stdint

#ifdef __cplusplus
extern "C" {
#endif


// ######################################## Enumerations ###########################################


// ######################################### Structures ############################################


// ####################################### Public variables ########################################


// ####################################### Global functions ########################################

void pca9555DIG_IN_Config(u8_t pin);
u8_t pca9555DIG_IN_GetState(u8_t pin);
void pca9555DIG_IN_Invert(u8_t pin);

void pca9555DIG_OUT_Config(u8_t pin);
void pca9555DIG_OUT_SetState(u8_t pin, u8_t State, u8_t Now);
int	pca9555DIG_OUT_GetState(u8_t pin);
int	pca9555DIG_OUT_WriteAll(void);
void pca9555DIG_OUT_Toggle(u8_t pin);

void pca9555Init(void);

int	pca9555Diagnostics(i2c_di_t * psI2C_DI);
int	pca9555Identify(i2c_di_t * psI2C_DI);
int	pca9555Config(i2c_di_t * psI2C_DI);
void pca9555ReConfig(i2c_di_t * psI2C_DI);
int	pca9555Check(u32_t tIntvl);

int pca9555Report(report_t * psR);

#ifdef __cplusplus
}
#endif
