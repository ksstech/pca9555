/*
 * pca9555.h - Copyright (c) 2014-23 Andre M. Maree/KSS Technologies (Pty) Ltd.
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif


// ######################################## Enumerations ###########################################


// ######################################### Structures ############################################

struct i2c_di_t;
struct report_t;

// ####################################### Public variables ########################################


// ####################################### Global functions ########################################

void pca9555DIG_IN_Config(u8_t pin);
u8_t pca9555DIG_IN_GetState(u8_t pin);
void pca9555DIG_IN_Invert(u8_t pin);

void pca9555DIG_OUT_Config(u8_t pin);
void pca9555DIG_OUT_SetStateLazy(u8_t pin, u8_t State);
void pca9555DIG_OUT_SetState(u8_t pin, u8_t State);
int pca9555DIG_OUT_WriteAll(void);
int	pca9555DIG_OUT_GetState(u8_t pin);
void pca9555DIG_OUT_Toggle(u8_t pin);

int	pca9555Diagnostics(struct i2c_di_t * psI2C);
int	pca9555Identify(struct i2c_di_t * psI2C);
int	pca9555Config(struct i2c_di_t * psI2C);
int	pca9555Check(void);

int pca9555Report(report_t * psR);

#ifdef __cplusplus
}
#endif
