// pca9555.h

#pragma once

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

/**
 * @brief	Check if buffer is dirty, if so write to device.
 * @return	1/true if status written else 0/false
 */
bool pca9555DIG_OUT_WriteAll(void);

/**
 * @brief	Update buffered pin status without writing new status to device
 * @return	1/true if buffer is dirty/changed else 0/false
 */
bool pca9555DIG_OUT_SetStateLazy(u8_t pin, u8_t State);

/**
 * @brief	Update buffered pin status and [if changed] write status to device
 * @return	1/true if status written else 0/false
 */
bool pca9555DIG_OUT_SetState(u8_t pin, u8_t State);

int	pca9555DIG_OUT_GetState(u8_t pin);
void pca9555DIG_OUT_Toggle(u8_t pin);

struct i2c_di_t;
int	pca9555Diagnostics(struct i2c_di_t * psI2C);
int	pca9555Identify(struct i2c_di_t * psI2C);
int	pca9555Config(struct i2c_di_t * psI2C);
int	pca9555Check(void);

struct report_t;
int pca9555Report(struct report_t * psR);

#ifdef __cplusplus
}
#endif
