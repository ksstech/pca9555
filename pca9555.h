// pca9555.h

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// ######################################## Enumerations ###########################################

typedef enum { cfgDIR, cfgINV, stateGET, stateTGL_LAZY, stateTGL, stateSET_LAZY, stateSET, pca9555FUNC } pca9555func_e;

// ######################################### Structures ############################################

// ####################################### Public variables ########################################

// ####################################### Global functions ########################################

/**
 * @brief	Check if buffer is dirty, if so write to device.
 * @return	1/true if status written else 0/false
 */
int pca9555Flush(void);

/**
 * @brief	Configure Pin specified as INput
 * @param[in]	Pin on which the operation is to be performed
 * @return	erSUCCESS
 */
#define pca9555Direction(Pin, Dir) pca9555Function(cfgDIR, Pin, Dir)

/**
 * @brief	Configure Pin specified as INVerted INput
 * @param[in]	Pin on which the operation is to be performed
 * @return	erSUCCESS
 */
#define pca9555Invert(Pin) pca9555Function(cfgINV, Pin, 0)

/**
 * @brief	Read the state (0/1) of the Pin specified
 * @param[in]	Pin on which the operation is to be performed
 * @return	boolean state of the Pin
 */
#define pca9555GetState(Pin) pca9555Function(stateGET, Pin, 0)

/**
 * @brief	Toggle the state of a Pin (previously configured as OUTput)
 * @param[in]	Pin on which the operation is to be performed
 * @return	1/true if buffer is dirty/changed else 0/false
 */
#define pca9555ToggleLazy(Pin) pca9555Function(stateTGL_LAZY, Pin, 0)

/**
 * @brief	Toggle the state of a Pin (previously configured as OUTput)
 * @param[in]	Pin on which the operation is to be performed
 * @return	1/true if status written else 0/false
 */
#define pca9555Toggle(Pin) pca9555Function(stateTGL, Pin, 0)

/**
 * @brief	Update buffered pin status without writing new status to device
 * @param[in]	Pin on which the operation is to be performed
 * @param[in]	NewState optional state for SET type operations
 * @return	1/true if buffer is dirty/changed else 0/false
 */
#define pca9555SetStateLazy(Pin, NewState) pca9555Function(stateSET_LAZY, Pin, NewState)

/**
 * @brief	Update buffered pin status and [if changed] write status to device
 * @param[in]	Pin on which the operation is to be performed
 * @param[in]	NewState optional state for SET type operations
 * @return	1/true if status written else 0/false
 */
#define pca9555SetState(Pin, NewState) pca9555Function(stateSET, Pin, NewState)

/**
 * @brief	Perform configuration, read, write and toggle operations on device
 * @param[in]	Func enumerated operation function code
 * @param[in]	Pin on which the operation is to be performed
 * @param[in]	NewState optional state for SET type operations
 * @return	
 */
int pca9555Function(pca9555func_e Func, u8_t Pin, bool NewState);

struct i2c_di_t;
int	pca9555Diagnostics(struct i2c_di_t * psI2C);
int	pca9555Identify(struct i2c_di_t * psI2C);
int	pca9555Config(struct i2c_di_t * psI2C);
int	pca9555Verify(void);

struct report_t;
int pca9555Report(struct report_t * psR);

#ifdef __cplusplus
}
#endif
