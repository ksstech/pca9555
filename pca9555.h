// pca9555.h

#pragma once

#include "hal_i2c_common.h"

#ifdef __cplusplus
extern "C" {
#endif

// ########################################## MACROS ###############################################

#define	pca9555ADDR_LOW				0x20				// default with A2 A1 A0 all '0'
#define	pca9555ADDR_HIGH			0x27
#define	pca9555NUM_PINS				16

// ######################################## Enumerations ###########################################

enum {													// Register index enumeration
	pca9555_IN,											// RO - INput status registers
	pca9555_OUT, 										// WO - OUTput control registers
	pca9555_POL, 										// WO - INput POLarity, 1=Inverted
	pca9555_CFG, 										// Direction config 0=OUT 1=IN
	pca9555_NUM,
};

typedef enum { cfgDIR, cfgINV, stateGET, stateTGL_LAZY, stateTGL, stateSET_LAZY, stateSET, pca9555FUNC } pca9555func_e;

// ######################################### Structures ############################################

typedef struct __attribute__((packed)) pca9555_s {
	i2c_di_t *	psI2C;									// size = 4
	union {												// size = 8
		u16_t Regs[pca9555_NUM];
		struct __attribute__((packed)) {
			u16_t	Reg_IN;
			u16_t	Reg_OUT;
			u16_t	Reg_POL;
			u16_t	Reg_CFG;
		};
	};
	bool fDirty;
} pca9555_t;
DUMB_STATIC_ASSERT(sizeof(pca9555_t) == 13);

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
