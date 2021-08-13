/*
 * Copyright 2014-21 Andre M. Maree/KSS Technologies (Pty) Ltd.
 */

#include	"hal_variables.h"
#include	"pca9555.h"

#include	"x_errors_events.h"
#include	"printfx.h"
#include	"syslog.h"
#include	"systiming.h"

#define	debugFLAG					0xF000

#define	debugREGISTERS				(debugFLAG & 0x0001)
#define	debugSTATES					(debugFLAG & 0x0002)

#define	debugTIMING					(debugFLAG_GLOBAL & debugFLAG & 0x1000)
#define	debugTRACK					(debugFLAG_GLOBAL & debugFLAG & 0x2000)
#define	debugPARAM					(debugFLAG_GLOBAL & debugFLAG & 0x4000)
#define	debugRESULT					(debugFLAG_GLOBAL & debugFLAG & 0x8000)

// ########################################## MACROS ###############################################

#define	pca9555ADDR_LOW				0x20		// default with A2 A1 A0 all '0'
#define	pca9555ADDR_HIGH			0x27
#define	pca9555NUM_PINS				16

// ######################################## Enumerations ###########################################

enum {													// Register index enumeration
	pca9555_IN,											// RO - INput status registers
	pca9555_OUT, 										// WO - OUTput control registers
	pca9555_POL, 										// WO - INput POLarity, 1=Inverted
	pca9555_CFG, 										// Direction config 0=OUT 1=IN
	pca9555_NUM,
} ;

// ######################################### Structures ############################################

typedef struct __attribute__((packed)) pca9555_s {
	i2c_di_t *	psI2C ;									// size = 4
	union {												// size = 8
		uint16_t		Regs[pca9555_NUM] ;
		struct __attribute__((packed)) {
			uint16_t	Reg_IN ;
			uint16_t	Reg_OUT ;
			uint16_t	Reg_POL ;
			uint16_t	Reg_CFG ;
		} ;
	} ;
	bool				f_WriteIsDirty ;
} pca9555_t ;
DUMB_STATIC_ASSERT(sizeof(pca9555_t) == 13) ;

// ######################################### Local variables #######################################

pca9555_t	sPCA9555 = { 0 } ;
const char * const DS9555RegNames[] = { "Input", "Output", "PolInv", "Config" } ;

// ####################################### Local functions #########################################

int32_t	pca9555ReadRegister(uint8_t Reg) {
	IF_PRINT(debugREGISTERS, "READ #%d %s : %016J\n", Reg, DS9555RegNames[Reg], sPCA9555.Regs[Reg]) ;
	uint8_t	cChr = Reg << 1 ;							// force to uint16_t boundary 0/2/4/6
	return halI2C_Queue(sPCA9555.psI2C, i2cWDR_FB, &cChr, sizeof(cChr),
			(uint8_t *) &sPCA9555.Regs[Reg], sizeof(uint16_t), (i2cq_p1_t) NULL, (i2cq_p2_t) NULL) ;
}

int32_t	pca9555WriteRegister(uint8_t Reg) {
	uint8_t	cBuf[3] ;
	cBuf[0] = Reg << 1 ;						// force to uint16_t boundary 0 / 2 / 4 / 6
	cBuf[1] = sPCA9555.Regs[Reg] >> 8 ;
	cBuf[2] = sPCA9555.Regs[Reg] & 0xFF ;
	IF_PRINT(debugREGISTERS, "WRITE #%d %s : %016J\n", Reg, DS9555RegNames[Reg], sPCA9555.Regs[Reg]) ;
	return halI2C_Queue(sPCA9555.psI2C, i2cW_FB, cBuf, sizeof(cBuf), (uint8_t *) NULL, 0, (i2cq_p1_t) NULL, (i2cq_p2_t) NULL) ;
}

void	pca9555SetDirection(uint16_t Mask) {
	sPCA9555.Regs[pca9555_CFG] = Mask ;					// 0xFFFF = Inputs, 0x0000 = Outputs
	pca9555WriteRegister(pca9555_CFG) ;
}

void	pca9555SetInversion(uint16_t Mask) {
	sPCA9555.Regs[pca9555_POL] = Mask ;					// 0x0000 = normal, 0xFFFF = inverted
	pca9555WriteRegister(pca9555_POL) ;
}

void	pca9555SetOutLevel(uint16_t Mask) {
	sPCA9555.Regs[pca9555_OUT] = Mask ;					// 0 = OFF, 1 = ON
	pca9555WriteRegister(pca9555_OUT) ;
}

void	pca9555Reset(void) {
	pca9555SetOutLevel(0xFFFF) ;
	pca9555SetInversion(0x0000) ;
	pca9555SetDirection(0xFFFF) ;
}

// ###################################### Global functions #########################################

void	pca9555DIG_IN_Config(uint8_t pin) {
	IF_myASSERT(debugPARAM, pin < pca9555NUM_PINS) ;
	// To configure as INput, make the bit a '1'
	sPCA9555.Regs[pca9555_CFG] |= (0x0001 << pin) ;
	pca9555WriteRegister(pca9555_CFG) ;
}

uint8_t	pca9555DIG_IN_GetState(uint8_t pin) {
	IF_myASSERT(debugPARAM, pin < pca9555NUM_PINS) ;
	// Ensure we are reading an input pin
	IF_myASSERT(debugSTATES, (sPCA9555.Regs[pca9555_CFG] & (0x0001 << pin)) == 1) ;
	pca9555ReadRegister(pca9555_IN) ;
	return (sPCA9555.Regs[pca9555_IN] & (0x0001 << pin)) ? 1 : 0 ;
}

void	pca9555DIG_IN_Invert(uint8_t pin) {
	IF_myASSERT(debugPARAM, pin < pca9555NUM_PINS) ;
	// Ensure we are inverting an input pin
	IF_myASSERT(debugSTATES, (sPCA9555.Regs[pca9555_CFG] & (1U << pin)) == 1) ;
	sPCA9555.Regs[pca9555_POL] ^= (1U << pin) ;
	pca9555WriteRegister(pca9555_POL) ;
}

void	pca9555DIG_OUT_Config(uint8_t pin) {
	IF_myASSERT(debugPARAM, pin < pca9555NUM_PINS) ;
	sPCA9555.Regs[pca9555_CFG] &= ~(1U << pin) ;		// To configure as OUTput, make the bit a '0'
	pca9555WriteRegister(pca9555_CFG) ;
}

void	pca9555DIG_OUT_SetState(uint8_t pin, uint8_t NewState, uint8_t Now) {
	IF_myASSERT(debugPARAM, (pin < pca9555NUM_PINS) && (sPCA9555.Regs[pca9555_CFG] & (0x0001 << pin)) == 0) ;
	uint8_t CurState = (sPCA9555.Regs[pca9555_OUT] & (1U << pin)) ? 1 : 0 ;
	if (NewState == CurState) {							// state not changed
		return ;										// then return
	}
	if (NewState == 1) {
		sPCA9555.Regs[pca9555_OUT] |= (1U << pin) ;
	} else {
		sPCA9555.Regs[pca9555_OUT] &= ~(1U << pin) ;
	}
	sPCA9555.f_WriteIsDirty = 1 ;						// bit just changed, show as dirty
	IF_PRINT(debugSTATES, "Pin #%d [%d -> %d]", pin, CurState, NewState) ;
	if (Now) {
		pca9555DIG_OUT_WriteAll() ;
	}
}

int32_t	pca9555DIG_OUT_GetState(uint8_t pin) {
	IF_myASSERT(debugPARAM, pin < pca9555NUM_PINS && (sPCA9555.Regs[pca9555_CFG] & (1 << pin)) == 0) ;
	return (sPCA9555.Regs[pca9555_OUT] & (1 << pin)) ? 1 : 0 ;
}

int32_t	pca9555DIG_OUT_WriteAll(void) {
	if (sPCA9555.f_WriteIsDirty) {
		IF_SYSTIMER_START(debugTIMING, stPCA9555) ;
		pca9555WriteRegister(pca9555_OUT) ;
		IF_SYSTIMER_STOP(debugTIMING, stPCA9555) ;
		sPCA9555.f_WriteIsDirty = 0 ;					// show as clean, just written
		return 1 ;
	}
	return 0 ;
}

void	pca9555DIG_OUT_Toggle(uint8_t pin) {
	IF_myASSERT(debugPARAM, (pin < pca9555NUM_PINS) && (sPCA9555.Regs[pca9555_CFG] & (0x0001 << pin)) == 0) ;
	sPCA9555.Regs[pca9555_OUT] ^= (1U << pin) ;
	pca9555WriteRegister(pca9555_OUT) ;
}

// ################################## Diagnostics functions ########################################

#define	pca9555TEST_INTERVAL			300

int32_t	pca9555Identify(i2c_di_t * psI2C_DI) {
	psI2C_DI->Delay	= pdMS_TO_TICKS(10) ;				// default device timeout
	psI2C_DI->Test	= 1 ;								// test mode
	sPCA9555.psI2C 	= psI2C_DI ;

	// Step 1 - ensure all set to defaults
	pca9555SetDirection(0xFFFF) ;						// default
	pca9555SetInversion(0x0000) ;						// default

	// Step 2 - read all registers
	for (int r = pca9555_IN; r < pca9555_NUM; pca9555ReadRegister(r++)) ;

	// Step 3 - Check initial default values
	if ((sPCA9555.Regs[pca9555_CFG] == 0xFFFF) &&
		(sPCA9555.Regs[pca9555_POL] == 0x0000)) {
		// passed phase 1, now step 4
		uint16_t OrigOUT = sPCA9555.Regs[pca9555_OUT] ;
		pca9555SetDirection(0x0000) ;					// all OUTputs
		pca9555ReadRegister(pca9555_OUT) ;
		if (sPCA9555.Regs[pca9555_OUT] == OrigOUT) {
			psI2C_DI->Type		= i2cDEV_PCA9555 ;
			// 3 bytes = 300uS @ 100Khz, 75uS @ 400Khz
			psI2C_DI->Speed		= i2cSPEED_400 ;
			psI2C_DI->DevIdx 	= 0 ;
			psI2C_DI->Test		= 0 ;
			return erSUCCESS ;
		}
	}
	psI2C_DI->Test	= 0 ;
	sPCA9555.psI2C 	= NULL ;
	return erFAILURE ;
}

int32_t	pca9555Config(i2c_di_t * psI2C_DI) {
	pca9555SetDirection(0x0000) ;						// Configure all as OUTputs
	pca9555SetInversion(0x0000) ;						// polarity NOT inverted
	IF_SYSTIMER_INIT(debugTIMING, stPCA9555, stMICROS, "PCA9555", 30, 3000) ;
	return erSUCCESS ;
}

void	pca9555ReConfig(i2c_di_t * psI2C_DI) {
	pca9555WriteRegister(pca9555_CFG) ;
	pca9555WriteRegister(pca9555_POL) ;
	pca9555WriteRegister(pca9555_OUT) ;
}

int32_t	pca9555Diagnostics(i2c_di_t * psI2C_DI) {
	// configure as outputs and display
	printfx("PCA9555: Default (all Outputs )status\n") ;
	pca9555SetDirection(0x0000) ;
	vTaskDelay(pdMS_TO_TICKS(pca9555TEST_INTERVAL)) ;

	// set all OFF and display
	printfx("PCA9555: All outputs (OFF) status\n") ;
	pca9555SetOutLevel(0x0000) ;
	vTaskDelay(pdMS_TO_TICKS(pca9555TEST_INTERVAL)) ;

	// set all ON and display
	printfx("PCA9555: All outputs (ON) status\n") ;
	pca9555SetOutLevel(0xFFFF) ;
	vTaskDelay(pdMS_TO_TICKS(pca9555TEST_INTERVAL)) ;

	// set all OFF and display
	printfx("PCA9555: All outputs (OFF) status\n") ;
	pca9555SetOutLevel(0x0000) ;
	vTaskDelay(pdMS_TO_TICKS(pca9555TEST_INTERVAL)) ;

	// set all back to inputs and display
	printfx("PCA9555: All Inputs (again) status\n") ;
	pca9555SetDirection(0xFFFF) ;
	vTaskDelay(pdMS_TO_TICKS(pca9555TEST_INTERVAL)) ;

	// Change INput to OUTput(0) and turn ON(1)
	printfx("PCA9555: Config as Outputs 1 by 1, switch ON using SetState\n") ;
	for (uint8_t pin = 0; pin < pca9555NUM_PINS; pin++) {
		pca9555DIG_OUT_Config(pin) ;				// default to OFF (0) after config
		pca9555DIG_OUT_SetState(pin, 1, 1) ;
		vTaskDelay(pdMS_TO_TICKS(pca9555TEST_INTERVAL)) ;
	}

	// then switch them OFF 1 by 1 using TOGGLE functionality
	printfx("PCA9555: Switch OFF 1 by 1 using TOGGLE\n") ;
	for (uint8_t pin = 0; pin < pca9555NUM_PINS; ++pin) {
		pca9555DIG_OUT_Toggle(pin) ;
		vTaskDelay(pdMS_TO_TICKS(pca9555TEST_INTERVAL)) ;
	}
	pca9555Reset() ;
	printfx("PCA9555: Diagnostics completed. All LEDs = OFF !!!\n") ;
	return erSUCCESS ;
}

/* Due to an induced reverse voltage cause by the collapsing magnetic field of the solenoid in the
 * door striker or water valve it can cause the I2C bus to "hang". In order to resolve this we need
 * to at regular intervals check that the PCA9555 can be read and that the value read back corresponds
 * with the last value written. If not, FSM of I2C peripheral on the ESP32 must be reset completely */
#define	pcaCHECK_INTERVAL				(30 * MILLIS_IN_SECOND)
uint32_t	pcaSuccessCount, pcaResetCount, pcaCheckInterval ;

int	pca9555Check(uint32_t tIntvl) {
	pcaCheckInterval += pdMS_TO_TICKS(tIntvl) ;
	if ((pcaCheckInterval % pcaCHECK_INTERVAL) >= pdMS_TO_TICKS(tIntvl)) return 0;
	pca9555ReadRegister(pca9555_IN) ;
	uint16_t TestRead	= sPCA9555.Reg_IN ;
	TestRead = ~TestRead ;
	TestRead = (TestRead >> 8) | (TestRead << 8) ;
	IF_PRINT(debugSTATES, "PCA9555  Rd=0x%04x  Adj=0x%04x  Wr=0x%04x\n", sPCA9555.Reg_IN, TestRead, sPCA9555.Reg_OUT) ;
	if (TestRead == sPCA9555.Reg_OUT) {
		++pcaSuccessCount ;
		return 0 ;										// all OK, no reset required...
	}
	// If not, general reset, reconfigure and start again...
	halI2C_Recover(sPCA9555.psI2C) ;					// Reset FSM
	return 1 ;
}

void	pca9555Report(void) {
	printfx("PCA9555\t INP=0x%04X  OUT=0x%04X  POL=0x%04X  CFG=0x%04x", sPCA9555.Regs[pca9555_IN],
			sPCA9555.Regs[pca9555_OUT], sPCA9555.Regs[pca9555_POL], sPCA9555.Regs[pca9555_CFG]) ;
	printfx("\tChecks  OK=%d  Fail=%d\n", pcaSuccessCount, pcaResetCount) ;
}
