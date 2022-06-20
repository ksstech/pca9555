/*
 * pca9555.c
 * Copyright (c) 2014-22 Andre M. Maree / KSS Technologies (Pty) Ltd.
 */

#include "hal_variables.h"
#include "pca9555.h"
#include "x_errors_events.h"
#include "printfx.h"
#include "syslog.h"
#include "systiming.h"

#define	debugFLAG					0xF000

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
		u16_t		Regs[pca9555_NUM] ;
		struct __attribute__((packed)) {
			u16_t	Reg_IN ;
			u16_t	Reg_OUT ;
			u16_t	Reg_POL ;
			u16_t	Reg_CFG ;
		} ;
	} ;
	bool				f_WriteIsDirty ;
} pca9555_t ;
DUMB_STATIC_ASSERT(sizeof(pca9555_t) == 13) ;

// ######################################### Local variables #######################################

pca9555_t	sPCA9555 = { 0 } ;
const char * const DS9555RegNames[] = { "Input", "Output", "PolInv", "Config" } ;

// ####################################### Local functions #########################################

int	pca9555ReadRegister(u8_t Reg) {
	u8_t	cChr = Reg << 1 ;							// force to u16_t boundary 0/2/4/6
	// Adding a delay of 0mS ensure that read and write operations are separately executed
	return halI2C_Queue(sPCA9555.psI2C, i2cWDR_FB, &cChr, sizeof(cChr),
			(u8_t *) &sPCA9555.Regs[Reg], sizeof(u16_t), (i2cq_p1_t) NULL, (i2cq_p2_t) NULL) ;
}

int	pca9555WriteRegister(u8_t Reg) {
	u8_t	cBuf[3];
	cBuf[0] = Reg << 1;									// force to u16_t boundary 0 / 2 / 4 / 6
	cBuf[1] = sPCA9555.Regs[Reg] >> 8;
	cBuf[2] = sPCA9555.Regs[Reg] & 0xFF;
	IF_SYSTIMER_START(debugTIMING, stPCA9555);
	int iRV = halI2C_Queue(sPCA9555.psI2C, i2cW_FB, cBuf, sizeof(cBuf), (u8_t *) NULL, 0, (i2cq_p1_t) NULL, (i2cq_p2_t) NULL);
	IF_SYSTIMER_STOP(debugTIMING, stPCA9555);
	return iRV;
}

void pca9555SetDirection(u16_t Mask) {
	sPCA9555.Regs[pca9555_CFG] = Mask ;					// 0xFFFF = Inputs, 0x0000 = Outputs
	pca9555WriteRegister(pca9555_CFG) ;
}

void pca9555SetInversion(u16_t Mask) {
	sPCA9555.Regs[pca9555_POL] = Mask ;					// 0x0000 = normal, 0xFFFF = inverted
	pca9555WriteRegister(pca9555_POL) ;
}

void pca9555SetOutLevel(u16_t Mask) {
	sPCA9555.Regs[pca9555_OUT] = Mask ;					// 0 = OFF, 1 = ON
	pca9555WriteRegister(pca9555_OUT) ;
}

void pca9555Reset(void) {
	pca9555SetOutLevel(0xFFFF) ;
	pca9555SetInversion(0x0000) ;
	pca9555SetDirection(0xFFFF) ;
}

// ###################################### Global functions #########################################

void pca9555DIG_IN_Config(u8_t pin) {
	IF_myASSERT(debugPARAM, pin < pca9555NUM_PINS) ;
	// To configure as INput, make the bit a '1'
	sPCA9555.Regs[pca9555_CFG] |= (0x0001 << pin) ;
	pca9555WriteRegister(pca9555_CFG) ;
}

u8_t	pca9555DIG_IN_GetState(u8_t pin) {
	IF_myASSERT(debugPARAM, pin < pca9555NUM_PINS) ;
	// Ensure we are reading an input pin
	IF_myASSERT(debugTRACK, (sPCA9555.Regs[pca9555_CFG] & (0x0001 << pin)) == 1) ;
	pca9555ReadRegister(pca9555_IN) ;
	return (sPCA9555.Regs[pca9555_IN] & (0x0001 << pin)) ? 1 : 0 ;
}

void pca9555DIG_IN_Invert(u8_t pin) {
	IF_myASSERT(debugPARAM, pin < pca9555NUM_PINS) ;
	// Ensure we are inverting an input pin
	IF_myASSERT(debugTRACK, (sPCA9555.Regs[pca9555_CFG] & (1U << pin)) == 1) ;
	sPCA9555.Regs[pca9555_POL] ^= (1U << pin) ;
	pca9555WriteRegister(pca9555_POL) ;
}

void pca9555DIG_OUT_Config(u8_t pin) {
	IF_myASSERT(debugPARAM, pin < pca9555NUM_PINS) ;
	sPCA9555.Regs[pca9555_CFG] &= ~(1U << pin) ;		// To configure as OUTput, make the bit a '0'
	pca9555WriteRegister(pca9555_CFG) ;
}

void pca9555DIG_OUT_SetState(u8_t pin, u8_t NewState, u8_t Now) {
	IF_myASSERT(debugPARAM, (pin < pca9555NUM_PINS) && (sPCA9555.Regs[pca9555_CFG] & (0x0001 << pin)) == 0) ;
	u8_t CurState = (sPCA9555.Regs[pca9555_OUT] & (1U << pin)) ? 1 : 0 ;
	if (NewState == CurState)
		return;
	if (NewState == 1)
		sPCA9555.Regs[pca9555_OUT] |= (1U << pin);
	else
		sPCA9555.Regs[pca9555_OUT] &= ~(1U << pin);
	sPCA9555.f_WriteIsDirty = 1;						// bit just changed, show as dirty
//	P("Pin #%d [%d -> %d]", pin, CurState, NewState);
	if (Now)
		pca9555DIG_OUT_WriteAll();
}

int	pca9555DIG_OUT_GetState(u8_t pin) {
	IF_myASSERT(debugPARAM, pin < pca9555NUM_PINS && (sPCA9555.Regs[pca9555_CFG] & (1 << pin)) == 0) ;
	return (sPCA9555.Regs[pca9555_OUT] & (1 << pin)) ? 1 : 0 ;
}

int	pca9555DIG_OUT_WriteAll(void) {
	if (sPCA9555.f_WriteIsDirty) {
		pca9555WriteRegister(pca9555_OUT) ;
		sPCA9555.f_WriteIsDirty = 0 ;					// show as clean, just written
		return 1 ;
	}
	return 0 ;
}

void pca9555DIG_OUT_Toggle(u8_t pin) {
	IF_myASSERT(debugPARAM, (pin < pca9555NUM_PINS) && (sPCA9555.Regs[pca9555_CFG] & (0x0001 << pin)) == 0) ;
	sPCA9555.Regs[pca9555_OUT] ^= (1U << pin) ;
	pca9555WriteRegister(pca9555_OUT) ;
}

// ################################## Diagnostics functions ########################################

#define	pca9555TEST_INTERVAL			300

int	pca9555Identify(i2c_di_t * psI2C_DI) {
	psI2C_DI->TRXmS	= 10;			// default device timeout
	psI2C_DI->CLKuS = 400;			// Max 13000 (13mS)
	psI2C_DI->Test	= 1;			// test mode
	sPCA9555.psI2C 	= psI2C_DI;

	// Step 1 - ensure all set to defaults
	pca9555SetDirection(0xFFFF) ;						// default
	pca9555SetInversion(0x0000) ;						// default

	// Step 2 - read all registers
	for (int r = pca9555_IN; r < pca9555_NUM; pca9555ReadRegister(r++)) ;

	// Step 3 - Check initial default values
	if ((sPCA9555.Regs[pca9555_CFG] == 0xFFFF) &&
		(sPCA9555.Regs[pca9555_POL] == 0x0000)) {
		// passed phase 1, now step 4
		u16_t OrigOUT = sPCA9555.Regs[pca9555_OUT] ;
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

int	pca9555Config(i2c_di_t * psI2C_DI) {
	pca9555SetDirection(0x0000) ;						// Configure all as OUTputs
	pca9555SetInversion(0x0000) ;						// polarity NOT inverted
	IF_SYSTIMER_INIT(debugTIMING, stPCA9555, stMICROS, "PCA9555", 200, 3200) ;
	return erSUCCESS ;
}

void pca9555ReConfig(i2c_di_t * psI2C_DI) {
	pca9555WriteRegister(pca9555_CFG) ;
	pca9555WriteRegister(pca9555_POL) ;
	pca9555WriteRegister(pca9555_OUT) ;
}

int	pca9555Diagnostics(i2c_di_t * psI2C_DI) {
	// configure as outputs and display
	printfx("PCA9555: Default (all Outputs )status\r\n") ;
	pca9555SetDirection(0x0000) ;
	vTaskDelay(pdMS_TO_TICKS(pca9555TEST_INTERVAL)) ;

	// set all OFF and display
	printfx("PCA9555: All outputs (OFF) status\r\n") ;
	pca9555SetOutLevel(0x0000) ;
	vTaskDelay(pdMS_TO_TICKS(pca9555TEST_INTERVAL)) ;

	// set all ON and display
	printfx("PCA9555: All outputs (ON) status\r\n") ;
	pca9555SetOutLevel(0xFFFF) ;
	vTaskDelay(pdMS_TO_TICKS(pca9555TEST_INTERVAL)) ;

	// set all OFF and display
	printfx("PCA9555: All outputs (OFF) status\r\n") ;
	pca9555SetOutLevel(0x0000) ;
	vTaskDelay(pdMS_TO_TICKS(pca9555TEST_INTERVAL)) ;

	// set all back to inputs and display
	printfx("PCA9555: All Inputs (again) status\r\n") ;
	pca9555SetDirection(0xFFFF) ;
	vTaskDelay(pdMS_TO_TICKS(pca9555TEST_INTERVAL)) ;

	// Change INput to OUTput(0) and turn ON(1)
	printfx("PCA9555: Config as Outputs 1 by 1, switch ON using SetState\r\n") ;
	for (u8_t pin = 0; pin < pca9555NUM_PINS; pin++) {
		pca9555DIG_OUT_Config(pin) ;				// default to OFF (0) after config
		pca9555DIG_OUT_SetState(pin, 1, 1) ;
		vTaskDelay(pdMS_TO_TICKS(pca9555TEST_INTERVAL)) ;
	}

	// then switch them OFF 1 by 1 using TOGGLE functionality
	printfx("PCA9555: Switch OFF 1 by 1 using TOGGLE\r\n") ;
	for (u8_t pin = 0; pin < pca9555NUM_PINS; ++pin) {
		pca9555DIG_OUT_Toggle(pin) ;
		vTaskDelay(pdMS_TO_TICKS(pca9555TEST_INTERVAL)) ;
	}
	pca9555Reset() ;
	printfx("PCA9555: Diagnostics completed. All LEDs = OFF !!!\r\n") ;
	return erSUCCESS ;
}

/* Due to an induced reverse voltage cause by the collapsing magnetic field of the solenoid in the
 * door striker or water valve it can cause the I2C bus to "hang". In order to resolve this we need
 * to at regular intervals check that the PCA9555 can be read and that the value read back corresponds
 * with the last value written. If not, FSM of I2C peripheral on the ESP32 must be reset completely */
#define	pcaCHECK_INTERVAL				(30 * MILLIS_IN_SECOND)
u32_t pcaSuccessCount, pcaResetCount, pcaCheckInterval ;

int	pca9555Check(u32_t tIntvl) {
	pcaCheckInterval += pdMS_TO_TICKS(tIntvl) ;
	if ((pcaCheckInterval % pcaCHECK_INTERVAL) >= pdMS_TO_TICKS(tIntvl))
		return 0;
	pca9555ReadRegister(pca9555_IN) ;
	u16_t TestRead	= sPCA9555.Reg_IN ;
	TestRead = ~TestRead ;
	TestRead = (TestRead >> 8) | (TestRead << 8) ;
//	P("PCA9555  Rd=0x%04x  Adj=0x%04x  Wr=0x%04x\r\n", sPCA9555.Reg_IN, TestRead, sPCA9555.Reg_OUT) ;
	if (TestRead == sPCA9555.Reg_OUT) {
		++pcaSuccessCount ;
		return 0 ;										// all OK, no reset required...
	}
	// If not, general reset, reconfigure and start again...
	halI2C_Recover(sPCA9555.psI2C) ;					// Reset FSM
	++pcaResetCount;
	return 1 ;
}

void pca9555Report(void) {
	printfx("\tPCA9555  I=x%04X  O=x%04X  P=x%04X  C=x%04x  OK=%d  Fail=%d\r\n",
			sPCA9555.Regs[pca9555_IN], sPCA9555.Regs[pca9555_OUT], sPCA9555.Regs[pca9555_POL],
			sPCA9555.Regs[pca9555_CFG], pcaSuccessCount, pcaResetCount) ;
}
