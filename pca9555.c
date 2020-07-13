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
 * pca9555.c
 */

#include	"pca9555.h"									// +hal_i2c +x_struct_union +stdint

#include	"x_errors_events.h"							// + stdint
#include	"syslog.h"									// +x_definitions +stdarg +stdint
#include	"printfx.h"									// +x_definitions +stdarg +stdint +stdio
#include	"systiming.h"								// +x_definitions +stdbool +stdint

#include	"hal_debug.h"								// +esp_err +rom/esp_sys

#define	debugFLAG					0x0000

#define	debugREGISTERS				(debugFLAG & 0x0001)
#define	debugTIMING					(debugFLAG & 0x0002)

#define	debugTRACK					(debugFLAG & 0x2000)
#define	debugPARAM					(debugFLAG & 0x4000)
#define	debugSUCCESS				(debugFLAG & 0x8000)

// ########################################## MACROS ###############################################

#define	configPCA9555_LOW			0x20		// default with A2 A1 A0 all '0'
#define	configPCA9555_HIGH			0x27

// ######################################### Structures ############################################

typedef struct __attribute__((packed)) pca9555_s {
	i2c_dev_info_t *	psI2C ;						// size = 4
	union {											// size = 8
		uint16_t		Regs[regPCA9555_NUM] ;
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
const char * DS9555RegNames[] = { "Input", "Output", "PolInv", "Config" } ;

// ####################################### Local functions #########################################

int32_t	pca9555ReadRegister(uint8_t Reg) {
	IF_PRINT(debugREGISTERS, "#%d %s : %016J\n", Reg, DS9555RegNames[Reg], sPCA9555.Regs[Reg]) ;
	uint8_t	cChr = Reg << 1 ;							// force to uint16_t boundary 0 / 2 / 4 / 6
	return halI2C_WriteRead(sPCA9555.psI2C, &cChr, sizeof(cChr), (uint8_t *) &sPCA9555.Regs[Reg], sizeof(uint16_t)) ;
}

int32_t	pca9555WriteRegister(uint8_t Reg) {
	uint8_t	cBuf[3] ;
	cBuf[0] = Reg << 1 ;						// force to uint16_t boundary 0 / 2 / 4 / 6
	cBuf[1] = sPCA9555.Regs[Reg] >> 8 ;
	cBuf[2] = sPCA9555.Regs[Reg] & 0xFF ;
	IF_PRINT(debugREGISTERS, "#%d %s : %016J\n", Reg, DS9555RegNames[Reg], sPCA9555.Regs[Reg]) ;
	return halI2C_Write(sPCA9555.psI2C, cBuf, sizeof(cBuf)) ;
}

void	pca9555AllInputs(void) {
	sPCA9555.Regs[regPCA9555_CFG] = 0xFFFF ;
	pca9555WriteRegister(regPCA9555_CFG) ;
}

void	pca9555AllOutputs(void) {
	sPCA9555.Regs[regPCA9555_CFG] = 0x0000 ;
	pca9555WriteRegister(regPCA9555_CFG) ;
}

void	pca9555AllOFF(void) {
	sPCA9555.Regs[regPCA9555_OUT] = 0x0000 ;
	pca9555WriteRegister(regPCA9555_OUT) ;
}

void	pca9555AllON(void) {
	sPCA9555.Regs[regPCA9555_OUT] = 0xFFFF ;
	pca9555WriteRegister(regPCA9555_OUT) ;
}

void	pca9555Reset(void) {
	pca9555AllOutputs() ;
	pca9555AllOFF() ;
}

// ###################################### Global functions #########################################

void	pca9555DIG_IN_Config(uint8_t pin) {
	IF_myASSERT(debugPARAM, pin < pinPCA9555_NUM) ;
	// To configure as INput, make the bit a '1'
	sPCA9555.Regs[regPCA9555_CFG] |= (0x0001 << pin) ;
	pca9555WriteRegister(regPCA9555_CFG) ;
}

uint8_t	pca9555DIG_IN_GetState(uint8_t pin) {
	IF_myASSERT(debugPARAM, pin < pinPCA9555_NUM) ;
	// Ensure we are reading an input pin
	IF_myASSERT(debugTRACK, (sPCA9555.Regs[regPCA9555_CFG] & (0x0001 << pin)) == 1) ;
	pca9555ReadRegister(regPCA9555_IN) ;
	return (sPCA9555.Regs[regPCA9555_IN] & (0x0001 << pin)) ? true : false ;
}

void	pca9555DIG_IN_Invert(uint8_t pin) {
	IF_myASSERT(debugPARAM, pin < pinPCA9555_NUM) ;
	// Ensure we are inverting an input pin
	IF_myASSERT(debugTRACK, (sPCA9555.Regs[regPCA9555_CFG] & (1U << pin)) == 1) ;
	sPCA9555.Regs[regPCA9555_POL] ^= (1U << pin) ;
	pca9555WriteRegister(regPCA9555_POL) ;
}

void	pca9555DIG_OUT_Config(uint8_t pin) {
	IF_myASSERT(debugPARAM, pin < pinPCA9555_NUM) ;
	sPCA9555.Regs[regPCA9555_CFG] &= ~(1U << pin) ;		// To configure as OUTput, make the bit a '0'
	pca9555WriteRegister(regPCA9555_CFG) ;
}

void	pca9555DIG_OUT_SetState(uint8_t pin, uint8_t NewState, uint8_t Now) {
	IF_myASSERT(debugPARAM, (pin < pinPCA9555_NUM) && (sPCA9555.Regs[regPCA9555_CFG] & (0x0001 << pin)) == 0) ;
	uint8_t CurState = (sPCA9555.Regs[regPCA9555_OUT] & (1U << pin)) ? 1 : 0 ;
	if (NewState == CurState) {							// state not changed
		return ;										// then return
	}
	if (NewState == 1) {
		sPCA9555.Regs[regPCA9555_OUT] |= (1U << pin) ;
	} else {
		sPCA9555.Regs[regPCA9555_OUT] &= ~(1U << pin) ;
	}
	sPCA9555.f_WriteIsDirty = 1 ;						// bit just changed, show as dirty
	IF_PRINT(debugTRACK, "Pin #%d [%d -> %d]", pin, CurState, NewState) ;
	if (Now) {
		pca9555DIG_OUT_WriteAll() ;
	}
}

int32_t	pca9555DIG_OUT_GetState(uint8_t pin) {
	IF_myASSERT(debugPARAM, pin < pinPCA9555_NUM && (sPCA9555.Regs[regPCA9555_CFG] & (1 << pin)) == 0) ;
	return (sPCA9555.Regs[regPCA9555_OUT] & (1 << pin)) ? 1 : 0 ;
}

int32_t	pca9555DIG_OUT_WriteAll(void) {
	if (sPCA9555.f_WriteIsDirty) {
		IF_EXEC_1(debugTIMING && (systimerPCA9555 < 31), xSysTimerStart, systimerPCA9555) ;
		pca9555WriteRegister(regPCA9555_OUT) ;
		IF_EXEC_1(debugTIMING && (systimerPCA9555 < 31), xSysTimerStop, systimerPCA9555) ;
		sPCA9555.f_WriteIsDirty = 0 ;					// show as clean, just written
		return 1 ;
	}
	return 0 ;
}

void	pca9555DIG_OUT_Toggle(uint8_t pin) {
	IF_myASSERT(debugPARAM, (pin < pinPCA9555_NUM) && (sPCA9555.Regs[regPCA9555_CFG] & (0x0001 << pin)) == 0) ;
	sPCA9555.Regs[regPCA9555_OUT] ^= (1U << pin) ;
	pca9555WriteRegister(regPCA9555_OUT) ;
}

// ################################## Diagnostics functions ########################################

#define	pca9555TEST_INTERVAL			300

int32_t	pca9555Identify(i2c_dev_info_t * psI2C_DI) {
	psI2C_DI->Delay	= pdMS_TO_TICKS(10) ;
	psI2C_DI->Type	= i2cDEV_PCA9555 ;
	sPCA9555.psI2C 	= psI2C_DI ;

	pca9555AllInputs() ;								// Configure all as inputs
	sPCA9555.Regs[regPCA9555_POL] = 0x0000 ;			// and ensure none polarity inverted
	pca9555WriteRegister(regPCA9555_POL) ;
	pca9555ReadRegister(regPCA9555_IN) ;
	uint16_t RegStart = sPCA9555.Regs[regPCA9555_IN] ;	// save the pre-invert status

	sPCA9555.Regs[regPCA9555_POL] = 0xFFFF ;			// then invert all pins
	pca9555WriteRegister(regPCA9555_POL) ;
	pca9555ReadRegister(regPCA9555_IN) ;				// read back the (inverted) status
	if (sPCA9555.Regs[regPCA9555_IN] != (RegStart ^ 0xFFFF)) {	// check if it is as expected..
		return erFAILURE ;
	}
	return erSUCCESS ;
}

int32_t	pca9555Config(i2c_dev_info_t * psI2C_DI) {
	pca9555Reset() ;
	IF_SYSTIMER_INIT(debugTIMING, systimerPCA9555, systimerCLOCKS, "PCA9555", myUS_TO_CLOCKS(300), myUS_TO_CLOCKS(30000)) ;
	return erSUCCESS ;
}

int32_t	pca9555Diagnostics(i2c_dev_info_t * psI2C_DI) {
	// configure as outputs and display
	PRINT("PCA9555: Default (all Outputs )status\n") ;
	pca9555AllOutputs() ;
	vTaskDelay(pdMS_TO_TICKS(pca9555TEST_INTERVAL)) ;

	// set all OFF and display
	PRINT("PCA9555: All outputs (OFF) status\n") ;
	pca9555AllOFF() ;
	vTaskDelay(pdMS_TO_TICKS(pca9555TEST_INTERVAL)) ;

	// set all ON and display
	PRINT("PCA9555: All outputs (ON) status\n") ;
	pca9555AllON() ;
	vTaskDelay(pdMS_TO_TICKS(pca9555TEST_INTERVAL)) ;

	// set all OFF and display
	PRINT("PCA9555: All outputs (OFF) status\n") ;
	pca9555AllOFF() ;
	vTaskDelay(pdMS_TO_TICKS(pca9555TEST_INTERVAL)) ;

	// set all back to inputs and display
	PRINT("PCA9555: All Inputs (again) status\n") ;
	pca9555AllInputs() ;
	vTaskDelay(pdMS_TO_TICKS(pca9555TEST_INTERVAL)) ;

	// Change INput to OUTput(0) and turn ON(1)
	PRINT("PCA9555: Config as Outputs 1 by 1, switch ON using SetState\n") ;
	for (uint8_t pin = 0; pin < pinPCA9555_NUM; pin++) {
		pca9555DIG_OUT_Config(pin) ;				// default to OFF (0) after config
		pca9555DIG_OUT_SetState(pin, 1, 1) ;
		vTaskDelay(pdMS_TO_TICKS(pca9555TEST_INTERVAL)) ;
	}

	// then switch them OFF 1 by 1 using TOGGLE functionality
	PRINT("PCA9555: Switch OFF 1 by 1 using TOGGLE\n") ;
	for (uint8_t pin = 0; pin < pinPCA9555_NUM; pin++) {
		pca9555DIG_OUT_Toggle(pin) ;
		vTaskDelay(pdMS_TO_TICKS(pca9555TEST_INTERVAL)) ;
	}
	pca9555Reset() ;
	PRINT("PCA9555: Diagnostics completed. All LEDs = OFF !!!\n") ;
	return erSUCCESS ;
}

/**
 * Due to an induced reverse voltage cause by the collapsing magnetic field of the solenoid in the
 * door striker or water valve it can cause the I2C bus to "hang". In order to resolve this we need
 * to at regular intervals check that the PCA9555 can be read and that the value read back corresponds
 * with the last value written. If not, the FSM of the I2C peripheral on the ESP32 must be reset completely
 */
#define	pcaCHECK_INTERVAL				(30 * MILLIS_IN_SECOND)
uint32_t	pcaSuccessCount, pcaResetCount, pcaCheckInterval ;

int32_t	pca9555Check(uint32_t tIntvl) {
	pcaCheckInterval += pdMS_TO_TICKS(tIntvl) ;
	if ((pcaCheckInterval % pcaCHECK_INTERVAL) >= pdMS_TO_TICKS(tIntvl)) {
		return 0 ;
	}
	pca9555ReadRegister(regPCA9555_IN) ;
	uint16_t TestRead	= sPCA9555.Reg_IN ;
	TestRead = ~TestRead ;
	TestRead = (TestRead >> 8) | (TestRead << 8) ;
	IF_PRINT(debugTRACK, "PCA9555  Rd=0x%04x  Adj=0x%04x  Wr=0x%04x\n", sPCA9555.Reg_IN, TestRead, sPCA9555.Reg_OUT) ;
	if (TestRead == sPCA9555.Reg_OUT) {
		++pcaSuccessCount ;
		return 0 ;									// all OK, no reset required...
	}
	// If not, general reset, reconfigure and start again...
	halI2C_Recover(halI2C_0) ;							// Reset FSM

	pca9555WriteRegister(regPCA9555_CFG) ;
	pca9555WriteRegister(regPCA9555_POL) ;
	pca9555WriteRegister(regPCA9555_OUT) ;
	SL_ERR("I2C Recover done, ok=%d vs %d", pcaSuccessCount, ++pcaResetCount) ;
	return 1 ;
}
