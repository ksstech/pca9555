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

#include	"hal_config.h"
#include	"hal_i2c.h"
#include	"pca9555.h"

#include	"x_debug.h"
#include	"x_printf.h"
#include	"x_buffers.h"
#include	"x_errors_events.h"
#include	"x_syslog.h"
#include	"x_systiming.h"

#include	<stdint.h>

#define	debugFLAG					0x0002

#define	debugREGISTERS				(debugFLAG & 0x0001)
#define	debugTIMING					(debugFLAG & 0x0002)

#define	debugTRACK					(debugFLAG & 0x2000)
#define	debugPARAM					(debugFLAG & 0x4000)
#define	debugSUCCESS				(debugFLAG & 0x8000)

pca9555_t	sPCA9555 = { 0 } ;

const char * DS9555RegNames[] = { "Input", "Output", "PolInv", "Config" } ;

// ####################################### Local functions #########################################

int32_t	pca9555ReadRegister(pca9555_t * psPCA9555, uint8_t Reg) {
	IF_PRINT(debugREGISTERS, "#%d %s : %016J\n", Reg, DS9555RegNames[Reg], psPCA9555->Regs[Reg]) ;
	uint8_t	cChr = Reg << 1 ;							// force to uint16_t boundary 0 / 2 / 4 / 6
	int32_t iRetVal = halI2C_WriteRead(&psPCA9555->sI2Cdev, &cChr, sizeof(cChr), (uint8_t *) &psPCA9555->Regs[Reg], sizeof(uint16_t)) ;
	IF_myASSERT(debugSUCCESS, iRetVal == erSUCCESS)
	return iRetVal ;
}

int32_t	pca9555WriteRegister(pca9555_t * psPCA9555, uint8_t Reg) {
	uint8_t	cBuf[3] ;
	cBuf[0] = Reg << 1 ;						// force to uint16_t boundary 0 / 2 / 4 / 6
	cBuf[1] = psPCA9555->Regs[Reg] >> 8 ;
	cBuf[2] = psPCA9555->Regs[Reg] & 0xFF ;
	IF_PRINT(debugREGISTERS, "#%d %s : %016J\n", Reg, DS9555RegNames[Reg], psPCA9555->Regs[Reg]) ;
	int32_t iRetVal = halI2C_Write(&psPCA9555->sI2Cdev, cBuf, sizeof(cBuf)) ;
	IF_myASSERT(debugSUCCESS, iRetVal == erSUCCESS) ;
	return iRetVal ;
}

void	pca9555AllInputs(pca9555_t * psPCA9555) {
	psPCA9555->Regs[regPCA9555_CFG] = 0xFFFF ;
	pca9555WriteRegister(psPCA9555, regPCA9555_CFG) ;
}

void	pca9555AllOutputs(pca9555_t * psPCA9555) {
	psPCA9555->Regs[regPCA9555_CFG] = 0x0000 ;
	pca9555WriteRegister(psPCA9555, regPCA9555_CFG) ;
}

void	pca9555AllOFF(pca9555_t * psPCA9555) {
	psPCA9555->Regs[regPCA9555_OUT] = 0x0000 ;
	pca9555WriteRegister(psPCA9555, regPCA9555_OUT) ;
}

void	pca9555AllON(pca9555_t * psPCA9555) {
	psPCA9555->Regs[regPCA9555_OUT] = 0xFFFF ;
	pca9555WriteRegister(psPCA9555, regPCA9555_OUT) ;
}

void	pca9555Reset(pca9555_t * psPCA9555) {
	pca9555AllOutputs(psPCA9555) ;
	pca9555AllOFF(psPCA9555) ;
}

// ###################################### Global functions #########################################

void	pca9555DIG_IN_Config(uint8_t pin) {
	IF_myASSERT(debugPARAM, pin < pinPCA9555_NUM)
	// To configure as INput, make the bit a '1'
	sPCA9555.Regs[regPCA9555_CFG] |= (0x0001 << pin) ;
	pca9555WriteRegister(&sPCA9555, regPCA9555_CFG) ;
}

uint8_t	pca9555DIG_IN_GetState(uint8_t pin) {
	IF_myASSERT(debugPARAM, pin < pinPCA9555_NUM)
	// Ensure we are reading an input pin
	IF_myASSERT(debugTRACK, (sPCA9555.Regs[regPCA9555_CFG] & (0x0001 << pin)) == 1)
	pca9555ReadRegister(&sPCA9555, regPCA9555_IN) ;
	return (sPCA9555.Regs[regPCA9555_IN] & (0x0001 << pin)) ? true : false ;
}

void	pca9555DIG_IN_Invert(uint8_t pin) {
	IF_myASSERT(debugPARAM, pin < pinPCA9555_NUM)
	// Ensure we are inverting an input pin
	IF_myASSERT(debugTRACK, (sPCA9555.Regs[regPCA9555_CFG] & (1U << pin)) == 1)
	sPCA9555.Regs[regPCA9555_POL] ^= (1U << pin) ;
	pca9555WriteRegister(&sPCA9555, regPCA9555_POL) ;
}

void	pca9555DIG_OUT_Config(uint8_t pin) {
	IF_myASSERT(debugPARAM, pin < pinPCA9555_NUM)
	sPCA9555.Regs[regPCA9555_CFG] &= ~(1U << pin) ;		// To configure as OUTput, make the bit a '0'
	pca9555WriteRegister(&sPCA9555, regPCA9555_CFG) ;
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
		pca9555WriteRegister(&sPCA9555, regPCA9555_OUT) ;
		IF_EXEC_1(debugTIMING && (systimerPCA9555 < 31), xSysTimerStop, systimerPCA9555) ;
		sPCA9555.f_WriteIsDirty = 0 ;					// show as clean, just written
		return 1 ;
	}
	return 0 ;
}

void	pca9555DIG_OUT_Toggle(uint8_t pin) {
	IF_myASSERT(debugPARAM, (pin < pinPCA9555_NUM) && (sPCA9555.Regs[regPCA9555_CFG] & (0x0001 << pin)) == 0) ;
	sPCA9555.Regs[regPCA9555_OUT] ^= (1U << pin) ;
	pca9555WriteRegister(&sPCA9555, regPCA9555_OUT) ;
}

// ################################## Diagnostics functions ########################################

#define	pca9555TEST_INTERVAL			300

int32_t	pca9555Diagnostics(void) {
	// configure as outputs and display
	printfx("PCA9555: Default (all Outputs )status\n") ;
	pca9555AllOutputs(&sPCA9555) ;
	vTaskDelay(pdMS_TO_TICKS(pca9555TEST_INTERVAL)) ;

	// set all OFF and display
	printfx("PCA9555: All outputs (OFF) status\n") ;
	pca9555AllOFF(&sPCA9555) ;
	vTaskDelay(pdMS_TO_TICKS(pca9555TEST_INTERVAL)) ;

	// set all ON and display
	printfx("PCA9555: All outputs (ON) status\n") ;
	pca9555AllON(&sPCA9555) ;
	vTaskDelay(pdMS_TO_TICKS(pca9555TEST_INTERVAL)) ;

	// set all OFF and display
	printfx("PCA9555: All outputs (OFF) status\n") ;
	pca9555AllOFF(&sPCA9555) ;
	vTaskDelay(pdMS_TO_TICKS(pca9555TEST_INTERVAL)) ;

	// set all back to inputs and display
	printfx("PCA9555: All Inputs (again) status\n") ;
	pca9555AllInputs(&sPCA9555) ;
	vTaskDelay(pdMS_TO_TICKS(pca9555TEST_INTERVAL)) ;

	// Change INput to OUTput(0) and turn ON(1)
	printfx("PCA9555: Config as Outputs 1 by 1, switch ON using SetState\n") ;
	for (uint8_t pin = 0; pin < pinPCA9555_NUM; pin++) {
		pca9555DIG_OUT_Config(pin) ;				// default to OFF (0) after config
		pca9555DIG_OUT_SetState(pin, 1, 1) ;
		vTaskDelay(pdMS_TO_TICKS(pca9555TEST_INTERVAL)) ;
	}

	// then switch them OFF 1 by 1 using TOGGLE functionality
	printfx("PCA9555: Switch OFF 1 by 1 using TOGGLE\n") ;
	for (uint8_t pin = 0; pin < pinPCA9555_NUM; pin++) {
		pca9555DIG_OUT_Toggle(pin) ;
		vTaskDelay(pdMS_TO_TICKS(pca9555TEST_INTERVAL)) ;
	}
	pca9555Reset(&sPCA9555) ;
	printfx("PCA9555: Diagnostics completed. All LEDs = OFF !!!\n") ;
	return erSUCCESS ;
}

int32_t	pca9555Identify(uint8_t eChan, uint8_t Addr) {
	sPCA9555.sI2Cdev.chanI2C	= eChan ;
	sPCA9555.sI2Cdev.addrI2C	= Addr ;
	sPCA9555.sI2Cdev.dlayI2C	= pdMS_TO_TICKS(10) ;
	pca9555AllInputs(&sPCA9555) ;						// Configure all as inputs
	sPCA9555.Regs[regPCA9555_POL] = 0x0000 ;			// and ensure none polarity inverted
	pca9555WriteRegister(&sPCA9555, regPCA9555_POL) ;

	pca9555ReadRegister(&sPCA9555, regPCA9555_IN) ;
	uint16_t RegStart = sPCA9555.Regs[regPCA9555_IN] ;	// save the pre-invert status

	sPCA9555.Regs[regPCA9555_POL] = 0xFFFF ;			// then invert all pins
	pca9555WriteRegister(&sPCA9555, regPCA9555_POL) ;

	pca9555ReadRegister(&sPCA9555, regPCA9555_IN) ;		// read back the (inverted) status
	if (sPCA9555.Regs[regPCA9555_IN] != (RegStart ^ 0xFFFF)) {	// check if it is as expected..
		sPCA9555.sI2Cdev.chanI2C	= 0 ;
		sPCA9555.sI2Cdev.addrI2C	= 0 ;
		sPCA9555.sI2Cdev.dlayI2C	= 0 ;
		return erFAILURE ;
	}
	sPCA9555.sI2Cdev.epidI2C.devclass	= devPCA9555 ;
	sPCA9555.sI2Cdev.epidI2C.subclass	= subGPIO ;
	sPCA9555.sI2Cdev.epidI2C.epuri		= URI_UNKNOWN ;
	sPCA9555.sI2Cdev.epidI2C.epunit		= UNIT_UNKNOWN ;
	return erSUCCESS ;
}

int32_t	pca9555Config(void) {
	pca9555Reset(&sPCA9555) ;
	IF_SYSTIMER_INIT(debugTIMING && (systimerPCA9555 < 31), systimerPCA9555, systimerCLOCKS, "PCA9555", myUS_TO_CLOCKS(300), myUS_TO_CLOCKS(30000)) ;
	return erSUCCESS ;
}

#define	pcaCHECK_INTERVAL				(30 * MILLIS_IN_SECOND)
uint32_t	pcaSuccessCount, pcaResetCount, pcaCheckInterval ;

int32_t	pca9555Check(uint32_t tIntvl) {
	IF_myASSERT(debugPARAM, sPCA9555.sI2Cdev.addrI2C != 0) ;
	pcaCheckInterval += pdMS_TO_TICKS(tIntvl) ;
	if ((pcaCheckInterval % pcaCHECK_INTERVAL) >= pdMS_TO_TICKS(tIntvl)) {
		return 0 ;
	}
	pca9555ReadRegister(&sPCA9555, regPCA9555_IN) ;
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

	pca9555WriteRegister(&sPCA9555, regPCA9555_CFG) ;
	pca9555WriteRegister(&sPCA9555, regPCA9555_POL) ;
	pca9555WriteRegister(&sPCA9555, regPCA9555_OUT) ;
	SL_ERR("I2C Recover done, ok=%d vs %d", pcaSuccessCount, ++pcaResetCount) ;
	return 1 ;
}
