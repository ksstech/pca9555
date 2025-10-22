// pca9555.c - Copyright (c) 2014-25 Andre M. Maree / KSS Technologies (Pty) Ltd.

#include "hal_platform.h"

#if (HAL_PCA9555 > 0)
#include "errors_events.h"
#include "pca9555.h"
#include "report.h"
#include "syslog.h"
#include "systiming.h"

// ########################################## MACROS ###############################################

#define	debugFLAG					0xF000
#define	debugFLUSH					(debugFLAG & 0x0001)
#define	debugTIMING					(debugFLAG_GLOBAL & debugFLAG & 0x1000)
#define	debugTRACK					(debugFLAG_GLOBAL & debugFLAG & 0x2000)
#define	debugPARAM					(debugFLAG_GLOBAL & debugFLAG & 0x4000)
#define	debugRESULT					(debugFLAG_GLOBAL & debugFLAG & 0x8000)

// ######################################## Enumerations ###########################################

// ######################################### Structures ############################################

// ######################################### Local variables #######################################

pca9555_t sPCA9555 = { 0 };
const char * const DS9555RegNames[] = { "Input", "Output", "PolInv", "Config" };

#if (cmakePLTFRM == HW_AC01) || (cmakePLTFRM == HW_RS01)	/* defaults for both AC0x and RS01 */
	static const u16_t pca9555Out = 0b0000000000000000;	/* all 0=OFF */
	static const u16_t pca9555Pol = 0b0000000000000000;	/* all NON inverted */
	static const u16_t pca9555Cfg = 0b0000000000000000;	/* all outputs */
#endif

// ####################################### Public variables ########################################

// ####################################### Local functions #########################################

/**
 * @brief	read 16bit value from specified register
 * @param[in]	Reg	number range 0->3
 * @return	result from the I2C operation
 */
static int pca9555ReadRegister(u8_t Reg) {
	u8_t cChr = Reg << 1;								// force to u16_t boundary 0/2/4/6
	// Adding a delay of 0mS ensure that write & read operations are separately executed
	return halI2C_Queue(sPCA9555.psI2C, i2cWDR_FB, &cChr, sizeof(cChr), (u8_t *) &sPCA9555.Regs[Reg], sizeof(u16_t), (i2cq_p1_t) NULL, (i2cq_p2_t) NULL);
}

/**
 * @brief	write 16bit value to specified register
 * @param[in]	Reg	number range 0->3
 * @return	result from the I2C operation
 */
static int pca9555WriteRegister(u8_t Reg) {
	u8_t cBuf[3];
	cBuf[0] = Reg << 1;									// force to u16_t boundary 0/2/4/6
	cBuf[1] = sPCA9555.Regs[Reg] >> 8;
	cBuf[2] = sPCA9555.Regs[Reg] & 0xFF;
	return halI2C_Queue(sPCA9555.psI2C, i2cW_FB, cBuf, sizeof(cBuf), (u8_t *) NULL, 0, (i2cq_p1_t) NULL, (i2cq_p2_t) NULL);
}

int pca9555Flush(void) {
	if (sPCA9555.fDirty) {
		IF_CPT(debugFLUSH, "Flush x%04X" strNL, sPCA9555.Regs[pca9555_OUT]);
		pca9555WriteRegister(pca9555_OUT);
		sPCA9555.fDirty = 0;							// show as clean, just written
		return 1;
	}
	return 0;
}

int pca9555Function(pca9555func_e Func, u8_t Pin, bool NewState) {
	IF_myASSERT(debugPARAM, Pin < pca9555NUM_PINS && (Func < pca9555FUNC));
	#if (cmakePLTFRM == HW_AC01)
	if (sSysFlags.ac00 && Pin < 8)						// AC01 pins 0->7 map as 7->0 on AC00
		Pin = 7 - Pin;
	#endif
	u16_t Mask = 1 << Pin;
	if (Func >= stateTGL_LAZY) {						// All OUTput pin only function
		IF_myASSERT(debugTRACK, (sPCA9555.Regs[pca9555_CFG] & Mask) == 0);
		bool CurState = (sPCA9555.Regs[pca9555_OUT] & Mask) ? 1 : 0;
		if (Func <= stateTGL) {							// stateTGL[_LAZY]
			NewState = !CurState;;						// calculate NewState
			Func += (stateSET_LAZY - stateTGL_LAZY);	// stateTGL[_LAZY] -> stateSET[_LAZY]
		}
		if (NewState != CurState) {
			if (NewState)	sPCA9555.Regs[pca9555_OUT] |= Mask;
			else			sPCA9555.Regs[pca9555_OUT] &= ~Mask;
			sPCA9555.fDirty = 1;						// bit just changed, show as dirty
		}
		return (Func < stateSET) ? sPCA9555.fDirty : pca9555Flush();

	} else if (Func == stateGET) {						// Can be INPut or OUTput....
		if (sPCA9555.Regs[pca9555_CFG] & Mask) {		// configured as INput ?
			int iRV = pca9555ReadRegister(pca9555_IN);	// read live status
			return (iRV == erSUCCESS) ? ((sPCA9555.Regs[pca9555_IN] & Mask) ? 1 : 0) : xSyslogError(__FUNCTION__, iRV);
		}
		return (sPCA9555.Regs[pca9555_OUT] & Mask) ? 1 : 0; // configured as OUTput

	} else if (Func == cfgINV) {						// MUST be INput
		IF_myASSERT(debugTRACK, (sPCA9555.Regs[pca9555_CFG] & Mask) == 1);
		sPCA9555.Regs[pca9555_POL] ^= Mask;
		pca9555WriteRegister(pca9555_POL);

	} else if (Func == cfgDIR) {						// Direction INput vs OUTput
		if (NewState)	sPCA9555.Regs[pca9555_CFG] |= Mask;
		else			sPCA9555.Regs[pca9555_CFG] &= ~Mask;
		pca9555WriteRegister(pca9555_CFG);
	}
	return 0;
}

// ################################## Diagnostics functions ########################################

// Due to an induced reverse voltage cause by the collapsing magnetic field of the solenoid in the
// door striker or water valve it can cause the I2C bus to "hang". In order to resolve this we need
// to check that the PCA9555 can be read and that the value read back corresponds
// with the last value written. If not, try to correct the value...
#define	pcaCHECK_INTERVAL	5
u32_t pcaSuccessCount, pcaResetCount, pcaCheckInterval;

int	pca9555Verify(void) {
	++pcaCheckInterval;
	if ((pcaCheckInterval % pcaCHECK_INTERVAL) == 0)
		return 0;
	pca9555ReadRegister(pca9555_IN);					// Time to do a check
	u16_t RegInInv = sPCA9555.Reg_IN;
	// AMM not sure the logic behind this....
	#if (cmakePLTFRM == HW_AC01) || (cmakePLTFRM == HW_RS01)
		RegInInv = (RegInInv >> 8) | (RegInInv << 8);
	#endif
	if (RegInInv == sPCA9555.Reg_OUT) {
		++pcaSuccessCount;								// all OK, no reset required...
		return 0; 
	}
	++pcaResetCount;
#if (appNEW_CODE == 1)
	// attempt to correct the error by rewriting the output register
	SL_NOT("Rout=x%04hX  Rinv=x%04hX  Err=%lu vs %lu", RegInInv, sPCA9555.Reg_OUT, pcaResetCount, pcaSuccessCount);
	pca9555WriteRegister(pca9555_OUT);
#else
	u16_t ErrorBits = RegInInv ^ sPCA9555.Reg_OUT;		// Determine bits that are wrong
	SL_NOT("Rin=x%04hX  Rout=x%04hX  Diff=x%04hX  Err=%lu vs %lu", RegInInv, sPCA9555.Reg_OUT, ErrorBits, pcaResetCount, pcaSuccessCount);
	halI2C_ResetSubSystem(sPCA9555.psI2C);				// general reset, reconfigure and start again...
#endif
	return 1;
}

#define	pca9555TEST_INTERVAL			300				// mSec delay between test stages

int	pca9555Identify(i2c_di_t * psI2C) {
	sPCA9555.psI2C = psI2C;
	psI2C->Type = i2cDEV_PCA9555;
	psI2C->Speed = i2cSPEED_400;
	psI2C->TObus = 25;
	psI2C->Test	= 1;
	// Step 1 - ensure all set to defaults
	sPCA9555.Regs[pca9555_POL] = 0x0000;				// non inverted/normal
	int iRV = pca9555WriteRegister(pca9555_POL);
	if (iRV < erSUCCESS)
		goto exit;
	sPCA9555.Regs[pca9555_CFG] = 0xFFFF;				// all inputs
	iRV = pca9555WriteRegister(pca9555_CFG);
	if (iRV < erSUCCESS)
		goto exit;
	// Step 2 - read all registers
	for (int r = pca9555_IN; r < pca9555_NUM; ++r) {
		 iRV = pca9555ReadRegister(r);
		 if (iRV < erSUCCESS)
		 	goto exit;
	}
	// Step 3 - Check initial default values
	if (sPCA9555.Regs[pca9555_POL] != 0 || sPCA9555.Regs[pca9555_CFG] != 0xFFFF)
		return erINV_WHOAMI;
	u16_t OrigOUT = sPCA9555.Regs[pca9555_OUT];			// passed steps 1~3, now step 4
	sPCA9555.Regs[pca9555_CFG] = 0x0000;				// all OUTputs
	pca9555WriteRegister(pca9555_CFG);
	pca9555ReadRegister(pca9555_OUT);
	if (sPCA9555.Regs[pca9555_OUT] != OrigOUT)
		return erINV_WHOAMI;
	psI2C->IDok = 1;
	psI2C->Test	= 0;
exit:
	return iRV;
}

int	pca9555Config(i2c_di_t * psI2C) {
	int iRV = erINV_STATE;
	if (psI2C->IDok == 0)
		goto exit;
	psI2C->CFGok = 0;
	halEventUpdateDevice(devMASK_PCA9555, 0);
	sPCA9555.Regs[pca9555_CFG] = pca9555Cfg;
	iRV = pca9555WriteRegister(pca9555_CFG);			// set required direction, IN vs OUT
	if (iRV < erSUCCESS)
		goto exit;
	sPCA9555.Regs[pca9555_POL] = pca9555Pol;
	iRV = pca9555WriteRegister(pca9555_POL);			// set required invertion status
	if (iRV < erSUCCESS)
		goto exit;
	sPCA9555.Regs[pca9555_OUT] = pca9555Out;
	iRV = pca9555WriteRegister(pca9555_OUT);			// set required output status (optional)
	if (iRV < erSUCCESS)
		goto exit;
	psI2C->CFGok = 1;
	sPCA9555.fDirty = 0;
	halEventUpdateDevice(devMASK_PCA9555, 1);
	// once off init....
	if (psI2C->CFGerr == 0)
		IF_SYSTIMER_INIT(debugTIMING, stPCA9555, stMICROS, "PCA9555", 200, 3200);
exit:
	return iRV;
}

int	pca9555Diagnostics(i2c_di_t * psI2C) {
	// configure as outputs and display
	PX("Default (all Outputs )status" strNL);
	sPCA9555.Regs[pca9555_CFG] = 0x0000;
	pca9555WriteRegister(pca9555_CFG);
	vTaskDelay(pdMS_TO_TICKS(pca9555TEST_INTERVAL));

	// set all OFF and display
	PX("All outputs (OFF) status" strNL);
	sPCA9555.Regs[pca9555_OUT] = 0x0000;
	pca9555WriteRegister(pca9555_OUT);
	vTaskDelay(pdMS_TO_TICKS(pca9555TEST_INTERVAL));

	// set all ON and display
	PX("All outputs (ON) status" strNL);
	sPCA9555.Regs[pca9555_OUT] = 0xFFFF;
	pca9555WriteRegister(pca9555_OUT);
	vTaskDelay(pdMS_TO_TICKS(pca9555TEST_INTERVAL));

	// set all OFF and display
	PX("All outputs (OFF) status" strNL);
	sPCA9555.Regs[pca9555_OUT] = 0x0000;
	pca9555WriteRegister(pca9555_OUT);
	vTaskDelay(pdMS_TO_TICKS(pca9555TEST_INTERVAL));

	// set all back to inputs and display
	PX("All Inputs (again) status" strNL);
	sPCA9555.Regs[pca9555_CFG] = 0xFFFF;
	pca9555WriteRegister(pca9555_CFG);
	vTaskDelay(pdMS_TO_TICKS(pca9555TEST_INTERVAL));

	// Change INput to OUTput(0) and turn ON(1)
	PX("Config as Outputs 1 by 1, switch ON using SetState" strNL);
	for (u8_t Pin = 0; Pin < pca9555NUM_PINS; Pin++) {
		pca9555Direction(Pin, 0);					// default to OFF (0) after config
		pca9555SetState(Pin, 1);
		vTaskDelay(pdMS_TO_TICKS(pca9555TEST_INTERVAL));
	}

	// then switch them OFF 1 by 1 using TOGGLE functionality
	PX("Switch OFF 1 by 1 using TOGGLE" strNL);
	for (u8_t Pin = 0; Pin < pca9555NUM_PINS; ++Pin) {
		pca9555Toggle(Pin);
		vTaskDelay(pdMS_TO_TICKS(pca9555TEST_INTERVAL));
	}
	PX("Diagnostics completed. All LEDs = OFF !!!" strNL);
	return erSUCCESS;
}

int pca9555Report(report_t * psR) {
	int iRV = halI2C_DeviceReport(psR, (void *) sPCA9555.psI2C);
	iRV += xReport(psR, "Inp=0x%04hX  Out=0x%04hX  Pol=0x%04hX  Cfg=0x%04hx  OK=%lu  Fail=%lu" strNLx2,
			sPCA9555.Regs[pca9555_IN], sPCA9555.Regs[pca9555_OUT], sPCA9555.Regs[pca9555_POL],
			sPCA9555.Regs[pca9555_CFG], pcaSuccessCount, pcaResetCount);
	return iRV;
}

#endif
