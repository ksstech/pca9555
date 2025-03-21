// pca9555.c - Copyright (c) 2014-25 Andre M. Maree / KSS Technologies (Pty) Ltd.

#include "hal_platform.h"

#if (HAL_PCA9555 > 0)
#include "hal_i2c_common.h"
#include "errors_events.h"
#include "pca9555.h"
#include "printfx.h"
#include "syslog.h"
#include "systiming.h"

#define	debugFLAG					0xF000

#define	debugTIMING					(debugFLAG_GLOBAL & debugFLAG & 0x1000)
#define	debugTRACK					(debugFLAG_GLOBAL & debugFLAG & 0x2000)
#define	debugPARAM					(debugFLAG_GLOBAL & debugFLAG & 0x4000)
#define	debugRESULT					(debugFLAG_GLOBAL & debugFLAG & 0x8000)

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

// ######################################### Local variables #######################################

pca9555_t sPCA9555 = { 0 };
const char * const DS9555RegNames[] = { "Input", "Output", "PolInv", "Config" };

#if (appPLTFRM == HW_AC01)								/* defaults for both AC0x */
	static const u16_t pca9555Out = 0b0000000000000000;		/* all 0=OFF */
	static const u16_t pca9555Pol = 0b0000000000000000;		/* all NON inverted */
	static const u16_t pca9555Cfg = 0b0000000000000000;		/* all outputs */
#endif

// ####################################### Local functions #########################################

static int pca9555ReadRegister(u8_t Reg) {
	u8_t cChr = Reg << 1;								// force to u16_t boundary 0/2/4/6
	// Adding a delay of 0mS ensure that write & read operations are separately executed
	return halI2C_Queue(sPCA9555.psI2C, i2cWDR_FB, &cChr, sizeof(cChr), (u8_t *) &sPCA9555.Regs[Reg], sizeof(u16_t), (i2cq_p1_t) NULL, (i2cq_p2_t) NULL);
}

static int pca9555WriteRegister(u8_t Reg, u16_t Val) {
	sPCA9555.Regs[Reg] = Val;
	u8_t cBuf[3];
	cBuf[0] = Reg << 1;									// force to u16_t boundary 0 / 2 / 4 / 6
	cBuf[1] = Val >> 8;
	cBuf[2] = Val & 0xFF;
	int iRV = halI2C_Queue(sPCA9555.psI2C, i2cW_FB, cBuf, sizeof(cBuf), (u8_t *) NULL, 0, (i2cq_p1_t) NULL, (i2cq_p2_t) NULL);
	if (Reg == pca9555_OUT && iRV == erSUCCESS)
		sPCA9555.fDirty = 0;							// show as clean, just written
	return iRV;
}

void pca9555Reset(void) {
	pca9555WriteRegister(pca9555_CFG, pca9555Cfg);
	pca9555WriteRegister(pca9555_POL, pca9555Pol);
	pca9555WriteRegister(pca9555_OUT, pca9555Out);
}

// ###################################### Global functions #########################################

int pca9555Flush(void) {
	if (sPCA9555.fDirty) {
		pca9555WriteRegister(pca9555_OUT, sPCA9555.Regs[pca9555_OUT]);
		return 1;
	}
	return 0;
}


int pca9555Function(pca9555func_e Func, u8_t Pin, bool NewState) {
	IF_myASSERT(debugPARAM, Pin < pca9555NUM_PINS && (Func < pca9555FUNC));
	#if (appPLTFRM == HW_AC01)
	if (sSysFlags.ac00 && Pin < 8)						// AC01 pins 0->7 map to 7->0 on AC00
		Pin = 7 - Pin;
	#endif
	u8_t Mask = 1 << Pin;
	if (Func >= stateTGL_LAZY) {						// All OUTput pin only function
		IF_myASSERT(debugTRACK, (sPCA9555.Regs[pca9555_CFG] & Mask) == 0);
		bool CurState = (sPCA9555.Regs[pca9555_OUT] & Mask) ? 1 : 0;
		if (Func <= stateTGL) {							// stateTGL[_LAZY]
			NewState = sPCA9555.Regs[pca9555_OUT] & Mask ? 0 : 1;
			Func += (stateSET_LAZY - stateTGL_LAZY);	// adjust stateTGL[_LAZY] -> stateSET[_LAZY]
		}
		if (NewState != CurState) {
			if (NewState)	sPCA9555.Regs[pca9555_OUT] |= Mask;
			else			sPCA9555.Regs[pca9555_OUT] &= ~Mask;
			sPCA9555.fDirty = 1;						// bit just changed, show as dirty
		}
		return (Func <= stateSET_LAZY) ? sPCA9555.fDirty : pca9555Flush();

	} else if (Func == stateGET) {
		if (sPCA9555.Regs[pca9555_CFG] & Mask) {		// configured as INput ?
			int iRV = pca9555ReadRegister(pca9555_IN);
			return (iRV == erSUCCESS) ? ((sPCA9555.Regs[pca9555_IN] & Mask) ? 1 : 0) : xSyslogError(__FUNCTION__, iRV);
		} else {										// configured as OUTput
			return (sPCA9555.Regs[pca9555_OUT] & Mask) ? 1 : 0;
		}

	} else if (Func == cfgINV) {						// MUST be INput
		IF_myASSERT(debugTRACK, (sPCA9555.Regs[pca9555_CFG] & Mask) == 1);
		pca9555WriteRegister(pca9555_POL, sPCA9555.Regs[pca9555_POL] ^ Mask);

	} else if (Func == cfgDIR) {						// Direction INput vs OUTput
		if (NewState)	sPCA9555.Regs[pca9555_CFG] |= Mask;
		else			sPCA9555.Regs[pca9555_CFG] &= ~Mask;
		pca9555WriteRegister(pca9555_CFG, sPCA9555.Regs[pca9555_CFG]);
	}
	return 0;
}

// ################################## Diagnostics functions ########################################


// Due to an induced reverse voltage cause by the collapsing magnetic field of the solenoid in the
// door striker or water valve it can cause the I2C bus to "hang". In order to resolve this we need
// to check that the PCA9555 can be read and that the value read back corresponds
// with the last value written. If not, FSM of I2C peripheral on the ESP32 must be reset completely
#define	pcaCHECK_INTERVAL	5
u32_t pcaSuccessCount, pcaResetCount, pcaCheckInterval;

int	pca9555Verify(void) {
	++pcaCheckInterval;
	if ((pcaCheckInterval % pcaCHECK_INTERVAL) == 0)
		return 0;
	pca9555ReadRegister(pca9555_IN);					// Time to do a check
	u16_t RegInInv = sPCA9555.Reg_IN;
	// AMM not sure the logic behind this....
	if (appPLTFRM == HW_AC01)
		RegInInv = (RegInInv >> 8) | (RegInInv << 8);
	if (RegInInv == sPCA9555.Reg_OUT) {
		++pcaSuccessCount;								// all OK, no reset required...
		return 0; 
	}
	++pcaResetCount;
	u16_t ErrorBits = RegInInv ^ sPCA9555.Reg_OUT;		// Determine bits that are wrong
	SL_NOT("Rin=x%04hX  Rout=x%04hX  Diff=x%04hX  Err=%lu vs %lu", RegInInv, sPCA9555.Reg_OUT, ErrorBits, pcaResetCount, pcaSuccessCount);
	// general reset, reconfigure and start again...
	halI2C_ErrorHandler(sPCA9555.psI2C, __FUNCTION__, ESP_FAIL);	/* error code chosen to force FSM reset */
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
	int iRV = pca9555WriteRegister(pca9555_POL, 0);		// default non inverted/normal
	if (iRV < erSUCCESS)
		return iRV;
	iRV = pca9555WriteRegister(pca9555_CFG, 0xFFFF);	// default all Inputs
	if (iRV < erSUCCESS)
		return iRV;
	// Step 2 - read all registers
	for (int r = pca9555_IN; r < pca9555_NUM; ++r) {
		 iRV = pca9555ReadRegister(r);
		 if (iRV < erSUCCESS)
		 	return iRV;
	}
	// Step 3 - Check initial default values
	if (sPCA9555.Regs[pca9555_POL] != 0 || sPCA9555.Regs[pca9555_CFG] != 0xFFFF)
		return erINV_WHOAMI;
	u16_t OrigOUT = sPCA9555.Regs[pca9555_OUT];			// passed phase 1, now step 4
	pca9555WriteRegister(pca9555_CFG, 0);				// all OUTputs
	pca9555ReadRegister(pca9555_OUT);
	if (sPCA9555.Regs[pca9555_OUT] != OrigOUT)
		return erINV_WHOAMI;
	psI2C->IDok = 1;
	psI2C->Test	= 0;
	return iRV;
}

int	pca9555Config(i2c_di_t * psI2C) {
	if (psI2C->IDok == 0)
		return erINV_STATE;
	psI2C->CFGok = 0;
	halEventUpdateDevice(devMASK_PCA9555, 0);
	int iRV = pca9555WriteRegister(pca9555_CFG, pca9555Cfg);	// IN vs OUT
	if (iRV < erSUCCESS)
		return iRV;
	iRV = pca9555WriteRegister(pca9555_POL, pca9555Pol);	// Non Invert
	if (iRV < erSUCCESS)
		return iRV;
	iRV = pca9555WriteRegister(pca9555_OUT, pca9555Out);	// All OUTputs
	if (iRV < erSUCCESS)
		return iRV;
	psI2C->CFGok = 1;
	sPCA9555.fDirty = 0;
	halEventUpdateDevice(devMASK_PCA9555, 1);
	// once off init....
	if (psI2C->CFGerr == 0)
		IF_SYSTIMER_INIT(debugTIMING, stPCA9555, stMICROS, "PCA9555", 200, 3200);
	return iRV;
}

int	pca9555Diagnostics(i2c_di_t * psI2C) {
	// configure as outputs and display
	wprintfx(NULL, "Default (all Outputs )status" strNL);
	pca9555WriteRegister(pca9555_CFG, 0x0000);
	vTaskDelay(pdMS_TO_TICKS(pca9555TEST_INTERVAL));

	// set all OFF and display
	wprintfx(NULL, "All outputs (OFF) status" strNL);
	pca9555WriteRegister(pca9555_OUT, 0x0000);
	vTaskDelay(pdMS_TO_TICKS(pca9555TEST_INTERVAL));

	// set all ON and display
	wprintfx(NULL, "All outputs (ON) status" strNL);
	pca9555WriteRegister(pca9555_OUT, 0xFFFF);
	vTaskDelay(pdMS_TO_TICKS(pca9555TEST_INTERVAL));

	// set all OFF and display
	wprintfx(NULL, "All outputs (OFF) status" strNL);
	pca9555WriteRegister(pca9555_OUT, 0x0000);
	vTaskDelay(pdMS_TO_TICKS(pca9555TEST_INTERVAL));

	// set all back to inputs and display
	wprintfx(NULL, "All Inputs (again) status" strNL);
	pca9555WriteRegister(pca9555_CFG, 0xFFFF);
	vTaskDelay(pdMS_TO_TICKS(pca9555TEST_INTERVAL));

	// Change INput to OUTput(0) and turn ON(1)
	wprintfx(NULL, "Config as Outputs 1 by 1, switch ON using SetState" strNL);
	for (u8_t Pin = 0; Pin < pca9555NUM_PINS; Pin++) {
		pca9555Direction(Pin, 0);					// default to OFF (0) after config
		pca9555SetState(Pin, 1);
		vTaskDelay(pdMS_TO_TICKS(pca9555TEST_INTERVAL));
	}

	// then switch them OFF 1 by 1 using TOGGLE functionality
	wprintfx(NULL, "Switch OFF 1 by 1 using TOGGLE" strNL);
	for (u8_t Pin = 0; Pin < pca9555NUM_PINS; ++Pin) {
		pca9555Toggle(Pin);
		vTaskDelay(pdMS_TO_TICKS(pca9555TEST_INTERVAL));
	}
	pca9555Reset();
	wprintfx(NULL, "Diagnostics completed. All LEDs = OFF !!!" strNL);
	return erSUCCESS;
}

int pca9555Report(report_t * psR) {
	int iRV = halI2C_DeviceReport(psR, (void *) sPCA9555.psI2C);
	iRV += wprintfx(psR, "Inp=0x%04hX  Out=0x%04hX  Pol=0x%04hX  Cfg=0x%04hx  OK=%lu  Fail=%lu" strNLx2,
			sPCA9555.Regs[pca9555_IN], sPCA9555.Regs[pca9555_OUT], sPCA9555.Regs[pca9555_POL],
			sPCA9555.Regs[pca9555_CFG], pcaSuccessCount, pcaResetCount);
	return iRV;
}

#endif
