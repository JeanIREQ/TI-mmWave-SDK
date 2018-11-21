/*
 *   @file  main.c
 *
 *   @brief
 *      Unit Test code for CAN
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2016 Texas Instruments, Inc.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**************************************************************************
 *************************** Include Files ********************************
 **************************************************************************/

/* Standard Include Files. */
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

/* BIOS/XDC Include Files. */
#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/IHeap.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/Memory.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/heaps/HeapBuf.h>
#include <ti/sysbios/heaps/HeapMem.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/family/arm/v7a/Pmu.h>

/* mmWave SK Include Files: */
#include <ti/common/sys_common.h>
#include <ti/drivers/soc/soc.h>
#include <ti/drivers/can/can.h>
#include <ti/drivers/pinmux/pinmux.h>
#include <ti/drivers/esm/esm.h>
#include <ti/utils/testlogger/logger.h>
#include <ti/drivers/osal/HwiP.h>
#include <ti/drivers/gpio/gpio.h>
#include <ti/drivers/uart/UART.h>

/**************************************************************************
 *************************** Global Definitions ***************************
 **************************************************************************/

/** \brief Number of messages sent */
#define DCAN_APP_TEST_MESSAGE_COUNT     100

/** \brief DCAN input clock - 20MHz */
#define DCAN_APP_INPUT_CLK              (40000000U)

/** \brief DCAN output bit rate - 1MHz */
#define DCAN_APP_BIT_RATE               (1000000U)
/** \brief DCAN Propagation Delay - 700ns */
#define DCAN_APP_PROP_DELAY             (700U)
/** \brief DCAN Sampling Point - 70% */
#define DCAN_APP_SAMP_PT                (70U)

/** \brief DCAN TX message object used */
#define DCAN_TX_MSG_OBJ                 (0x1U)
/** \brief DCAN RX message object used */
#define DCAN_RX_MSG_OBJ                 (0x7U)

/* Global Variables */
volatile uint32_t       gTxDoneFlag = 0, gRxDoneFlag = 0, gParityErrFlag = 0;
volatile uint32_t                iterationCount = 0U;
volatile uint32_t       gTxPkts = 0, gRxPkts = 0, gErrStatusInt = 0;
CAN_DCANCfgParams       appDcanCfgParams;
CAN_DCANMsgObjCfgParams appDcanTxCfgParams;
CAN_DCANMsgObjCfgParams appDcanRxCfgParams;
CAN_DCANBitTimeParams   appDcanBitTimeParams;
CAN_DCANData            appDcanTxData;
CAN_DCANData            appDcanRxData;
uint32_t                dataLength = 0U;
uint32_t                msgLstErrCnt = 0U;
uint32_t                dataMissMatchErrCnt = 0U;
uint32_t                rxTicks[DCAN_APP_TEST_MESSAGE_COUNT];
uint32_t                txTicks[DCAN_APP_TEST_MESSAGE_COUNT];
uint32_t                minRxTicks;
uint32_t                maxRxTicks;
uint32_t                minTxTicks;
uint32_t                maxTxTicks;
uint32_t                totalTxTicks;
uint32_t                totalRxTicks;

UART_Handle logUartHandle;

void sendRemote(const char *message)
{
    UART_writePolling(logUartHandle, (uint8_t*)message, strlen(message));
}

void sendRemote1(const char *message, int a)
{
    char text[1024];
    sprintf(text, message, a);
    UART_writePolling(logUartHandle, (uint8_t*)text, strlen(text));
}

void sendRemote2(const char *message, int a, int b)
{
    char text[1024];
    sprintf(text, message, a, b);
    UART_writePolling(logUartHandle, (uint8_t*)text, strlen(text));
}

/**
 *  @b Description
 *  @n
 *      This function starts the PMU counter.
 *
 *   @param[in] counter
 *      Counter id used for benchmarking
 *
 *  @retval
 *      Not Applicable.
 */
void Test_benchmarkStart(uint32_t counter){
    /* Initialize counter to count cycles */
    Pmu_configureCounter(counter, 0x11, FALSE);

    /* Reset PMU counter */
    Pmu_resetCount(counter);

    /* Start PMU counter */
    Pmu_startCounter(counter);
}

/**
 *  @b Description
 *  @n
 *      This function stops a PMU counter and returns the current
 *      counter value.
 *
 *   @param[in] counter
 *      Counter id used for benchmarking
 *
 *  @retval
 *      Current PMU counter value.
 */
uint32_t Test_benchmarkStop(uint32_t counter){
    /* Stop PMU counter */
    Pmu_stopCounter(counter);

    /* Read PMU counter */
    return (Pmu_getCount(counter));
}

/**
 *  @b Description
 *  @n
 *      Application implemented callback function to handle error and status interrupts.
 *
 *   @param[in] handle
 *      Handle to the CAN Driver
 *   @param[in] errStatusResp
 *      Response structure containing the Error and status information
 *
 *  @retval
 *      Not Applicable.
 */
static void DCANAppErrStatusCallback(CAN_Handle handle, CAN_ErrStatusResp* errStatusResp){
    gErrStatusInt++;
    if (errStatusResp->parityError == 1){
        gParityErrFlag = 1;
    }
    return;
}

/**
 *  @b Description
 *  @n
 *      Application implemented callback function to handle Tx complete and receive interrupts.
 *
 *   @param[in] handle
 *      Handle to the message object
 *   @param[in] msgObjectNum
 *      Message object number
 *   @param[in] direction
 *      Direction of the object number
 *
 *  @retval
 *      Not Applicable.
 */
static void DCANAppCallback(CAN_MsgObjHandle handle, uint32_t msgObjectNum, CAN_Direction direction){
    int32_t     errCode, retVal;

    if (direction == CAN_Direction_TX){
       // if (msgObjectNum != DCAN_TX_MSG_OBJ)
       // {
       //     System_printf ("Error: Tx callback received for incorrect Message Object %d\n", msgObjectNum);
       //     return;
       // }
       // else
        {
            gTxPkts++;
            gTxDoneFlag = 1;
            return;
        }
    }
    if (direction == CAN_Direction_RX){
      //  if (msgObjectNum != DCAN_RX_MSG_OBJ)
      //  {
      //      System_printf ("Error: Rx callback received for incorrect Message Object %d\n", msgObjectNum);
      //      return;
      //  }
      //  else
        {
            /* Reset the receive buffer */
            memset(&appDcanRxData, 0, sizeof (appDcanRxData));
            dataLength = 0;

            if (gRxPkts < DCAN_APP_TEST_MESSAGE_COUNT){
                /* Reset the counter: */
                Test_benchmarkStart(0);
            }
            retVal = CAN_getData (handle, &appDcanRxData, &errCode);
            if (retVal < 0){
                sendRemote2("Error: CAN receive data for iteration %d failed [Error code %d]\n", iterationCount, errCode);
                return;
            }

            if (gRxPkts < DCAN_APP_TEST_MESSAGE_COUNT){
                /* Stop the counter: */
                rxTicks[gRxPkts] = Test_benchmarkStop(0);

                /* Update the receive statistics: */
                minRxTicks   = (minRxTicks < rxTicks[gRxPkts]) ? minRxTicks : rxTicks[gRxPkts];
                maxRxTicks   = (maxRxTicks > rxTicks[gRxPkts]) ? maxRxTicks : rxTicks[gRxPkts];
                totalRxTicks = totalRxTicks + rxTicks[gRxPkts];
            }

            /* Check if sent data is lost or not */
            if (appDcanRxData.msgLostFlag == 1){
                msgLstErrCnt++;
            }

            gRxPkts++;
            gRxDoneFlag = 1;
            return;
        }
    }
}

/**
 * \brief   This function takes I/P Clk frequency, required bit-rate, reference
 *          sampling point, propagation delayon the CAN bus and calculates the
 *          value to be programmed for DCAN BTR register.
 *          This API doesn't do the actual programming. This is
 *          intended to be used as a utility function. And the application
 *          should call the #DCANSetBitTime function to do the actual
 *          programming.
 *
 * \param   clkFreq       I/P clock frequency to DCAN controller in terms of MHz
 * \param   bitRate       Required bit-rate on the CAN bus in KHz
 * \param   refSamplePnt  Reference Sampling point in terms of %
 * \param   propDelay     Required Propagation delay in terms of ns
 * \param   bitTimeParams  Pointer to params where the calculated register
 *                        fields are populated
 *
 * \return  Returns the error status information as STW_SOK for success and
 *          STW_EFAIL when no valid baudrate calculation possible for the
 *          configured baudrate and propagation delay
 */
int32_t DCANAppCalcBitTimeParams(uint32_t               clkFreq,
                                uint32_t                bitRate,
                                uint32_t                refSamplePnt,
                                uint32_t                propDelay,
                                CAN_DCANBitTimeParams*  bitTimeParams){
    Double  tBitRef = 1000 * 1000 / bitRate;
    Double  newBaud = 0, newNProp = 0, newNSeg = 0, newSjw = 0, newP = 0;
    Double  nQRef, nProp, fCan, nQ, nSeg, baud, sp, p, newSp = 0;
    float   tQ;

    for (p = 1; p <= 1024; p++){
        tQ    = ((p / clkFreq) * 1000.0);
        nQRef = tBitRef / tQ;

        if ((nQRef >= 8) && (nQRef <= 25)){
            nProp = ceil(propDelay / tQ);
            fCan  = clkFreq / p;
            nQ    = fCan / bitRate * 1000;
            nSeg  = ceil((nQ - nProp - 1) / 2);

            if ((nProp <= 8) && (nProp > 0) && (nSeg <= 8) && (nSeg > 0)){
                baud = fCan / (1 + nProp + 2 * nSeg) * 1000;

                sp = (1 + nProp + nSeg) / (1 + nProp + nSeg + nSeg) * 100;

                if ((abs(baud - bitRate)) < (abs(newBaud - bitRate))){
                    newBaud  = baud;
                    newNProp = nProp;
                    newNSeg  = nSeg;
                    newSjw   = (nSeg < 4) ? nSeg : 4;
                    newP     = p - 1;
                    newSp    = sp;
                }
                else if ((abs(baud - bitRate)) == (abs(newBaud - bitRate))){
                    if ((abs(sp - refSamplePnt)) < (abs(newSp - refSamplePnt))){
                        newBaud  = baud;
                        newNProp = nProp;
                        newNSeg  = nSeg;
                        newSjw   = (nSeg < 4) ? nSeg : 4;
                        newP     = p - 1;
                        newSp    = sp;
                    }
                }
            }
        }
    }
    if ((newBaud == 0) || (newBaud > 1000)){
        return -1;
    }

    bitTimeParams->baudRatePrescaler    = (((uint32_t) newP) & 0x3F);
    bitTimeParams->baudRatePrescalerExt =
        ((((uint32_t) newP) & 0x3C0) ? (((uint32_t) newP) & 0x3C0) >> 6 : 0);
    bitTimeParams->syncJumpWidth = ((uint32_t) newSjw) - 1;

    /* propSeg = newNProp, phaseSeg = newNSeg, samplePoint = newSp
     * nominalBitTime = (1 + newNProp + 2 * newNSeg), nominalBitRate = newBaud
     * brpFreq  = clkFreq / (brp + 1), brpeFreq = clkFreq / (newP + 1)
     * brp      = bitTimeParams->baudRatePrescaler;
     */

    bitTimeParams->timeSegment1 = newNProp + newNSeg - 1;
    bitTimeParams->timeSegment2 = newNSeg - 1;

    return 0;
}

/**
 *  @b Description
 *  @n
 *      Platform specific intializations.
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */

static int32_t PlatformInit(void){
    int32_t         errCode;
    SOC_Handle      socHandle;
    SOC_Cfg         socCfg;
    sendRemote ("1");

    /* Initialize the SOC confiugration: */
    memset ((void *)&socCfg, 0, sizeof(SOC_Cfg));
    sendRemote ("2");

    /* Populate the SOC configuration: */
    socCfg.clockCfg = SOC_SysClock_INIT;
    sendRemote ("3");

    /* Initialize the SOC Module: This is done as soon as the application is started
     * to ensure that the MPU is correctly configured. */
    socHandle = SOC_init (&socCfg, &errCode);
    if (socHandle == NULL){
        sendRemote1 ("Error: SOC Module Initialization failed [Error code %d]\n", errCode);
        return -1;
    }
    sendRemote ("4");

    /* Setup the PINMUX to bring out the XWR14xx CAN pins */
    Pinmux_Set_OverrideCtrl(SOC_XWR14XX_PINP5_PADAE, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
    Pinmux_Set_FuncSel(SOC_XWR14XX_PINP5_PADAE, SOC_XWR14XX_PINP5_PADAE_CAN_TX);
    Pinmux_Set_OverrideCtrl(SOC_XWR14XX_PINR8_PADAD, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
    Pinmux_Set_FuncSel(SOC_XWR14XX_PINR8_PADAD, SOC_XWR14XX_PINR8_PADAD_CAN_RX);

Pinmux_Set_OverrideCtrl(SOC_XWR14XX_PINN4_PADAB, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
Pinmux_Set_FuncSel(SOC_XWR14XX_PINN4_PADAB, SOC_XWR14XX_PINN4_PADAB_GPIO_0);

    sendRemote ("5");

    /* Configure the divide value for DCAN source clock */
    SOC_setPeripheralClock(socHandle, SOC_MODULE_DCAN, SOC_CLKSOURCE_VCLK, 4U, &errCode);
    sendRemote ("6");

    /* Initialize peripheral memory */
    SOC_initPeripheralRam(socHandle, SOC_MODULE_DCAN, &errCode);
    sendRemote ("7");

    return 0;
}

static void DCANAppInitParams(CAN_DCANCfgParams*        dcanCfgParams,
                              CAN_DCANMsgObjCfgParams*  dcanTxCfgParams,
                              CAN_DCANMsgObjCfgParams*  dcanRxCfgParams,
                              CAN_DCANData*             dcanTxData){
    /*Intialize DCAN Config Params*/
    dcanCfgParams->parityEnable         = 0;
    dcanCfgParams->intrLine0Enable      = 1;
    dcanCfgParams->intrLine1Enable      = 1;
    dcanCfgParams->eccModeEnable        = 0;
    dcanCfgParams->testModeEnable       = 1;
    dcanCfgParams->testMode = CAN_DCANTestMode_EXT_LPBACK;
    dcanCfgParams->stsChangeIntrEnable  = 1;
    
    dcanCfgParams->autoRetransmitDisable = 1;
    dcanCfgParams->autoBusOnEnable       = 0;
    dcanCfgParams->errIntrEnable         = 1;
    dcanCfgParams->autoBusOnTimerVal     = 0;
    dcanCfgParams->if1DmaEnable          = 0;
    dcanCfgParams->if2DmaEnable          = 0;
    dcanCfgParams->if3DmaEnable          = 0;
    dcanCfgParams->ramAccessEnable       = 0;
    dcanCfgParams->appCallBack           = DCANAppErrStatusCallback;

    /*Intialize DCAN tx Config Params*/
    dcanTxCfgParams->xIdFlagMask       = 0x1;
    dcanTxCfgParams->dirMask           = 0x1;
    dcanTxCfgParams->msgIdentifierMask = 0x1FFFFFFF;

    dcanTxCfgParams->msgValid      = 1;
    dcanTxCfgParams->xIdFlag       = CAN_DCANXidType_11_BIT;
    dcanTxCfgParams->direction     = CAN_Direction_TX;
    dcanTxCfgParams->msgIdentifier = 0xC1;
    //dcanTxCfgParams->msgIdentifier = 0x73;

    dcanTxCfgParams->uMaskUsed    = 1;
    dcanTxCfgParams->intEnable    = 1;

    dcanTxCfgParams->remoteEnable = 0;
    dcanTxCfgParams->fifoEOBFlag  = 1;
    dcanTxCfgParams->appCallBack  = DCANAppCallback;

    /*Intialize DCAN Rx Config Params*/
    dcanRxCfgParams->xIdFlagMask       = 0x1;
    //dcanRxCfgParams->msgIdentifierMask = 0x1FFFFFFF;
    dcanRxCfgParams->msgIdentifierMask = 0x0;
    dcanRxCfgParams->dirMask           = 0x1;

    dcanRxCfgParams->msgValid      = 1;
    dcanRxCfgParams->xIdFlag       = CAN_DCANXidType_11_BIT;
    dcanRxCfgParams->direction     = CAN_Direction_RX;
    //dcanRxCfgParams->msgIdentifier = 0xC1;
    dcanRxCfgParams->msgIdentifier = 0x99;

    dcanRxCfgParams->uMaskUsed    = 1;
    dcanRxCfgParams->intEnable    = 1;

    dcanRxCfgParams->remoteEnable = 0;
    dcanRxCfgParams->fifoEOBFlag  = 1;
    dcanRxCfgParams->appCallBack  = DCANAppCallback;

    /*Intialize DCAN Tx transfer Params*/
    sendRemote ("Configuring the CAN");
    dcanTxData->dataLength = DCAN_MAX_MSG_LENGTH;
    dcanTxData->msgData[0] = 0xDE;
    dcanTxData->msgData[1] = 0xAD;
    dcanTxData->msgData[2] = 0xBE;
    dcanTxData->msgData[3] = 0xEF;
    dcanTxData->msgData[4] = 0xBA;
    dcanTxData->msgData[5] = 0xBE;
    dcanTxData->msgData[6] = 0xDE;
    dcanTxData->msgData[7] = 0xAD;
}

static int32_t dcanTransmitTest(){
    CAN_Handle              canHandle;
    CAN_MsgObjHandle        txMsgObjHandle[0x0F];
    CAN_MsgObjHandle        rxMsgObjHandle;
    int32_t                 retVal = 0;
    int32_t                 errCode = 0;
	volatile int32_t j;

    /* Initialize the DCAN parameters that need to be specified by the application */
    DCANAppInitParams(&appDcanCfgParams,
                      &appDcanTxCfgParams,
                      &appDcanRxCfgParams,
                      &appDcanTxData);
    sendRemote ("1\n");


    /* Initialize the CAN driver */
    canHandle = CAN_init(&appDcanCfgParams, &errCode);
    if (canHandle == NULL){
        sendRemote1 ("Error: CAN Module Initialization failed [Error code %d]\n", errCode);
        return -1;
    }
    sendRemote ("2\n");


    /* Set the desired bit rate based on input clock */
    retVal = DCANAppCalcBitTimeParams(DCAN_APP_INPUT_CLK / 1000000,
                                            DCAN_APP_BIT_RATE / 1000,
                                            DCAN_APP_SAMP_PT,
                                            DCAN_APP_PROP_DELAY,
                                            &appDcanBitTimeParams);
    if (retVal < 0){
        sendRemote ("Error: CAN Module bit time parameters are incorrect \n");
        return -1;
    }
    sendRemote ("3\n");


    /* Configure the CAN driver */
    retVal = CAN_configBitTime (canHandle, &appDcanBitTimeParams, &errCode);
    if (retVal < 0){
        sendRemote1 ("Error: CAN Module configure bit time failed [Error code %d]\n", errCode);
        return -1;
    }
    sendRemote ("4\n");
   

    /* Setup the receive message object */
    rxMsgObjHandle = CAN_createMsgObject (canHandle, DCAN_RX_MSG_OBJ, &appDcanRxCfgParams, &errCode);
    if (rxMsgObjHandle == NULL){
        sendRemote1 ("Error: CAN create Rx message object failed [Error code %d]\n", errCode);
        return -1;
    }
    sendRemote ("5\n");


	for( j = 1; j <= 10;j++){
		appDcanTxCfgParams.msgIdentifier = 0x73 + j;
	    sendRemote ("6\n");
				
		
		txMsgObjHandle[j] = CAN_createMsgObject (canHandle, j, &appDcanTxCfgParams, &errCode);
		if (txMsgObjHandle[j] == NULL){
			sendRemote1 ("Error: CAN create Tx message object failed [Error code %d]\n", errCode);
			return -1;
		}
	}
    sendRemote ("7\n");


	j = 1;
    while (1){
	    sendRemote ("8\n");


		if(j>10){
		    sendRemote ("9\n");
			j=1;
		}
		/* Send data over Tx message object */
        retVal = CAN_transmitData (txMsgObjHandle[j], &appDcanTxData, &errCode);
	    sendRemote1 ("10: %d\n", retVal);

        if (retVal < 0){
            sendRemote2 ("Error: CAN transmit data for iteration %d failed [Error code %d]\n", iterationCount, errCode);
            return -1;
        }
        iterationCount++;
        j++;
		Task_sleep(1);
    }
}

/**
 *  @b Description
 *  @n
 *      System Initialization Task which initializes the various
 *      components in the system.
 *
 *  @retval
 *      Not Applicable.
 */
static void Test_initTask(UArg arg0, UArg arg1)
{
    int32_t         retVal = 0;

    /* Initialize the test logger framework: */
    //MCPI_Initialize ();

/* Initialize the GPIO */
GPIO_init();
GPIO_setConfig(SOC_XWR14XX_GPIO_0, GPIO_CFG_OUTPUT);
GPIO_write(SOC_XWR14XX_GPIO_0,0);

    /* Initialize the platform */
    retVal = PlatformInit();
    if (retVal < 0){
        sendRemote ("HEY it screwed up\n");
    }

    sendRemote("Debug: External transmit testing\n");
    retVal = dcanTransmitTest();
    if (retVal == -1)
        System_printf("External transmit testing", MCPI_TestResult_FAIL);
    else
        System_printf("External transmit testing", MCPI_TestResult_PASS);

    if (retVal < 0){
        sendRemote("Debug:CAN testing failed\n");
    }

    /* Exit BIOS */
    BIOS_exit(0);

    return;
}

void MmwDemo_sleep(void){
    /* issue WFI (Wait For Interrupt) instruction */
    asm(" WFI ");
}

/**
 *  @b Description
 *  @n
 *      This is the entry point into the unit test code
 *
 *  @retval
 *      Not Applicable.
 */
int32_t main (void)
{
	Task_Params taskParams;
    UART_Params uartParams;

    /* Initialize the ESM: Dont clear errors as TI RTOS does it */
    ESM_init(0U);
UART_init();
Pinmux_Set_OverrideCtrl(SOC_XWR14XX_PINN6_PADBE, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
Pinmux_Set_FuncSel(SOC_XWR14XX_PINN6_PADBE, SOC_XWR14XX_PINN6_PADBE_MSS_UARTA_TX);
Pinmux_Set_OverrideCtrl(SOC_XWR14XX_PINN5_PADBD, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
Pinmux_Set_FuncSel(SOC_XWR14XX_PINN5_PADBD, SOC_XWR14XX_PINN5_PADBD_MSS_UARTA_RX);
/* Setup the default UART Parameters */
UART_Params_init(&uartParams);
uartParams.clockFrequency = (200 * 1000000);
uartParams.baudRate       = 115200;
uartParams.isPinMuxDone   = 1;
logUartHandle = UART_open(0, &uartParams);

    /* Initialize the Task Parameters. */
    Task_Params_init(&taskParams);
    taskParams.stackSize = 6*1024;
    Task_create(Test_initTask, &taskParams, NULL);

    /* Start BIOS */
	BIOS_start();
    return 0;
}
