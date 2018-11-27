/*
 *   @file  cli.c
 *
 *   @brief
 *      CLI Utility implementation
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

/* BIOS/XDC Include Files. */
#include <xdc/std.h>
#include <ti/sysbios/knl/Task.h>

/* mmWave SDK Include Files: */
#include <ti/common/sys_common.h>
#include <ti/drivers/uart/UART.h>
#include <ti/utils/cli/cli.h>
#include <ti/utils/cli/include/cli_internal.h>

/**************************************************************************
 *************************** Global Variables *****************************
 **************************************************************************/

/**
 * @brief   Global variable which tracks the CLI MCB
 */
CLI_MCB gCLI;

char *manual_cmd[23] = {
        "sensorStop\n",
        "flushCfg\n",
        "dfeDataOutputMode 1\n",
        "channelCfg 15 5 0\n",
        "adcCfg 2 1\n",
        "adcbufCfg 0 1 0 1\n",
        "profileCfg 0 77 39 7 57.14 0 0 70 1 240 4884 0 0 30\n",
        "chirpCfg 0 0 0 0 0 0 0 1\n",
        "chirpCfg 1 1 0 0 0 0 0 4\n",
        "frameCfg 0 1 16 0 33.333 1 0\n",
        "guiMonitor 1 0 0 0 0 0\n",
        "cfarCfg 0 2 8 4 3 0 768\n",
        "peakGrouping 1 0 1 1 229\n",
        "multiObjBeamForming 1 0.5\n",
        "clutterRemoval 0\n",
        "calibDcRangeSig 0 -5 8 256\n",
        "compRangeBiasAndRxChanPhase 0.0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0\n",
        "measureRangeBiasAndRxChanPhase 0 1.5 0.2\n",
        "CQRxSatMonitor 0 3 5 123 0\n",
        "CQSigImgMonitor 0 119 4\n",
        "analogMonitor 1 1\n",
        "sensorStart\n",
        "\n"};
int nbManualCmd = 23;
int cntManualCmd = 0;

/**************************************************************************
 **************************** CLI Functions *******************************
 **************************************************************************/

/**
 *  @b Description
 *  @n
 *      The function is the HELP generated by the CLI Module
 *
 *  \ingroup CLI_UTIL_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not Applicable.
 */
static int32_t CLI_help (int32_t argc, char* argv[])
{
    uint32_t    index;

    /* Display the banner: */
    CLI_write ("Help: This will display the usage of the CLI commands\n");
    CLI_write ("Command: Help Description\n");

    /* Cycle through all the registered CLI commands: */
    for (index = 0; index < gCLI.numCLICommands; index++)
    {
        /* Display the help string*/
        CLI_write ("%s: %s\n",
                    gCLI.cfg.tableEntry[index].cmd,
                   (gCLI.cfg.tableEntry[index].helpString == NULL) ?
                    "No help available" :
                    gCLI.cfg.tableEntry[index].helpString);
    }

    /* Is the mmWave Extension enabled? */
    if (gCLI.cfg.enableMMWaveExtension == 1U)
    {
        /* YES: Pass the control to the extension help handler. */
        CLI_MMWaveExtensionHelp ();
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Execution Task
 *
 *  \ingroup CLI_UTIL_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not Applicable.
 */
static void CLI_task(UArg arg0, UArg arg1)
{
    uint8_t                 cmdString[256];
    char*                   tokenizedArgs[CLI_MAX_ARGS];
    char*                   ptrCLICommand;
    char                    delimitter[] = " \r\n";
    uint32_t                argIndex;
    CLI_CmdTableEntry*      ptrCLICommandEntry;
    int32_t                 cliStatus;
    uint32_t                index;

    /* Do we have a banner to be displayed? */
    if (gCLI.cfg.cliBanner != NULL)
    {
        /* YES: Display the banner */
        CLI_write (gCLI.cfg.cliBanner);
    }

    Task_sleep(4000); /* need to wait for BIOS_start() */

    /* Loop around forever: */
    while (1)
    {
        /* Demo Prompt: */
        CLI_write (gCLI.cfg.cliPrompt);

        /* Reset the command string: */
        memset ((void *)&cmdString[0], 0, sizeof(cmdString));

        /* Read the command message from the UART: */
        if(nbManualCmd > cntManualCmd)
        {
            strcpy((char*)&cmdString[0], manual_cmd[cntManualCmd]);
            CLI_write (manual_cmd[cntManualCmd]);
            cntManualCmd++;
        }
        else
            UART_read (gCLI.cfg.cliUartHandle, &cmdString[0], (sizeof(cmdString) - 1));

        /* Reset all the tokenized arguments: */
        memset ((void *)&tokenizedArgs, 0, sizeof(tokenizedArgs));
        argIndex      = 0;
        ptrCLICommand = (char*)&cmdString[0];

        /* comment lines found - ignore the whole line*/
        if (cmdString[0]=='%') {
            CLI_write ("Skipped\n");
            continue;
        }

        /* Set the CLI status: */
        cliStatus = -1;

        /* The command has been entered we now tokenize the command message */
        while (1)
        {
            /* Tokenize the arguments: */
            tokenizedArgs[argIndex] = strtok(ptrCLICommand, delimitter);
            if (tokenizedArgs[argIndex] == NULL)
                break;

            /* Increment the argument index: */
            argIndex++;
            if (argIndex >= CLI_MAX_ARGS)
                break;

            /* Reset the command string */
            ptrCLICommand = NULL;
        }

        /* Were we able to tokenize the CLI command? */
        if (argIndex == 0)
            continue;

        /* Cycle through all the registered CLI commands: */
        for (index = 0; index < gCLI.numCLICommands; index++)
        {
            ptrCLICommandEntry = &gCLI.cfg.tableEntry[index];

            /* Do we have a match? */
            if (strcmp(ptrCLICommandEntry->cmd, tokenizedArgs[0]) == 0)
            {
                /* YES: Pass this to the CLI registered function */
                cliStatus = ptrCLICommandEntry->cmdHandlerFxn (argIndex, tokenizedArgs);
//                if(nbManualCmd <= cntManualCmd)
                {
                    if (cliStatus == 0)
                    {
                        CLI_write ("Done\n");
                    }
                    else
                    {
                        CLI_write ("Error %d\n", cliStatus);
                    }
                }
                break;
            }
        }

        /* Did we get a matching CLI command? */
        if (index == gCLI.numCLICommands)
        {
            /* NO matching command found. Is the mmWave extension enabled? */
            if (gCLI.cfg.enableMMWaveExtension == 1U)
            {
                /* Yes: Pass this to the mmWave extension handler */
                cliStatus = CLI_MMWaveExtensionHandler (argIndex, tokenizedArgs);
            }

            /* Was the CLI command found? */
            if (cliStatus == -1)
            {
                /* No: The command was still not found */
                CLI_write ("'%s' is not recognized as a CLI command\n", tokenizedArgs[0]);
            }
        }
    }
}

/**
 *  @b Description
 *  @n
 *      Logging function which can log the messages to the CLI console
 *
 *  @param[in]  format
 *      Format string
 *
 *  \ingroup CLI_UTIL_EXTERNAL_FUNCTION
 *
 *  @retval
 *      Not Applicable.
 */
void CLI_write (const char* format, ...)
{
    va_list     arg;
    char        logMessage[256];
    int32_t     sizeMessage;

    /* Format the message: */
    va_start (arg, format);
    sizeMessage = vsnprintf (&logMessage[0], sizeof(logMessage), format, arg);
    va_end (arg);

    /* Log the message on the UART CLI console: */
    if (gCLI.cfg.usePolledMode == true)
    {
        /* Polled mode: */
        UART_writePolling (gCLI.cfg.cliUartHandle, (uint8_t*)&logMessage[0], sizeMessage);
    }
    else
    {
        /* Blocking Mode: */
        UART_write (gCLI.cfg.cliUartHandle, (uint8_t*)&logMessage[0], sizeMessage);
    }
}

/**
 *  @b Description
 *  @n
 *      This is the function which is used to initialize and setup the CLI
 *
 *  @param[in]  ptrCLICfg
 *      Pointer to the CLI configuration
 *
 *  \ingroup CLI_UTIL_EXTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t CLI_open (CLI_Cfg* ptrCLICfg)
{
    Task_Params     taskParams;
    uint32_t        index;

    /* Sanity Check: Validate the arguments */
    if (ptrCLICfg == NULL)
        return -1;

    /* Initialize the CLI MCB: */
    memset ((void*)&gCLI, 0, sizeof(CLI_MCB));

    /* Copy over the configuration: */
    memcpy ((void *)&gCLI.cfg, (void *)ptrCLICfg, sizeof(CLI_Cfg));

    /* Cycle through and determine the number of supported CLI commands: */
    for (index = 0; index < CLI_MAX_CMD; index++)
    {
        /* Do we have a valid entry? */
        if (gCLI.cfg.tableEntry[index].cmd == NULL)
        {
            /* NO: This is the last entry */
            break;
        }
        else
        {
            /* YES: Increment the number of CLI commands */
            gCLI.numCLICommands = gCLI.numCLICommands + 1;
        }
    }

    /* Is the mmWave Extension enabled? */
    if (gCLI.cfg.enableMMWaveExtension == 1U)
    {
        /* YES: Initialize the CLI Extension: */
        if (CLI_MMWaveExtensionInit (ptrCLICfg) < 0)
            return -1;
    }

    /* Do we have a CLI Prompt specified?  */
    if (gCLI.cfg.cliPrompt == NULL)
        gCLI.cfg.cliPrompt = "CLI:/>";

    /* The CLI provides a help command by default:
     * - Since we are adding this at the end of the table; a user of this module can also
     *   override this to provide its own implementation. */
    gCLI.cfg.tableEntry[gCLI.numCLICommands].cmd           = "help";
    gCLI.cfg.tableEntry[gCLI.numCLICommands].helpString    = NULL;
    gCLI.cfg.tableEntry[gCLI.numCLICommands].cmdHandlerFxn = CLI_help;

    /* Increment the number of CLI commands: */
    gCLI.numCLICommands++;

    /* Initialize the task parameters and launch the CLI Task: */
    Task_Params_init(&taskParams);
    taskParams.priority  = gCLI.cfg.taskPriority;
    taskParams.stackSize = 5*1024;
    gCLI.cliTaskHandle = Task_create(CLI_task, &taskParams, NULL);
    return 0;
}

/**
 *  @b Description
 *  @n
 *      This is the function which is used to close the CLI module
 *
 *  \ingroup CLI_UTIL_EXTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t CLI_close (void)
{
    /* Shutdown the CLI Task */
    Task_delete(&gCLI.cliTaskHandle);

    /* Cleanup the memory */
    memset ((void*)&gCLI, 0, sizeof(CLI_MCB));
    return 0;
}
