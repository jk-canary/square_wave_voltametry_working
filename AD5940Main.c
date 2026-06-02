/*!
 *****************************************************************************
 @file:    AD5940Main.c
 @author:  $Author: nxu2 $
 @brief:   Used to control specific application and process data.
 @version: $Revision: 766 $
 @date:    $Date: 2017-08-21 14:09:35 +0100 (Mon, 21 Aug 2017) $
 -----------------------------------------------------------------------------

Copyright (c) 2017-2019 Analog Devices, Inc. All Rights Reserved.

This software is proprietary to Analog Devices, Inc. and its licensors.
By using this software you agree to the terms of the associated
Analog Devices Software License Agreement.

*****************************************************************************/
#include "SqrWaveVoltammetry.h"
#include "ADuCM3029.h"
#include "FIT_CURVE.h"
/**
   User could configure following parameters
**/
#define DATA_SIZE_  321
#define bDATA_SIZE_  161
#define MAX_BUFFER_SIZE 32
#define APPBUFF_SIZE 1024
uint32_t AppBuff[APPBUFF_SIZE];
float LFOSCFreq;    /* Measured LFOSC frequency */
static uint32_t cycle_count = 1;
float voltage[bDATA_SIZE_];
float CurBuff[bDATA_SIZE_];
float y1_golay[bDATA_SIZE_];
float y2_Average[bDATA_SIZE_];
float y3_fit_line[bDATA_SIZE_];
float y4_sub_val[bDATA_SIZE_];
static float peak_val = 0.0f;
char uart_rx_buffer[MAX_BUFFER_SIZE];
char *rx_buff_ptr = uart_rx_buffer;
int linears = 0;
uint8_t buffer_index = 0;

/**
 * @brief An example to deal with data read back from AD5940. Here we just print data to UART
 * @note UART has limited speed, it has impact when sample rate is fast. Try to print some of the data not all of them.
 * @param pData: the buffer stored data for this application. The data from FIFO has been pre-processed.
 * @param DataCount: The available data count in buffer pData.
 * @return return 0.
*/
static int32_t RampShowResult(float *pData, uint32_t DataCount)
{
  //static uint32_t index;
	float count_ = 0;
	if( DataCount)
	{
		
		
		for(int i=0;i<DataCount;i++)
		{
		 if(i< DataCount/2 && !(i%2 == 0))
			{
				voltage[i] = count_;
				voltage[i+1] = count_;
				count_ += 10;
			}else if(i> DataCount/2 && !(i%2 == 0))
			{

				voltage[i] = count_;
				voltage[i+1] = count_;
				count_ -= 10;
			}else
			voltage[i] = count_;
			
			if(!(i%2==0))
			{
				voltage[i] -= 50;
			}else if(i%2==0)
			{
				voltage[i] += 50;
			}
		}
		

		
		printf("\n\nraw data\n\n");
		
			// Print data
			for(int i=0;i<DataCount;i++)
			{
				printf("%.3f, %f\n", voltage[i], pData[i]);

			}
			
			//printf("\n\npeak voltage\n\n");
			
			/*for(int i=0;i<DataCount;i++)
			{
				if(i%2==0)
					{
						printf("%f :%f\n", voltage[i], pData[i]);
					}
			}*/
			//printf("\n\n datacoutn : %u \n\n", DataCount);
			//printf("\n\nbase voltage\n\n");
			linears = 0;
			for(int i=1;i<DataCount;i++)
			{


				if(i%2 == 0 && i < DataCount)
					{
						CurBuff[linears] = pData[i] - pData[i+1];  
						linears++;
					}			
			}
	
		printf("\n\ncurrent difference\n\n");
			count_ = 0;
			//printf("\n THE CURRENT DIFFERENCE ");
		for(int i=0;i<DataCount/2;i++)
			{
			if(i< DataCount)
			{
					voltage[i] = count_;
					count_ += 10;
					
			}
			printf("%.3f :%f\n", voltage[i], CurBuff[i]);
		}
		//printf("\n\n  THE MAX CURRENT  \n\n");
		golay(10,DataCount/2,CurBuff,y1_golay);
		moving_window(5,1,DataCount/2,y1_golay,y2_Average);
		fit_cubic_fun((int)(DataCount/2),voltage,y2_Average,y3_fit_line,y4_sub_val);
		for(int i=0;i<DataCount/2;i++)
			{
					printf("\n%.1f:%f:%f:%f:%f:%f", voltage[i], CurBuff[i], y1_golay[i], y2_Average[i], y3_fit_line[i], y4_sub_val[i]);
			}
		for(int i = 20; i<(int)(DataCount/2); i++)
			{		
				if(i==20)
				{
					peak_val = y4_sub_val[i];
				}
				if(peak_val<=y4_sub_val[i])
				{
					peak_val = y4_sub_val[i];
				}
			}
		printf("\nthe peak value is :%f\n\n",peak_val);
	}
	
	
		if(DataCount)
	{
		printf("\nend of %d cycle\n", cycle_count);
		cycle_count++;
	}
  return 0;
}


void UART_Int_Handler(void)
{
   if (buffer_index == MAX_BUFFER_SIZE) {																		 
        buffer_index = 0;
    }
		uart_rx_buffer[buffer_index++] = (pADI_UART0->COMRX&BITM_UART_COMRX_RBR);  // Read the Rx Register and store in uart_rx_buffer
		uart_rx_buffer[strcspn(uart_rx_buffer, "\n")] = '\0';

}

/**
 * @brief The general configuration to AD5940 like FIFO/Sequencer/Clock. 
 * @note This function will firstly reset AD5940 using reset pin.
 * @return return 0.
*/
static int32_t AD5940PlatformCfg(void)
{
  CLKCfg_Type clk_cfg;
  SEQCfg_Type seq_cfg;  
  FIFOCfg_Type fifo_cfg;
  AGPIOCfg_Type gpio_cfg;
  LFOSCMeasure_Type LfoscMeasure;

  /* Use hardware reset */
  AD5940_HWReset();
  AD5940_Initialize();    /* Call this right after AFE reset */

  /* Platform configuration */
  /* Step1. Configure clock */
  clk_cfg.HFOSCEn = bTRUE;
  clk_cfg.HFXTALEn = bFALSE;
  clk_cfg.LFOSCEn = bTRUE;
  clk_cfg.HfOSC32MHzMode = bFALSE;
  clk_cfg.SysClkSrc = SYSCLKSRC_HFOSC;
  clk_cfg.SysClkDiv = SYSCLKDIV_1;
  clk_cfg.ADCCLkSrc = ADCCLKSRC_HFOSC;
  clk_cfg.ADCClkDiv = ADCCLKDIV_1;
  AD5940_CLKCfg(&clk_cfg);
  /* Step2. Configure FIFO and Sequencer*/
  fifo_cfg.FIFOEn = bTRUE;           /* We will enable FIFO after all parameters configured */
  fifo_cfg.FIFOMode = FIFOMODE_FIFO;
  fifo_cfg.FIFOSize = FIFOSIZE_4KB;   /* 2kB for FIFO, The reset 4kB for sequencer */
  fifo_cfg.FIFOSrc = FIFOSRC_SINC2NOTCH;   /* */
  fifo_cfg.FIFOThresh = 4;            /*  Don't care, set it by application paramter */
  AD5940_FIFOCfg(&fifo_cfg);
  seq_cfg.SeqMemSize = SEQMEMSIZE_2KB;  /* 4kB SRAM is used for sequencer, others for data FIFO */
  seq_cfg.SeqBreakEn = bFALSE;
  seq_cfg.SeqIgnoreEn = bTRUE;
  seq_cfg.SeqCntCRCClr = bTRUE;
  seq_cfg.SeqEnable = bFALSE;
  seq_cfg.SeqWrTimer = 0;
  AD5940_SEQCfg(&seq_cfg);
  /* Step3. Interrupt controller */
  AD5940_INTCCfg(AFEINTC_1, AFEINTSRC_ALLINT, bTRUE);   /* Enable all interrupt in INTC1, so we can check INTC flags */
  AD5940_INTCClrFlag(AFEINTSRC_ALLINT);
  AD5940_INTCCfg(AFEINTC_0, AFEINTSRC_DATAFIFOTHRESH|AFEINTSRC_ENDSEQ|AFEINTSRC_CUSTOMINT0, bTRUE); 
  AD5940_INTCClrFlag(AFEINTSRC_ALLINT);
  /* Step4: Configure GPIOs */
  gpio_cfg.FuncSet = GP0_INT|GP1_SLEEP|GP2_SYNC;  /* GPIO1 indicates AFE is in sleep state. GPIO2 indicates ADC is sampling. */
  gpio_cfg.InputEnSet = 0;
  gpio_cfg.OutputEnSet = AGPIO_Pin0|AGPIO_Pin1|AGPIO_Pin2;
  gpio_cfg.OutVal = 0;
  gpio_cfg.PullEnSet = 0;
  AD5940_AGPIOCfg(&gpio_cfg);
  /* Measure LFOSC frequency */
  /**@note Calibrate LFOSC using system clock. The system clock accuracy decides measurement accuracy. Use XTAL to get better result. */
  LfoscMeasure.CalDuration = 1000.0;  /* 1000ms used for calibration. */
  LfoscMeasure.CalSeqAddr = 0;        /* Put sequence commands from start address of SRAM */
  LfoscMeasure.SystemClkFreq = 16000000.0f; /* 16MHz in this firmware. */
  AD5940_LFOSCMeasure(&LfoscMeasure, &LFOSCFreq);
  printf("LFOSC Freq:%f\n", LFOSCFreq);
 // AD5940_SleepKeyCtrlS(SLPKEY_UNLOCK);         /*  */
  return 0;
}

/**
 * @brief The interface for user to change application paramters.
 * @return return 0.
*/
void AD5940RampStructInit(void)
{
	
  AppSWVCfg_Type *pRampCfg;
  
  AppSWVGetCfg(&pRampCfg);
  /* Step1: configure general parmaters */
  pRampCfg->SeqStartAddr = 0x10;                /* leave 16 commands for LFOSC calibration.  */
  pRampCfg->MaxSeqLen = 512-0x10;              /* 4kB/4 = 1024  */
  pRampCfg->RcalVal = 200.0;                  /* 10kOhm RCAL */
  pRampCfg->ADCRefVolt = 1.820f;               /* The real ADC reference voltage. Measure it from capacitor C12 with DMM. */
  pRampCfg->FifoThresh = 320;                   /* Maximum value is 2kB/4-1 = 512-1. Set it to higher value to save power. */
  pRampCfg->SysClkFreq = 16000000.0f;           /* System clock is 16MHz by default */
  pRampCfg->LFOSCClkFreq = LFOSCFreq;           /* LFOSC frequency */
	pRampCfg->AdcPgaGain = ADCPGA_1P5;
	pRampCfg->ADCSinc3Osr = ADCSINC3OSR_4;
  
	/* Step 2:Configure square wave signal parameters */
  pRampCfg->RampStartVolt = 0.0f;     /* Measurement starts at 0V*/
  pRampCfg->RampPeakVolt = 800.0f;     		 /* Measurement finishes at -0.4V */
  pRampCfg->VzeroStart = 1300.0f;           /* Vzero is voltage on SE0 pin: 1.3V */
  pRampCfg->VzeroPeak = 1300.0f;          /* Vzero is voltage on SE0 pin: 1.3V */
  pRampCfg->Frequency = 50;                 /* Frequency of square wave in Hz */
  pRampCfg->SqrWvAmplitude = 50;       /* Amplitude of square wave in mV */
  pRampCfg->SqrWvRampIncrement = 10; /* Increment in mV*/
  pRampCfg->SampleDelay = 9.0f;             /* Time between update DAC and ADC sample. Unit is ms and must be < (1/Frequency)/2 - 0.2*/
  pRampCfg->LPTIARtiaSel = LPTIARTIA_1K;      /* Maximum current decides RTIA value */
	pRampCfg->bRampOneDir = bTRUE;			/* Only measure ramp in one direction */
	pRampCfg->currINc =  pRampCfg->RampStartVolt/DAC12BITVOLT_1LSB;
	
}
void admin(void)
{  
	uint32_t temp;  
	AppSWVCfg_Type *pRampCfg;
  AD5940PlatformCfg();
  AD5940RampStructInit();
	//AppSWVGetCfg(&pRampCfg);
	
	//AD5940_McuSetLow();
  AppSWVInit(AppBuff, APPBUFF_SIZE);    /* Initialize RAMP application. Provide a buffer, which is used to store sequencer commands */
	AD5940_Delay10us(100000);		/* Add a delay to allow sensor reach equilibrium befor starting the measurement */
  AppSWVCtrl(APPCTRL_START, 0);  
	while(1)
	{
		AppSWVGetCfg(&pRampCfg);
		if(AD5940_GetMCUIntFlag())
    {
      AD5940_ClrMCUIntFlag();
      temp = APPBUFF_SIZE;
      AppSWVISR(AppBuff, &temp);
      RampShowResult((float*)AppBuff, temp);
    }		/* Repeat Measurement continuously*/
		if(pRampCfg->bTestFinished == bTRUE)
		{
			AD5940_Delay10us(200000);
			AD5940_SEQCtrlS(bTRUE);   /* Enable sequencer, and wait for trigger */
			AppSWVCtrl(APPCTRL_START, 0); 
			pRampCfg->bTestFinished = bFALSE;			
		}else if(cycle_count == 11)
		{
			cycle_count=1;
			AppSWVCtrl(APPCTRL_SHUTDOWN, 0);
			break;			
		}		
	}
}
void AD5940_Main(void)
{
        /* Control IMP measurement to start. Second parameter has no meaning with this command. */

  while(1)
  {
		if (strstr((const char *)uart_rx_buffer, "START"))
    {
			admin();
			memset(uart_rx_buffer, '\0', sizeof(uart_rx_buffer));
			buffer_index = 0;
		}
		

  }
}

