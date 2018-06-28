#include <stm32f4xx.h>
#include <arm_math.h>

#include <stm32f4_discovery.h>
#include <stm32f4_discovery_accelerometer.h>
#include <wolfson_pi_audio.h>
#include <diag/Trace.h>
#include <tests.h>
#include <dwt.h>
#include "filter.h"
//#include "stm32f4xx_nucleo.h"
#include "system_stm32f4xx.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_rcc.h"
#include "stm32f4xx_hal_rcc.h"
//#include "math_helper.h"

#define NO_INPUT 0
#define MONO_BYPASS 1
#define MONO_IN_MIXED_STEREO 2
#define MONO_IN_LEFT 3
#define MONO_IN_RIGHT 4

#define STEREO_OUTPUT 0
#define MONO_OUTPUT_DEBUG_FILE 1
#define MONO_OUTPUT 2

void setInput(int mode);
void setOutput(int mode);
void processBlock(void);
void readInterface(void);

#define NUM_TAPS   21
#define BLOCK_SIZE (WOLFSON_PI_AUDIO_TXRX_BUFFER_SIZE)/4

int16_t TxBuffer[WOLFSON_PI_AUDIO_TXRX_BUFFER_SIZE];
int16_t RxBuffer[WOLFSON_PI_AUDIO_TXRX_BUFFER_SIZE];

__IO BUFFER_StateTypeDef buffer_offset = BUFFER_OFFSET_NONE;
__IO uint8_t Volume = 100;

GPIO_InitTypeDef GPIO_InitStructure;

const float32_t doisPI = 6.283185307;

uint32_t i, j, k, k_L, k_R;

//------------------------------------------------------------------------------------
//SampleRate def:
float32_t sampleRate;
float32_t samplePeriod;

//------------------------------------------------------------------------------------
//Buffer:
float32_t inputF32Buffer_R[BLOCK_SIZE];
float32_t inputF32Buffer_L[BLOCK_SIZE];
float32_t outputF32Buffer_R[BLOCK_SIZE];
float32_t outputF32Buffer_L[BLOCK_SIZE];

FILE *inputF32Buffer_R_File, *outputF32Buffer_R_File;
FILE *inputF32Buffer_L_File, *outputF32Buffer_L_File;

float32_t processBuffer_ping_R[BLOCK_SIZE];
float32_t processBuffer_pong_R[BLOCK_SIZE];
float32_t processBuffer_ping_L[BLOCK_SIZE];
float32_t processBuffer_pong_L[BLOCK_SIZE];

float32_t inputF32Buffer_MONO[BLOCK_SIZE];
float32_t outputF32Buffer_MONO[BLOCK_SIZE];
float32_t processBuffer_ping_MONO[BLOCK_SIZE];
float32_t processBuffer_pong_MONO[BLOCK_SIZE];

//------------------------------------------------------------------------------------
FILE *inputF32Buffer_MONO_File, *outputF32Buffer_MONO_File;

//------------------------------------------------------------------------------------
//Sawtooth:
float32_t sawtooth_ths=0, sawtooth_freq=0, sawtooth_step=0;
float32_t  sawtooth_sig=0;

//------------------------------------------------------------------------------------
//sin:
float32_t sin_1_ths=0, sin_1_freq=0, sin_1_step=0, sin_1_n=0;
float32_t sin_1_sig[BLOCK_SIZE];

//------------------------------------------------------------------------------------
//onda quadrada:
float32_t pulse_1_ths=0, pulse_1_freq=0, pulse_1_datycyclo=0, pulse_1_sig =0;

//------------------------------------------------------------------------------------
//Acelerometro:
uint32_t AcceleroTicks;
int16_t AcceleroAxis[3];

#define NUM_TAPS   5
static q15_t firStateQ15[5 + NUM_TAPS - 1];
q15_t firCoeffsQ15_mediaMovel[NUM_TAPS] = {6553, 6553, 6553, 6553, 6553, 0};
q15_t inputQ15Buffer;
q15_t outputQ15Buffer;
arm_fir_instance_q15 S15;

//====================================================================================
//------------------------------------------------------------------------------------
int main(int argc, char* argv[])
{
	UNUSED(argc);
	UNUSED(argv);

	arm_fir_init_q15(&S15, NUM_TAPS, firCoeffsQ15_mediaMovel, &firStateQ15[0], 1);

	sampleRate = (float32_t)AUDIO_FREQUENCY_8K;
	samplePeriod = 1/sampleRate;

	outputF32Buffer_MONO_File = fopen("outputF32Buffer_MONO_File.txt", "w");
	fprintf(outputF32Buffer_MONO_File, "%f\n", 0.0);
	fclose(outputF32Buffer_MONO_File);

#ifdef OS_USE_SEMIHOSTING
	//Semihosting example
#endif

	// Initialise the HAL Library; it must be the first
	// instruction to be executed in the main program.
	HAL_Init();

	DWT_Enable();

#ifdef OS_USE_SEMIHOSTING
	//Semihosting example
#endif

	WOLFSON_PI_AUDIO_Init((INPUT_DEVICE_LINE_IN << 8) | OUTPUT_DEVICE_BOTH, 80, (int)sampleRate);

	WOLFSON_PI_AUDIO_SetInputMode(INPUT_DEVICE_LINE_IN);

	WOLFSON_PI_AUDIO_SetMute(AUDIO_MUTE_ON);

	WOLFSON_PI_AUDIO_Play(TxBuffer, RxBuffer, WOLFSON_PI_AUDIO_TXRX_BUFFER_SIZE);

	WOLFSON_PI_AUDIO_SetVolume(Volume);

	BSP_ACCELERO_Init();

	TEST_Init();

	//------------------------------------------------------------------------------------
	//GPIO:
    __GPIOD_CLK_ENABLE();
    GPIO_InitStructure.Pin   = GPIO_PIN_12;
    GPIO_InitStructure.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Pull  = GPIO_PULLUP;
    GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStructure);

    __GPIOD_CLK_ENABLE();
    GPIO_InitStructure.Pin   = GPIO_PIN_13;
    GPIO_InitStructure.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Pull  = GPIO_PULLUP;
    GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStructure);

    __GPIOD_CLK_ENABLE();
    GPIO_InitStructure.Pin   = GPIO_PIN_14;
    GPIO_InitStructure.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Pull  = GPIO_PULLUP;
    GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStructure);

    __GPIOD_CLK_ENABLE();
    GPIO_InitStructure.Pin   = GPIO_PIN_15;
    GPIO_InitStructure.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Pull  = GPIO_PULLUP;
    GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStructure);

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);

	//------------------------------------------------------------------------------------
	//LEDs
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);

	while (1) {

		if(buffer_offset == BUFFER_OFFSET_HALF)
		{
			//-----------------------------------------------------------------------------------------------------------------
			setInput(NO_INPUT);
			readInterface();
			processBlock();
			setOutput(MONO_OUTPUT);

			//-----------------------------------------------------------------------------------------------------------------
			//*****************************************************************************************************************
			for(i=0, k_L=0, k_R=0; i<(WOLFSON_PI_AUDIO_TXRX_BUFFER_SIZE/2); i++)
			{
				if(i%2)
				{
					inputF32Buffer_L[k_L] = (float32_t)(RxBuffer[i]/(2*32768.0));//convert to float LEFT
					k_L++;
				}
				else
				{
					//TxBuffer[i] = RxBuffer[i];//   RIGHT (canal de baixo no OcenAudio)
					inputF32Buffer_R[k_R] = (float32_t)(RxBuffer[i]/(2*32768.0));//convert to float RIGHT
					k_R++;
				}
			}

			for(i=0, k_L=0, k_R=0; i<(WOLFSON_PI_AUDIO_TXRX_BUFFER_SIZE/2); i++)
			{
				if(i%2)
				{
					TxBuffer[i] = (int16_t)(outputF32Buffer_L[k_L]*32768);//back to 1.15
					k_L++;
				}
				else
				{
					TxBuffer[i] = (int16_t)(outputF32Buffer_R[k_R]*32768);//back to 1.15
					k_R++;
				}
			}
			buffer_offset = BUFFER_OFFSET_NONE;
			//*****************************************************************************************************************
		}

		//=====================================================================================================================
		if(buffer_offset == BUFFER_OFFSET_FULL)
		{
			//-----------------------------------------------------------------------------------------------------------------
			setInput(NO_INPUT);
			processBlock();
			setOutput(MONO_OUTPUT);

			//*****************************************************************************************************************
			for(i=(WOLFSON_PI_AUDIO_TXRX_BUFFER_SIZE/2), k_L=0, k_R=0; i<WOLFSON_PI_AUDIO_TXRX_BUFFER_SIZE; i++)
			{
				if(i%2)
				{
					inputF32Buffer_L[k_L] = (float32_t)(RxBuffer[i]/(2*32768.0));//convert to float
					k_L++;
				}
				else
				{
					inputF32Buffer_R[k_R] = (float32_t)(RxBuffer[i]/(2*32768.0));//convert to float RIGHT
					k_R++;
				}
			}

			for(i=(WOLFSON_PI_AUDIO_TXRX_BUFFER_SIZE/2), k_L=0, k_R=0; i<WOLFSON_PI_AUDIO_TXRX_BUFFER_SIZE; i++)
			{
				if(i%2)
				{
					TxBuffer[i] = (int16_t)(outputF32Buffer_L[k_L]*32768);//back to 1.15
					k_L++;
				}
				else
				{
					TxBuffer[i] = (int16_t)(outputF32Buffer_R[k_R]*32768);//back to 1.15
					k_R++;
				}
			}
			buffer_offset = BUFFER_OFFSET_NONE;
			//*****************************************************************************************************************
			//-----------------------------------------------------------------------------------------------------------------
		}
		TEST_Main();
	}
	return 0;
}

/*--------------------------------
Callbacks implementation:
--------------------------------------------------------*/

/**
  * @brief  Manages the DMA full Transfer complete event.
  */
void WOLFSON_PI_AUDIO_TransferComplete_CallBack(void)
{
	buffer_offset = BUFFER_OFFSET_FULL;
}

/**
  * @brief  Manages the DMA Half Transfer complete event.
  */
void WOLFSON_PI_AUDIO_HalfTransfer_CallBack(void)
{
	  buffer_offset = BUFFER_OFFSET_HALF;
}

/**
  * @brief  Manages the DMA FIFO error interrupt.
  * @param  None
  * @retval None
  */
void WOLFSON_PI_AUDIO_OUT_Error_CallBack(void)
{
  /* Stop the program with an infinite loop */
  while (1);
}

//#define NO_INPUT 0
//#define MONO_BYPASS 1
//#define MONO_IN_MIXED_STEREO 2
//#define MONO_IN_LEFT 3
//#define MONO_IN_RIGHT 4

void setInput(int mode)
{
	if(mode){
		switch(mode)
		{
			case MONO_BYPASS:
				//-----------------------------------------------------------------------------------------------------------------
				//MONO BYPASS - MIXER STEREO
				arm_add_f32(inputF32Buffer_R, inputF32Buffer_L, outputF32Buffer_MONO, BLOCK_SIZE);
				break;

			case MONO_IN_MIXED_STEREO:
				//-----------------------------------------------------------------------------------------------------------------
				//MONO IN - MIXER STEREO
				arm_add_f32(inputF32Buffer_R, inputF32Buffer_L, inputF32Buffer_MONO, BLOCK_SIZE);
				break;

			case MONO_IN_LEFT:
				//-----------------------------------------------------------------------------------------------------------------
				//MONO IN - LEFT
				for(i=0; i<BLOCK_SIZE; i++)
				{
					inputF32Buffer_MONO[i] = inputF32Buffer_L[i];//LEFT
				}
				break;

			case MONO_IN_RIGHT:
				//-----------------------------------------------------------------------------------------------------------------
				//MONO IN - RIGHT
				for(i=0; i<BLOCK_SIZE; i++)
				{
					inputF32Buffer_MONO[i] = inputF32Buffer_R[i];//RIGHT
				}
				break;
		}
	}
}

//#define MONO_OUTPUT_DEBUG_FILE 1
//#define MONO_OUTPUT 2
//#define STEREO_OUTPUT 0

void setOutput(int mode)
{
	if(mode){
		switch(mode)
		{
			case MONO_BYPASS:
				//MONO OUT DEBUG FILE - MIXER STEREO
				outputF32Buffer_MONO_File = fopen("outputF32Buffer_MONO_File.txt", "a");
				for(i=0; i<BLOCK_SIZE; i++)
				{
					fprintf(outputF32Buffer_MONO_File, "%f\n", outputF32Buffer_MONO[i]);
				}
				fclose(outputF32Buffer_MONO_File);

				trace_printf("%.2f ... %.2f\n", sin_1_n, sin_1_n/(float32_t)BLOCK_SIZE);
				break;

			case MONO_OUTPUT:
				//-----------------------------------------------------------------------------------------------------------------
				////MONO OUT
				for(i=0; i<BLOCK_SIZE; i++)
				{
					outputF32Buffer_L[i] = outputF32Buffer_MONO[i];//LEFT
					outputF32Buffer_R[i] = outputF32Buffer_L[i];//RIGHT
				}
				break;
		}
	}
}

void readInterface()
{
	//-----------------------------------------------------------------------------------------------------------------
	BSP_ACCELERO_GetXYZ(AcceleroAxis);
	//trace_printf("x:%d y:%d z:%d ticks:%d\n", AcceleroAxis[0], AcceleroAxis[1], AcceleroAxis[2], AcceleroTicks);

	if(AcceleroAxis[0]<0)
		AcceleroAxis[0] = -AcceleroAxis[0];

	if(AcceleroAxis[1]<0)
		AcceleroAxis[1] = -AcceleroAxis[1];

	sin_1_step=(float32_t)AcceleroAxis[0];

	inputQ15Buffer = AcceleroAxis[1];
	arm_fir_fast_q15(&S15, &inputQ15Buffer, &outputQ15Buffer, BLOCK_SIZE);
	sawtooth_freq=(float32_t)outputQ15Buffer;
}

void processBlock()
{
	//-----------------------------------------------------------------------------------------------------------------
	//Geração de sinal de TOM - SAWTOOTH:
	//types: float32_t sin_1_ths=0, sin_1_freq=0, sin_1_step=0, sin_1_sig=0, sin_1_n;
	for(i=0; i<BLOCK_SIZE; i++)
	{
		sawtooth_ths = 0.01;

		//sawtooth_freq = 55;

		sawtooth_step = 2*sawtooth_ths*samplePeriod*sawtooth_freq;

		sawtooth_sig = sawtooth_sig + sawtooth_step;

		if(sawtooth_sig > sawtooth_ths)
		{
			sawtooth_sig = -sawtooth_ths;
			//sawtooth_sig = 0;
		}

		//-------------------------------------------------------------------------------------------------------------
		// Output:
		outputF32Buffer_MONO[i] = sawtooth_sig;
	}

	//-----------------------------------------------------------------------------------------------------------------
	//Geração de sinal LFO - Seno:
	//types: float32_t sin_1_ths=0, sin_1_freq=0, sin_1_step=0, sin_1_sig=0, sin_1_n;
	for(i=0; i<BLOCK_SIZE; i++)
	{
		sin_1_sig[i] = arm_sin_f32(6.283185307*(sin_1_n*samplePeriod));
		//sin_1_step = 1;
		sin_1_n = sin_1_n + sin_1_step;
		if(sin_1_n > sampleRate)
		{
			sin_1_n  = 0;
		}
	}

	//-----------------------------------------------------------------------------------------------------------------
	// Output:
	arm_mult_f32 (outputF32Buffer_MONO, sin_1_sig, outputF32Buffer_MONO, BLOCK_SIZE);
}
