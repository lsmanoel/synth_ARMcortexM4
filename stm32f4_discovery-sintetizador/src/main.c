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

#define NUM_TAPS   21
#define BLOCK_SIZE (WOLFSON_PI_AUDIO_TXRX_BUFFER_SIZE)/4

int16_t TxBuffer[WOLFSON_PI_AUDIO_TXRX_BUFFER_SIZE];
int16_t RxBuffer[WOLFSON_PI_AUDIO_TXRX_BUFFER_SIZE];

__IO BUFFER_StateTypeDef buffer_offset = BUFFER_OFFSET_NONE;
__IO uint8_t Volume = 100;


GPIO_InitTypeDef GPIO_InitStructure;

int main(int argc, char* argv[])
{
	UNUSED(argc);
	UNUSED(argv);

	uint32_t i, j, k, k_L, k_R;

	//------------------------------------------------------------------------------------
	//SampleRate def:
	float32_t sampleRate;
	float32_t samplePeriod;
	sampleRate = (float32_t)AUDIO_FREQUENCY_48K;
	samplePeriod = 1/sampleRate;

	//------------------------------------------------------------------------------------
	//Buffer:
	float32_t inputF32Buffer_R[BLOCK_SIZE];
	float32_t inputF32Buffer_L[BLOCK_SIZE];
	float32_t outputF32Buffer_R[BLOCK_SIZE];
	float32_t outputF32Buffer_L[BLOCK_SIZE];

	float32_t processBuffer_ping_R[BLOCK_SIZE];
	float32_t processBuffer_pong_R[BLOCK_SIZE];
	float32_t processBuffer_ping_L[BLOCK_SIZE];
	float32_t processBuffer_pong_L[BLOCK_SIZE];

	float32_t inputF32Buffer_MONO[BLOCK_SIZE];
	float32_t outputF32Buffer_MONO[BLOCK_SIZE];
	float32_t processBuffer_ping_MONO[BLOCK_SIZE];
	float32_t processBuffer_pong_MONO[BLOCK_SIZE];

	//------------------------------------------------------------------------------------
	//Sawtooth:
	float32_t sawtooth_ths=0, sawtooth_freq=0, sawtooth_step=0, sawtooth_sig=0;


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

	WOLFSON_PI_AUDIO_Init((INPUT_DEVICE_LINE_IN << 8) | OUTPUT_DEVICE_BOTH, 80, AUDIO_FREQUENCY_48K);

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

		//*****************************************************************************************************************
		//-----------------------------------------------------------------------------------------------------------------
		if(buffer_offset == BUFFER_OFFSET_HALF)
		{
			for(i=0, k_L=0, k_R=0; i<(WOLFSON_PI_AUDIO_TXRX_BUFFER_SIZE/2); i++) {
				if(i%2) {
					inputF32Buffer_L[k_L] = (float32_t)(RxBuffer[i]/32768.0);//convert to float LEFT
					k_L++;
				}
				else {
					//TxBuffer[i] = RxBuffer[i];//   RIGHT (canal de baixo no OcenAudio)
					inputF32Buffer_R[k_R] = (float32_t)(RxBuffer[i]/32768.0);//convert to float RIGHT
					k_R++;
				}
			}

			for(i=0, k_L=0, k_R=0; i<(WOLFSON_PI_AUDIO_TXRX_BUFFER_SIZE/2); i++) {
				if(i%2)	{
					TxBuffer[i] = (int16_t)(outputF32Buffer_L[k_L]*32768);//back to 1.15
					k_L++;
				}
				else{
					TxBuffer[i] = (int16_t)(outputF32Buffer_R[k_R]*32768);//back to 1.15
					k_R++;
				}
			}
			buffer_offset = BUFFER_OFFSET_NONE;
		}
		//-----------------------------------------------------------------------------------------------------------------
		//MONO IN - MIXER STEREO
//		for(i=0; i<BLOCK_SIZE; i++)
//		{
//			inputF32Buffer_MONO[i] = (inputF32Buffer_R[i]+inputF32Buffer_L[i])/2;//LEFT
//		}
		//-----------------------------------------------------------------------------------------------------------------
		//*****************************************************************************************************************

		//-----------------------------------------------------------------------------------------------------------------
		//Geração de sinal - Sawtooth:
		//sawtooth_ths, sawtooth_freq, sawtooth_step, sawtooth_sig;
		for(i=0; i<BLOCK_SIZE; i++)
		{
			sawtooth_ths = 0.5;

			sawtooth_freq = 220;

			sawtooth_step = samplePeriod*sawtooth_freq;

			sawtooth_sig = sawtooth_sig + sawtooth_step;

			if(sawtooth_sig > sawtooth_ths)
			{
				//sawtooth_sig = -sawtooth_sig;
				sawtooth_sig = 0;
			}

			outputF32Buffer_MONO[i] = sawtooth_sig;
		}




		//*****************************************************************************************************************
		//-----------------------------------------------------------------------------------------------------------------
		//MONO OUT - MIXER STEREO
		for(i=0; i<BLOCK_SIZE; i++)
		{
			outputF32Buffer_L[i] = outputF32Buffer_MONO[i];//LEFT
			outputF32Buffer_R[i] = outputF32Buffer_L[i];//RIGHT
		}
		//-----------------------------------------------------------------------------------------------------------------
		//MONO BYPASS  - MIXER STEREO
//		for(i=0; i<BLOCK_SIZE; i++)
//		{
//			outputF32Buffer_L[i] = (inputF32Buffer_R[i]+inputF32Buffer_L[i])/2;//LEFT
//			outputF32Buffer_R[i] = outputF32Buffer_L[i];//RIGHT
//		}
		//-----------------------------------------------------------------------------------------------------------------
		//STEREO BYPASS - FULL
//		for(i=0; i<BLOCK_SIZE; i++)
//		{
//			outputF32Buffer_L[i] = inputF32Buffer_L[i];//LEFT
//			outputF32Buffer_R[i] = inputF32Buffer_R[i];//RIGHT
//		}
		//-----------------------------------------------------------------------------------------------------------------
		if(buffer_offset == BUFFER_OFFSET_FULL)
		{
			for(i=(WOLFSON_PI_AUDIO_TXRX_BUFFER_SIZE/2), k_L=0, k_R=0; i<WOLFSON_PI_AUDIO_TXRX_BUFFER_SIZE; i++) {
				if(i%2) {
					inputF32Buffer_L[k_L] = (float32_t)(RxBuffer[i]/32768.0);//convert to float
					k_L++;
				}
				else {
					inputF32Buffer_R[k_R] = (float32_t)(RxBuffer[i]/32768.0);//convert to float RIGHT
					k_R++;
				}
			}

			for(i=(WOLFSON_PI_AUDIO_TXRX_BUFFER_SIZE/2), k_L=0, k_R=0; i<WOLFSON_PI_AUDIO_TXRX_BUFFER_SIZE; i++) {
				if(i%2)	{
					TxBuffer[i] = (int16_t)(outputF32Buffer_L[k_L]*32768);//back to 1.15
					k_L++;
				}
				else{
					TxBuffer[i] = (int16_t)(outputF32Buffer_R[k_R]*32768);//back to 1.15
					k_R++;
				}
			}
			buffer_offset = BUFFER_OFFSET_NONE;
		}
		//-----------------------------------------------------------------------------------------------------------------
		//*****************************************************************************************************************

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
