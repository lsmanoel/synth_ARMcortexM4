#include <stm32f4xx.h>
#include <arm_math.h>

#include <stm32f4_discovery.h>
#include <stm32f4_discovery_accelerometer.h>
#include <wolfson_pi_audio.h>
#include <diag/Trace.h>
//#include <tests.h>
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
//Acelerometro:
uint32_t AcceleroTicks;
int16_t AcceleroAxis[3];

#define NUM_TAPS   5
#define BUFFER_SIZE_ACEL_X 2
#define BUFFER_SIZE_ACEL_Y 2
#define BUFFER_SIZE_ACEL_Z 2

q15_t inputQ15Buffer_acel_x[BUFFER_SIZE_ACEL_X];uint32_t inputQ15Buffer_acel_x_n;
q15_t outputQ15Buffer_acel_x;
q15_t inputQ15Buffer_acel_y[BUFFER_SIZE_ACEL_Y];uint32_t inputQ15Buffer_acel_y_n;
q15_t outputQ15Buffer_acel_y;
q15_t inputQ15Buffer_acel_z[BUFFER_SIZE_ACEL_Z];uint32_t inputQ15Buffer_acel_z_n;
q15_t outputQ15Buffer_acel_z;

float32_t fixNote(float32_t chromeNote);

void readInterface()
{
	BSP_ACCELERO_GetXYZ(AcceleroAxis);

	//-----------------------------------------------------------------------------------------------------------------
	//trace_printf("x:%d y:%d z:%d ticks:%d\n", AcceleroAxis[0], AcceleroAxis[1], AcceleroAxis[2], AcceleroTicks);

	//-----------------------------------------------------------------------------------------------------------------
	if(inputQ15Buffer_acel_x_n<BUFFER_SIZE_ACEL_X)
	{
		inputQ15Buffer_acel_x[inputQ15Buffer_acel_x_n] = AcceleroAxis[0];
		inputQ15Buffer_acel_x_n++;
	}
	else
	{
		inputQ15Buffer_acel_x_n = 0;
		arm_mean_q15(inputQ15Buffer_acel_x, BUFFER_SIZE_ACEL_X, &outputQ15Buffer_acel_x);
		outputQ15Buffer_acel_x = outputQ15Buffer_acel_x + 2048;
	}

	//-----------------------------------------------------------------------------------------------------------------
	if(inputQ15Buffer_acel_y_n<BUFFER_SIZE_ACEL_Y)
	{
		inputQ15Buffer_acel_y[inputQ15Buffer_acel_y_n] = AcceleroAxis[1];
		inputQ15Buffer_acel_y_n++;
	}
	else
	{
		inputQ15Buffer_acel_y_n = 0;
		arm_mean_q15(inputQ15Buffer_acel_y, BUFFER_SIZE_ACEL_Y, &outputQ15Buffer_acel_y);
		outputQ15Buffer_acel_y = outputQ15Buffer_acel_y + 2048;
		//outputQ15Buffer_acel_y = outputQ15Buffer_acel_y/2048;
		//outputQ15Buffer_acel_y = outputQ15Buffer_acel_y*outputQ15Buffer_acel_y;
	}

	//-----------------------------------------------------------------------------------------------------------------
	if(inputQ15Buffer_acel_z_n<BUFFER_SIZE_ACEL_Z)
	{
		inputQ15Buffer_acel_z[inputQ15Buffer_acel_z_n] = AcceleroAxis[2];
		inputQ15Buffer_acel_z_n++;
	}
	else
	{
		inputQ15Buffer_acel_z_n = 0;
		arm_mean_q15(inputQ15Buffer_acel_z, BUFFER_SIZE_ACEL_Z, &outputQ15Buffer_acel_z);
		outputQ15Buffer_acel_z = outputQ15Buffer_acel_z + 2048;
	}
	//-----------------------------------------------------------------------------------------------------------------

	//-----------------------------------------------------------------------------------------------------------------
	//trace_printf("x:%d y:%d z:%d\n", outputQ15Buffer_acel_x, outputQ15Buffer_acel_y, outputQ15Buffer_acel_z);
}

void processBlock()
{
	float32_t sin_freq;
	//-----------------------------------------------------------------------------------------------------------------
	//Geração de sinal de TOM - SAWTOOTH:
	//------------------------------------------------------------------------------------
	//float32_t sawtooth_ths=0, sawtooth_freq=0, sawtooth_step=0;
	//float32_t  sawtooth_sig=0;
	for(i=0; i<BLOCK_SIZE; i++)
	{
		sawtooth_ths = 0.01;

		sawtooth_freq = fixNote((float32_t)outputQ15Buffer_acel_y/32);
		//sawtooth_freq = (float32_t)outputQ15Buffer_acel_y;
		//sawtooth_freq = 55;

		sawtooth_step = 2*sawtooth_ths*samplePeriod*sawtooth_freq;

        if (GPIO_PIN_SET == HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)) {
        	sawtooth_step = sawtooth_step/2;
        }

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
		//sin_freq = fixTime(outputQ15Buffer_acel_x);
		sin_freq = (float32_t)outputQ15Buffer_acel_x/(2048.0);
		sin_freq = sin_freq * sin_freq;
		sin_freq = sin_freq * sin_freq;

        if (GPIO_PIN_SET == HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)) {
        	//sin_freq = sin_freq*(8.0/3.0);
        	sin_1_n  = sampleRate/(4.0*sin_freq);
        }

		sin_1_sig[i] = arm_sin_f32(6.283185307*(sin_freq*sin_1_n*samplePeriod));

		//sin_1_step = 1;



		if(sin_1_n < 0.2*sampleRate/sin_freq)
		{
		    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
		    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
		    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
		    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
			sin_1_n++;
		}
		else if(sin_1_n < 0.4*sampleRate/sin_freq)
		{
		    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
		    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
		    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
		    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
			sin_1_n++;
		}
		else if(sin_1_n < 0.6*sampleRate/sin_freq)
		{
		    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
		    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
		    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
		    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
			sin_1_n++;
		}
		else if(sin_1_n < 0.8*sampleRate/sin_freq)
		{
		    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
		    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
		    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
		    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
		    sin_1_n++;
		}
		else
		{
			sin_1_n  = 0;
		}
	}

	//-----------------------------------------------------------------------------------------------------------------
	// Output:
	arm_mult_f32(outputF32Buffer_MONO, sin_1_sig, outputF32Buffer_MONO, BLOCK_SIZE);

	//trace_printf("STooth:%f sin_freq:%f\n", sawtooth_freq, sin_freq);
}

//====================================================================================
//------------------------------------------------------------------------------------
int main(int argc, char* argv[])
 {
	UNUSED(argc);
	UNUSED(argv);

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

	//TEST_Init();

	//------------------------------------------------------------------------------------
	//GPIO:
	HAL_Init();

	//Push Button
    __GPIOA_CLK_ENABLE();
    GPIO_InitStructure.Pin   = GPIO_PIN_0;
    GPIO_InitStructure.Mode  = GPIO_MODE_INPUT;
    GPIO_InitStructure.Pull  = GPIO_NOPULL;
    //GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

	//LED:
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

//    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
//    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
//    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
//    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);

	while (1) {

		readInterface();

		if(buffer_offset == BUFFER_OFFSET_HALF)
		{
			//-----------------------------------------------------------------------------------------------------------------
			setInput(NO_INPUT);
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



#define C1_NOTE_F  C4_NOTE_F/8
#define CS1_NOTE_F CS4_NOTE_F/8
#define D1_NOTE_F  D4_NOTE_F/8
#define DS1_NOTE_F DS4_NOTE_F/8
#define E1_NOTE_F  E4_NOTE_F/8
#define F1_NOTE_F  F4_NOTE_F/8
#define FS1_NOTE_F FS4_NOTE_F/8
#define G1_NOTE_F  G4_NOTE_F/8
#define GS1_NOTE_F GS4_NOTE_F/8
#define A1_NOTE_F  A4_NOTE_F/8
#define AS1_NOTE_F AS4_NOTE_F/8
#define B1_NOTE_F  B4_NOTE_F/8

#define C2_NOTE_F  C4_NOTE_F/4
#define CS2_NOTE_F CS4_NOTE_F/4
#define D2_NOTE_F  D4_NOTE_F/4
#define DS2_NOTE_F DS4_NOTE_F/4
#define E2_NOTE_F  E4_NOTE_F/4
#define F2_NOTE_F  F4_NOTE_F/4
#define FS2_NOTE_F FS4_NOTE_F/4
#define G2_NOTE_F  G4_NOTE_F/4
#define GS2_NOTE_F GS4_NOTE_F/4
#define A2_NOTE_F  A4_NOTE_F/4
#define AS2_NOTE_F AS4_NOTE_F/4
#define B2_NOTE_F  B4_NOTE_F/4

#define C3_NOTE_F  C4_NOTE_F/2
#define CS3_NOTE_F CS4_NOTE_F/2
#define D3_NOTE_F  D4_NOTE_F/2
#define DS3_NOTE_F DS4_NOTE_F/2
#define E3_NOTE_F  E4_NOTE_F/2
#define F3_NOTE_F  F4_NOTE_F/2
#define FS3_NOTE_F FS4_NOTE_F/2
#define G3_NOTE_F  G4_NOTE_F/2
#define GS3_NOTE_F GS4_NOTE_F/2
#define A3_NOTE_F  A4_NOTE_F/2
#define AS3_NOTE_F AS4_NOTE_F/2
#define B3_NOTE_F  B4_NOTE_F/2

#define C4_NOTE_F  261.62557 // Hz
#define CS4_NOTE_F 277.18263 // Hz
#define D4_NOTE_F  293.66477 // Hz
#define DS4_NOTE_F 311.12698 // Hz
#define E4_NOTE_F  329.62756 // Hz
#define F4_NOTE_F  349.22823 // Hz
#define FS4_NOTE_F 369.99442 // Hz
#define G4_NOTE_F  391.99544 // Hz
#define GS4_NOTE_F 415.3047 // Hz
#define A4_NOTE_F  440 // Hz
#define AS4_NOTE_F 466.16376 // Hz
#define B4_NOTE_F  493.8833 // Hz

#define C5_NOTE_F  2*C4_NOTE_F
#define CS5_NOTE_F 2*CS4_NOTE_F
#define D5_NOTE_F  2*D4_NOTE_F
#define DS5_NOTE_F 2*DS4_NOTE_F
#define E5_NOTE_F  2*E4_NOTE_F
#define F5_NOTE_F  2*F4_NOTE_F
#define FS5_NOTE_F 2*FS4_NOTE_F
#define G5_NOTE_F  2*G4_NOTE_F
#define GS5_NOTE_F 2*GS4_NOTE_F
#define A5_NOTE_F  2*A4_NOTE_F
#define AS5_NOTE_F 2*AS4_NOTE_F
#define B5_NOTE_F  2*B4_NOTE_F

 float32_t fixNote(float32_t chromeNote)
 {
	 float32_t scaleNote;
	 if(chromeNote < C1_NOTE_F)
	 {
		 scaleNote = C1_NOTE_F;
	 }
	 else if(chromeNote < D1_NOTE_F)
	 {
		 scaleNote = D1_NOTE_F;
	 }
	 else if(chromeNote < DS1_NOTE_F)
	 {
		 scaleNote = DS1_NOTE_F;
	 }
	 else if(chromeNote < F1_NOTE_F)
	 {
		 scaleNote = F1_NOTE_F;
	 }
	 else if(chromeNote < G1_NOTE_F)
	 {
		 scaleNote = G1_NOTE_F;
	 }
	 else if(chromeNote < GS1_NOTE_F)
	 {
		 scaleNote = GS1_NOTE_F;
	 }
	 else if(chromeNote < AS1_NOTE_F)
	 {
		 scaleNote = AS1_NOTE_F;
	 }
	 //---------------------------------------------------------
	 else if(chromeNote < C2_NOTE_F)
	 {
		 scaleNote = C2_NOTE_F;
	 }
	 else if(chromeNote < D2_NOTE_F)
	 {
		 scaleNote = D2_NOTE_F;
	 }
	 else if(chromeNote < DS2_NOTE_F)
	 {
		 scaleNote = DS2_NOTE_F;
	 }
	 else if(chromeNote < F2_NOTE_F)
	 {
		 scaleNote = F2_NOTE_F;
	 }
	 else if(chromeNote < G2_NOTE_F)
	 {
		 scaleNote = G2_NOTE_F;
	 }
	 else if(chromeNote < GS2_NOTE_F)
	 {
		 scaleNote = GS2_NOTE_F;
	 }
	 else if(chromeNote < AS2_NOTE_F)
	 {
		 scaleNote = AS2_NOTE_F;
	 }
	 //---------------------------------------------------------
	 else if(chromeNote < C3_NOTE_F)
	 {
		 scaleNote = C3_NOTE_F;
	 }
	 else if(chromeNote < D3_NOTE_F)
	 {
		 scaleNote = D3_NOTE_F;
	 }
	 else if(chromeNote < DS3_NOTE_F)
	 {
		 scaleNote = DS3_NOTE_F;
	 }
	 else if(chromeNote < F3_NOTE_F)
	 {
		 scaleNote = F3_NOTE_F;
	 }
	 else if(chromeNote < G3_NOTE_F)
	 {
		 scaleNote = G3_NOTE_F;
	 }
	 else if(chromeNote < GS3_NOTE_F)
	 {
		 scaleNote = GS3_NOTE_F;
	 }
	 else if(chromeNote < AS3_NOTE_F)
	 {
		 scaleNote = AS3_NOTE_F;
	 }
	 //---------------------------------------------------------
	 else if(chromeNote < C4_NOTE_F)
	 {
		 scaleNote = C4_NOTE_F;
	 }
	 else if(chromeNote < D4_NOTE_F)
	 {
		 scaleNote = D4_NOTE_F;
	 }
	 else if(chromeNote < DS4_NOTE_F)
	 {
		 scaleNote = DS4_NOTE_F;
	 }
	 else if(chromeNote < F4_NOTE_F)
	 {
		 scaleNote = F4_NOTE_F;
	 }
	 else if(chromeNote < G4_NOTE_F)
	 {
		 scaleNote = G4_NOTE_F;
	 }
	 else if(chromeNote < GS4_NOTE_F)
	 {
		 scaleNote = GS4_NOTE_F;
	 }
	 else if(chromeNote < AS4_NOTE_F)
	 {
		 scaleNote = AS4_NOTE_F;
	 }
	 //---------------------------------------------------------
	 else if(chromeNote < C5_NOTE_F)
	 {
		 scaleNote = C5_NOTE_F;
	 }
	 else if(chromeNote < D5_NOTE_F)
	 {
		 scaleNote = D5_NOTE_F;
	 }
	 else if(chromeNote < DS5_NOTE_F)
	 {
		 scaleNote = DS5_NOTE_F;
	 }
	 else if(chromeNote < F5_NOTE_F)
	 {
		 scaleNote = F5_NOTE_F;
	 }
	 else if(chromeNote < G5_NOTE_F)
	 {
		 scaleNote = G5_NOTE_F;
	 }
	 else if(chromeNote < GS5_NOTE_F)
	 {
		 scaleNote = GS5_NOTE_F;
	 }
	 else if(chromeNote < AS5_NOTE_F)
	 {
		 scaleNote = AS5_NOTE_F;
	 }

	return scaleNote;
 }
