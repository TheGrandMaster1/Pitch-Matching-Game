/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define ARM_MATH_CM4
#include "arm_math.h"
#include "stdbool.h"
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <stdio.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DAC_MAX_BUFFER_SIZE 30
#define FFT_LENGTH  1024    // Power of 2 for FFT
#define SAMPLE_RATE 44100   // Hz
#define PI 3.14159265358979f
#define RECORD_LEN	65000	// samples for 65k/44.1khz = 1.47s
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac1_ch1;

DFSDM_Filter_HandleTypeDef hdfsdm1_filter0;
DFSDM_Channel_HandleTypeDef hdfsdm1_channel2;
DMA_HandleTypeDef hdma_dfsdm1_flt0;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
// State machine for the game
enum GameState {
	IDLE,
	PLAY_TONE,
	WAIT_FOR_RECORD,
	RECORD_SOUND,
	ANALYZE_RECORDING,
	PLAYBACK_SOUND,
	SHOW_RESULT
};

enum GameState gameState = PLAY_TONE;

float32_t sine_value;
float32_t angle;
uint16_t dacBuffer[RECORD_LEN];  // DAC reads here (8-bit samples, since we read a byte from memory)
int32_t dfsdmBuffer[RECORD_LEN]; // DFSDM writes here (32-bit samples)


volatile uint32_t target_freq; //random frequency for the played audio
volatile float32_t real_freq;
volatile float32_t recorded_freq = 0.0f; // frequency detected in recorded audio
volatile int32_t raw;
// volatile bool playback = false;
// volatile bool recording = false;

// Global array for DMA:
volatile uint16_t sine_wave_array[DAC_MAX_BUFFER_SIZE];

uint8_t uart_rx_buffer[32];
volatile bool uart_cmd_ready = false;
char uart_line[32];
uint8_t uart_pos = 0;

// FFT Analysis Variables
float32_t fftInput[FFT_LENGTH];
float32_t fftOutput[FFT_LENGTH];
float32_t fftMagnitude[FFT_LENGTH/2];
arm_rfft_fast_instance_f32 fftInstance;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_DFSDM1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
// ADD FFT FUNCTION PROTOTYPE
float32_t analyze_frequency(int32_t* audio_data, uint32_t data_length);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void play_sound(void)
{
	//generate sine wave:
	for (int i = 0; i < DAC_MAX_BUFFER_SIZE; i++) {
	  angle = 2.0f * 3.1415926535f * i / DAC_MAX_BUFFER_SIZE;
	  sine_value = arm_sin_f32(angle);
	  sine_wave_array[i] = (uint16_t)(2047.5f * sine_value + 2047.5f);
	}
	// Start DAC in DMA mode for the sine wave:
	HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t*)sine_wave_array,DAC_MAX_BUFFER_SIZE, DAC_ALIGN_12B_R); //set DAC to read bytes from memory...
}

void set_random_frequency(void)
{
    // Generate random number between 400 Hz and 2100 Hz (example range)
	// rand() gives a random integer between 0 and RAND_MAX (usually 32767).
    target_freq = 400 + (rand() % 1700);

    // Assuming your timer clock = 80 MHz (adjust if different)
    uint32_t timer_clock = 120000000;

    // Compute new ARR value:
    // DAC update rate = timer_clock / (Prescaler+1) / (Period+1)
    // You need the update rate to be f_timer = N_samples * f_out
    uint32_t f_timer = DAC_MAX_BUFFER_SIZE * target_freq;
    uint32_t new_period = (timer_clock / f_timer) - 1;

    __HAL_TIM_DISABLE(&htim2);
    __HAL_TIM_SET_AUTORELOAD(&htim2, new_period);
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    __HAL_TIM_ENABLE(&htim2);

    real_freq = (float32_t)timer_clock / ((new_period + 1) * DAC_MAX_BUFFER_SIZE);
}
//FFT:
float32_t analyze_frequency(int32_t* audio_data, uint32_t data_length) {
    // 1. Prepare data for FFT (use first FFT_LENGTH samples)
    uint32_t samples_to_use = (data_length < FFT_LENGTH) ? data_length : FFT_LENGTH;

    // Convert to float and apply windowing
    for (int i = 0; i < samples_to_use; i++) {
        // Apply Hamming window to reduce spectral leakage
        float32_t window = 0.54f - 0.46f * arm_cos_f32(2.0f * PI * i / (FFT_LENGTH - 1));
        fftInput[i] = ((float32_t)audio_data[i]) * window;
    }

    // Zero-pad if we don't have enough samples
    for (int i = samples_to_use; i < FFT_LENGTH; i++) {
        fftInput[i] = 0.0f;
    }

    // 2. Perform Real FFT
    arm_rfft_fast_f32(&fftInstance, fftInput, fftOutput, 0);

    // 3. Compute magnitude spectrum
    for (int i = 0; i < FFT_LENGTH/2; i++) {
        float32_t real, imag;

        if (i == 0) {
            // DC component
            real = fftOutput[0];
            imag = 0.0f;
        } else if (i == FFT_LENGTH/2) {
            // Nyquist frequency
            real = fftOutput[1];
            imag = 0.0f;
        } else {
            real = fftOutput[2*i];
            imag = fftOutput[2*i + 1];
        }

        fftMagnitude[i] = sqrtf(real*real + imag*imag);
    }

    // 4. Find the peak frequency (ignore DC and very low frequencies)
    uint32_t maxIndex = 0;
    float32_t maxMagnitude = 0.0f;

    // Search in human voice range (100Hz to 3000Hz)
    uint32_t start_bin = (uint32_t)(100.0f * FFT_LENGTH / SAMPLE_RATE);  // ~2-3 bins
    uint32_t end_bin = (uint32_t)(3000.0f * FFT_LENGTH / SAMPLE_RATE);   // ~69 bins

    // Ensure we're within bounds
    if (start_bin < 1) start_bin = 1;
    if (end_bin >= FFT_LENGTH/2) end_bin = FFT_LENGTH/2 - 1;

    for (uint32_t i = start_bin; i <= end_bin; i++) {
        if (fftMagnitude[i] > maxMagnitude) {
            maxMagnitude = fftMagnitude[i];
            maxIndex = i;
        }
    }

    // 5. Convert bin index to frequency
    float32_t frequency = (float32_t)maxIndex * SAMPLE_RATE / FFT_LENGTH;

    // 6. Optional: Apply quadratic interpolation for better accuracy
    if (maxIndex > start_bin && maxIndex < end_bin) {
        float32_t y0 = fftMagnitude[maxIndex - 1];
        float32_t y1 = fftMagnitude[maxIndex];
        float32_t y2 = fftMagnitude[maxIndex + 1];

        // Quadratic interpolation formula
        float32_t delta = (y2 - y0) / (2.0f * (2.0f * y1 - y2 - y0));
        frequency = ((float32_t)maxIndex + delta) * SAMPLE_RATE / FFT_LENGTH;
    }

    // Only return frequency if we have a strong enough signal
    if (maxMagnitude < 100.0f) {  // Adjust this threshold based on signal levels
        return 0.0f;  // No significant frequency detected
    }

    return frequency;
}

// Wrapper for UART send messages
void send_msg(char* msg){
	HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 1000);
}

// ADD UART RECEIVE CALLBACK:
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        uint8_t c = uart_rx_buffer[0];

        if (c == '\r' || c == '\n') {
            uart_line[uart_pos] = '\0';    // terminate string
            uart_cmd_ready = true;
            uart_pos = 0;                  // reset for next command
        }
        else if (uart_pos < sizeof(uart_line) - 1) {
            uart_line[uart_pos++] = c;     // store character
        }

        HAL_UART_Receive_IT(&huart1, uart_rx_buffer, 1);  // restart interrupt
    }
}

// FUNCTION TO PROCESS UART COMMANDS:
void process_uart_command(char* cmd) {
    // Remove newline characters
    int len = strlen(cmd);
    while (len > 0 && (cmd[len-1] == '\r' || cmd[len-1] == '\n')) {
        cmd[len-1] = '\0';
        len--;
    }

    if (strcmp(cmd, "start") == 0 || strcmp(cmd, "s") == 0) {
        if (gameState == IDLE) {
            send_msg("Starting game...\r\n");
            gameState = PLAY_TONE;
        }
    }
    else if (strcmp(cmd, "record") == 0 || strcmp(cmd, "r") == 0) {
        if (gameState == WAIT_FOR_RECORD) {
            send_msg("Starting recording...\r\n");
            HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
            if (HAL_DFSDM_FilterRegularStart_DMA(&hdfsdm1_filter0, dfsdmBuffer, RECORD_LEN) != HAL_OK) {
                send_msg("Error starting recording!\r\n");
                gameState = IDLE;
            } else {
                gameState = RECORD_SOUND;
            }
        }
    }
    else if (strcmp(cmd, "help") == 0 || strcmp(cmd, "h") == 0) {
        send_msg("Available commands:\r\n");
        send_msg("start/s - Start the game\r\n");
        send_msg("record/r - Start recording (when prompted)\r\n");
        send_msg("help/h - Show this help\r\n");
    }
    else {
        char msg[60];
        sprintf(msg, "Unknown command: '%s'. Type 'help' for commands.\r\n", cmd);
        send_msg(msg);
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_DAC1_Init();
  MX_TIM2_Init();
  MX_DFSDM1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start(&htim2); //Don't forget to set DAC trigger tim2 for DMA

  // Initialize FFT
  if (arm_rfft_fast_init_f32(&fftInstance, FFT_LENGTH) != ARM_MATH_SUCCESS) {
      Error_Handler();
  }


  //for the random frequency:
  srand(HAL_GetTick());

  // START UART RECEPTION - ADD THIS LINE:
  HAL_UART_Receive_IT(&huart1, uart_rx_buffer, 1); //This tells the UART peripheral: "When you receive 1 byte, trigger an interrupt"

  send_msg("Pitch Matching Game Initialized!\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	if (uart_cmd_ready) {
	  process_uart_command(uart_line);
	  uart_cmd_ready = false;
	}
	switch(gameState) {
		case IDLE: {
			HAL_Delay(10);
      // waiting for user input via UART
			break;
    }
		case PLAY_TONE: {
			// Play the target frequency tone
			set_random_frequency();
			play_sound();
			send_msg("Listen to the target tone: ");
			char freq_msg[50];
			sprintf(freq_msg, "%.2f Hz\r\n", real_freq);
			send_msg(freq_msg);
			HAL_Delay(800);  // Play tone for 800ms
			HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
			send_msg("Now try to match the pitch! Send 'record' or 'r' to start recording.\r\n");
			gameState = WAIT_FOR_RECORD;
			break;
		}
			
		case WAIT_FOR_RECORD: {
			// waiting for user input via UART
			HAL_Delay(10);
			break;
		}
			
		case RECORD_SOUND:
			// Recording is in progress (handled by interrupt)
			HAL_Delay(10);
			break;
			
		case ANALYZE_RECORDING:
			// Analyze the recorded audio frequency
			send_msg("Analyzing recording with FFT...\r\n");
			// Perform frequency analysis
			recorded_freq = analyze_frequency(dfsdmBuffer, RECORD_LEN);

			if (recorded_freq > 0.0f) {
				char result_msg[100];
				sprintf(result_msg, "Detected frequency: %.2f Hz\r\n", recorded_freq);
				send_msg(result_msg);
			} else {
				send_msg("No clear frequency detected. Try singing louder!\r\n");
			}
			gameState = PLAYBACK_SOUND;

			break;
			
		case PLAYBACK_SOUND:
			// Play back the recorded sound
			__HAL_TIM_DISABLE(&htim2);
			__HAL_TIM_SET_AUTORELOAD(&htim2, 2750); //set back tim2 to 2750
			__HAL_TIM_SET_COUNTER(&htim2, 0);
			__HAL_TIM_ENABLE(&htim2);
			HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t *)dacBuffer, RECORD_LEN, DAC_ALIGN_12B_R);
			HAL_Delay(2000);  // Play for 2 seconds
			HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
			gameState = SHOW_RESULT;
			break;
			
		case SHOW_RESULT: {
			// Compare frequencies and show result
			if (recorded_freq > 0.0f) {
				float32_t diff = fabsf(recorded_freq - real_freq);
				float32_t accuracy = 100.0f - (diff / real_freq * 100.0f);

				char result_msg[150];
				sprintf(result_msg, "Target: %.2f Hz | Your pitch: %.2f Hz | Difference: %.2f Hz\r\n",
						real_freq, recorded_freq, diff);
				send_msg(result_msg);

				if (diff <= 10.0f) {  // 10Hz tolerance
					send_msg("Excellent match! 🎵\r\n");
				} else if (diff <= 50.0f) {
					send_msg("Good effort! 👍\r\n");
				} else {
					send_msg("Keep practicing! 🎤\r\n");
				}
			} else {
				send_msg("Could not detect your pitch. Try again!\r\n");
			}

			send_msg("Type 'start' to play again.\r\n");
      
			// go back to IDLE state
			gameState = IDLE;
			break;
		}
			
		default:
			gameState = IDLE;
			break;
	}

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_T2_TRGO;
  sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_ABOVE_80MHZ;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief DFSDM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DFSDM1_Init(void)
{

  /* USER CODE BEGIN DFSDM1_Init 0 */

  /* USER CODE END DFSDM1_Init 0 */

  /* USER CODE BEGIN DFSDM1_Init 1 */

  /* USER CODE END DFSDM1_Init 1 */
  hdfsdm1_filter0.Instance = DFSDM1_Filter0;
  hdfsdm1_filter0.Init.RegularParam.Trigger = DFSDM_FILTER_SW_TRIGGER;
  hdfsdm1_filter0.Init.RegularParam.FastMode = ENABLE;
  hdfsdm1_filter0.Init.RegularParam.DmaMode = ENABLE;
  hdfsdm1_filter0.Init.FilterParam.SincOrder = DFSDM_FILTER_FASTSINC_ORDER;
  hdfsdm1_filter0.Init.FilterParam.Oversampling = 55;
  hdfsdm1_filter0.Init.FilterParam.IntOversampling = 1;
  if (HAL_DFSDM_FilterInit(&hdfsdm1_filter0) != HAL_OK)
  {
    Error_Handler();
  }
  hdfsdm1_channel2.Instance = DFSDM1_Channel2;
  hdfsdm1_channel2.Init.OutputClock.Activation = ENABLE;
  hdfsdm1_channel2.Init.OutputClock.Selection = DFSDM_CHANNEL_OUTPUT_CLOCK_SYSTEM;
  hdfsdm1_channel2.Init.OutputClock.Divider = 50;
  hdfsdm1_channel2.Init.Input.Multiplexer = DFSDM_CHANNEL_EXTERNAL_INPUTS;
  hdfsdm1_channel2.Init.Input.DataPacking = DFSDM_CHANNEL_STANDARD_MODE;
  hdfsdm1_channel2.Init.Input.Pins = DFSDM_CHANNEL_SAME_CHANNEL_PINS;
  hdfsdm1_channel2.Init.SerialInterface.Type = DFSDM_CHANNEL_SPI_RISING;
  hdfsdm1_channel2.Init.SerialInterface.SpiClock = DFSDM_CHANNEL_SPI_CLOCK_INTERNAL;
  hdfsdm1_channel2.Init.Awd.FilterOrder = DFSDM_CHANNEL_FASTSINC_ORDER;
  hdfsdm1_channel2.Init.Awd.Oversampling = 1;
  hdfsdm1_channel2.Init.Offset = 0;
  hdfsdm1_channel2.Init.RightBitShift = 0x00;
  if (HAL_DFSDM_ChannelInit(&hdfsdm1_channel2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DFSDM_FilterConfigRegChannel(&hdfsdm1_filter0, DFSDM_CHANNEL_2, DFSDM_CONTINUOUS_CONV_ON) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DFSDM1_Init 2 */

  /* USER CODE END DFSDM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 2750;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : bluePB_Pin */
  GPIO_InitStruct.Pin = bluePB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(bluePB_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	// Button functionality removed - using UART commands instead
	// This callback can be left empty or removed
}

void HAL_DFSDM_FilterRegConvCpltCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter) {
	// Stop DFSDM first!
	HAL_DFSDM_FilterRegularStop_DMA(&hdfsdm1_filter0);

	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
	send_msg("Recording complete!\r\n");

	for (int i = 0; i < RECORD_LEN; i++) {
		raw = dfsdmBuffer[i] >> 8;  // 24-bit signed sample
		int32_t rawclip = raw;

		// Clip typical mic's amplitude (found ranges via mic testing)
		// This controls how buzzy/clear the audio is
		// Increasing fosr makes it more speady
		int max = 2500; //2500
		int min = -1000; //-1000
		int diff = max - min;
		if (raw >= max) rawclip = max;
		if (raw <= min) rawclip = min;
		rawclip += (-min);  // shift to stay in positive range

		dacBuffer[i] = (uint16_t)((rawclip * 4095) / diff); // scale to 12-bit DAC
	}
	
	// Move to analyze state
	if (gameState == RECORD_SOUND) {
		gameState = ANALYZE_RECORDING;
	}

}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
