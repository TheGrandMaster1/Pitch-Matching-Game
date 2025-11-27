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
#include "uart_funtions.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DAC_MAX_BUFFER_SIZE 30
#define FFT_LENGTH  1024    // Power of 2 for FFT
#define SAMPLE_RATE 44100   // Hz (reduced from 44100 to save RAM)
#define PI 3.14159265358979f
#define RECORD_LEN	102424	// samples for 2 seconds
#define SCREEN_WIDTH 90
#define SCREEN_HEIGHT 24
#define RESULT_WAIT 5 // seconds to show result screen
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
	MENU,
	PLAY_TONE,
	WAIT_FOR_RECORD,
	RECORD_SOUND,
	ANALYZE_RECORDING,
	PLAYBACK_SOUND,
	SHOW_RESULT
};

enum GameState gameState = MENU;
bool screen_drawn[7] = {false, false, false, false, false, false, false}; // Track if screen was drawn for each state
// Screen buffer: [row][column] = [height][width]
char screen_buffer[SCREEN_HEIGHT][SCREEN_WIDTH + 1]; // +1 for null terminator

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

// Cool spinner animation frames
const char* spinner[] = {"⠋", "⠙", "⠹", "⠸", "⠼", "⠴", "⠦", "⠧", "⠇", "⠏"};
#define SPINNER_FRAMES 10

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

// Screen function prototypes
void clear_screen(void);
void print_to_line(uint8_t row, uint8_t col, const char* text);
void print_screen(void);
void center_text(uint8_t row, const char* text);
void init_console(void);
void print_game_title(void);

void test_screen(void);
void menu_screen(void);
void play_tone_screen(void);
void wait_for_record_screen(void);
void record_sound_screen(bool first_draw);
void analyze_recording_screen(void);
void playback_sound_screen(bool first_draw, uint32_t elapsed_ms, uint32_t duration_ms);
void show_result_screen(void);
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
    //target_freq = 400 + (rand() % 1700);

	// Get random frequency in lena's voice range; 200-1200 Hz:
	target_freq = 200 + (rand() % 1000);

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
    // 1. Prepare data for FFT
    uint32_t samples_to_use = (data_length < FFT_LENGTH) ? data_length : FFT_LENGTH;

    // Remove DC offset
    int64_t dc_offset = 0;
    for (int i = 0; i < samples_to_use; i++) {
        dc_offset += audio_data[i];
    }
    dc_offset /= samples_to_use;

    // 2. Apply AGGRESSIVE high-pass filter - remove everything below ~200Hz
    static float32_t prev_input = 0.0f;
    static float32_t prev_output = 0.0f;
    static float32_t prev_input2 = 0.0f;
    static float32_t prev_output2 = 0.0f;

    // Two-stage high-pass filter for steeper cutoff
    float32_t alpha = 0.97f;  // More aggressive cutoff

    float32_t filtered[FFT_LENGTH];

    // First stage
    for (int i = 0; i < samples_to_use; i++) {
        float32_t current_input = (float32_t)(audio_data[i] - dc_offset);
        filtered[i] = alpha * (prev_output + current_input - prev_input);
        prev_input = current_input;
        prev_output = filtered[i];
    }

    // Second stage - filter again for steeper rolloff
    for (int i = 0; i < samples_to_use; i++) {
        float32_t current_input = filtered[i];
        filtered[i] = alpha * (prev_output2 + current_input - prev_input2);
        prev_input2 = current_input;
        prev_output2 = filtered[i];
    }

    prev_input = 0.0f;
    prev_output = 0.0f;
    prev_input2 = 0.0f;
    prev_output2 = 0.0f;

    // 3. Apply Hamming window
    for (int i = 0; i < samples_to_use; i++) {
        float32_t window = 0.54f - 0.46f * arm_cos_f32(2.0f * PI * i / (FFT_LENGTH - 1));
        fftInput[i] = filtered[i] * window;
    }

    for (int i = samples_to_use; i < FFT_LENGTH; i++) {
        fftInput[i] = 0.0f;
    }

    // 4. Perform Real FFT
    arm_rfft_fast_f32(&fftInstance, fftInput, fftOutput, 0);

    // 5. Compute magnitude spectrum
    fftMagnitude[0] = 0;
    for (int i = 1; i < FFT_LENGTH/2; i++) {
        float32_t real = fftOutput[2*i];
        float32_t imag = fftOutput[2*i + 1];
        fftMagnitude[i] = sqrtf(real*real + imag*imag);
    }

    // 6. Define search range - START AT 200Hz to avoid the 80-180Hz noise
    uint32_t min_bin = (uint32_t)(200.0f * FFT_LENGTH / SAMPLE_RATE);
    uint32_t max_bin = (uint32_t)(1500.0f * FFT_LENGTH / SAMPLE_RATE);

    if (min_bin < 2) min_bin = 2;
    if (max_bin >= FFT_LENGTH/2) max_bin = FFT_LENGTH/2 - 1;

    // 7. Find overall maximum
    float32_t max_magnitude = 0.0f;
    for (uint32_t i = min_bin; i <= max_bin; i++) {
        if (fftMagnitude[i] > max_magnitude) {
            max_magnitude = fftMagnitude[i];
        }
    }

    if (max_magnitude < 1000.0f) {
        return 0.0f;
    }

    // 8. Find ALL significant peaks
    typedef struct {
        uint32_t bin;
        float32_t magnitude;
        float32_t frequency;
    } Peak;

    Peak peaks[30];
    int peak_count = 0;
    float32_t threshold = max_magnitude * 0.15f;

    for (uint32_t i = min_bin + 1; i < max_bin - 1 && peak_count < 30; i++) {
        if (fftMagnitude[i] > threshold &&
            fftMagnitude[i] > fftMagnitude[i-1] &&
            fftMagnitude[i] > fftMagnitude[i+1]) {

            // Parabolic interpolation
            float32_t y0 = fftMagnitude[i-1];
            float32_t y1 = fftMagnitude[i];
            float32_t y2 = fftMagnitude[i+1];

            float32_t delta = 0.5f * (y2 - y0) / (2.0f * y1 - y2 - y0);
            float32_t freq = ((float32_t)i + delta) * SAMPLE_RATE / FFT_LENGTH;

            peaks[peak_count].bin = i;
            peaks[peak_count].magnitude = fftMagnitude[i];
            peaks[peak_count].frequency = freq;
            peak_count++;
        }
    }

    if (peak_count == 0) {
        return 0.0f;
    }

    // 9. Score each peak - HEAVILY favor higher frequencies
    float32_t best_fundamental = 0.0f;
    float32_t best_score = 0.0f;

    for (int i = 0; i < peak_count; i++) {
        float32_t candidate = peaks[i].frequency;
        float32_t score = 0.0f;

        // STRONG frequency weighting - higher = much better
        // 200Hz: weight = 0.4
        // 500Hz: weight = 1.0
        // 1000Hz: weight = 2.0
        // 1500Hz: weight = 3.0
        float32_t frequency_weight = candidate / 500.0f;
        if (frequency_weight < 0.4f) frequency_weight = 0.4f;
        if (frequency_weight > 3.0f) frequency_weight = 3.0f;

        score = peaks[i].magnitude * frequency_weight;

        // Check if other peaks are SUBHARMONICS of this candidate
        int subharmonic_count = 0;
        for (int div = 2; div <= 4; div++) {
            float32_t subharmonic_freq = candidate / (float32_t)div;

            if (subharmonic_freq < 200.0f) continue;

            for (int j = 0; j < peak_count; j++) {
                if (i == j) continue;

                float32_t diff = fabsf(peaks[j].frequency - subharmonic_freq);
                float32_t tolerance = subharmonic_freq * 0.15f;

                if (diff < tolerance) {
                    // Penalize heavily - this is probably not fundamental
                    score *= 0.5f;
                    subharmonic_count++;
                    break;
                }
            }
        }

        // Check for HARMONICS above this candidate
        int harmonic_count = 0;
        for (int h = 2; h <= 5; h++) {
            float32_t harmonic_freq = candidate * h;

            if (harmonic_freq > (float32_t)max_bin * SAMPLE_RATE / FFT_LENGTH) {
                continue;
            }

            for (int j = 0; j < peak_count; j++) {
                float32_t diff = fabsf(peaks[j].frequency - harmonic_freq);
                float32_t tolerance = candidate * 0.15f;

                if (diff < tolerance) {
                    score += peaks[j].magnitude * 0.3f;
                    harmonic_count++;
                    break;
                }
            }
        }

        // Huge boost if we found harmonics and no subharmonics
        if (harmonic_count > 0 && subharmonic_count == 0) {
            score *= 2.0f;
        }

        if (score > best_score) {
            best_score = score;
            best_fundamental = candidate;
        }
    }

    return best_fundamental;
}

// Add this debug function to see what the algorithm is thinking:
void debug_harmonic_analysis(void) {
    char msg[150];
    send_msg("\n=== Harmonic Analysis Debug ===\r\n");

    // Recompute to show scoring
    uint32_t min_bin = (uint32_t)(200.0f * FFT_LENGTH / SAMPLE_RATE);
    uint32_t max_bin = (uint32_t)(1200.0f * FFT_LENGTH / SAMPLE_RATE);

    float32_t max_magnitude = 0.0f;
    for (uint32_t i = min_bin; i <= max_bin; i++) {
        if (fftMagnitude[i] > max_magnitude) {
            max_magnitude = fftMagnitude[i];
        }
    }

    typedef struct {
        uint32_t bin;
        float32_t magnitude;
        float32_t frequency;
    } Peak;

    Peak peaks[30];
    int peak_count = 0;
    float32_t threshold = max_magnitude * 0.15f;

    for (uint32_t i = min_bin + 1; i < max_bin - 1 && peak_count < 30; i++) {
        if (fftMagnitude[i] > threshold &&
            fftMagnitude[i] > fftMagnitude[i-1] &&
            fftMagnitude[i] > fftMagnitude[i+1]) {

            float32_t y0 = fftMagnitude[i-1];
            float32_t y1 = fftMagnitude[i];
            float32_t y2 = fftMagnitude[i+1];

            float32_t delta = 0.5f * (y2 - y0) / (2.0f * y1 - y2 - y0);
            float32_t freq = ((float32_t)i + delta) * SAMPLE_RATE / FFT_LENGTH;

            peaks[peak_count].bin = i;
            peaks[peak_count].magnitude = fftMagnitude[i];
            peaks[peak_count].frequency = freq;
            peak_count++;
        }
    }

    sprintf(msg, "Found %d peaks. Analyzing candidates:\r\n", peak_count);
    send_msg(msg);

    // Show ALL candidates with detailed scoring
    for (int i = 0; i < peak_count; i++) {
        float32_t candidate = peaks[i].frequency;
        int harmonic_count = 0;
        int subharmonic_count = 0;

        // Check for subharmonics
        for (int div = 2; div <= 4; div++) {
            float32_t subharmonic_freq = candidate / (float32_t)div;
            if (subharmonic_freq < 100.0f) continue;

            for (int j = 0; j < peak_count; j++) {
                if (i == j) continue;
                float32_t diff = fabsf(peaks[j].frequency - subharmonic_freq);
                if (diff < subharmonic_freq * 0.15f) {
                    subharmonic_count++;
                    break;
                }
            }
        }

        // Check for harmonics
        for (int h = 2; h <= 5; h++) {
            float32_t harmonic_freq = candidate * h;
            if (harmonic_freq > 1200.0f) continue;

            for (int j = 0; j < peak_count; j++) {
                float32_t diff = fabsf(peaks[j].frequency - harmonic_freq);
                if (diff < candidate * 0.15f) {
                    harmonic_count++;
                    break;
                }
            }
        }

        sprintf(msg, "  %.1f Hz: mag %.0f, %dH/%dS\r\n",
                candidate, peaks[i].magnitude, harmonic_count, subharmonic_count);
        send_msg(msg);
    }
    send_msg("==============================\r\n\n");
}

// Wrapper for UART send messages
void send_msg(char* msg){
	// HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 1000);
	sendBuffer(huart1.Instance, (uint8_t*)msg, strlen(msg));
}

// Initialize console - print 24 lines and show loading animation
void init_console(void) {
  // Print 24 empty lines to set resolution
  char newlines[SCREEN_HEIGHT + 1];
  for(int i = 0; i < SCREEN_HEIGHT; i++) {
    newlines[i] = '\n';
  }
  newlines[SCREEN_HEIGHT] = '\0';
  HAL_UART_Transmit(&huart1, (uint8_t*)newlines, SCREEN_HEIGHT, 1000);
  
  // Show initialization screen with loading animation
  clear_screen();
  print_game_title();

  center_text(14, "Initializing System...");
  center_text(15, "Console Resolution: 90x24");
  
  center_text(16, "System Ready");
  
  // Print the screen once
  print_screen();
  
  // Show loading progress bar only on line 23 (last line) for 5 seconds
  uint32_t start_time = HAL_GetTick();
  uint32_t elapsed = 0;
  uint32_t duration = 2000;
  
  while (elapsed < duration) {
    // Calculate progress percentage
    uint32_t progress = (elapsed * 100) / duration;
    if (progress > 100) progress = 100;
    
    // Create progress bar: [====>    ] 50%
    int bar_width = 60; // Fixed bar width
    int filled = (progress * bar_width) / 100;
    
    char loading[SCREEN_WIDTH + 1];
    // Center the progress bar
    int start_pos = (SCREEN_WIDTH - bar_width - 8) / 2; // 8 for "[", "]", " ", "100%"
    int pos = 0;
    
    // Fill with spaces to center
    for(int i = 0; i < start_pos; i++) {
      loading[pos++] = ' ';
    }
    
    loading[pos++] = '[';
    for(int i = 0; i < bar_width; i++) {
      if (i < filled) {
        loading[pos++] = '=';
      } else if (i == filled && filled < bar_width) {
        loading[pos++] = '>';
      } else {
        loading[pos++] = ' ';
      }
    }
    loading[pos++] = ']';
    loading[pos++] = ' ';
    sprintf(loading + pos, "%3d%%", (int) progress);
    
    // Fill rest with spaces
    while (pos < SCREEN_WIDTH) {
      loading[pos++] = ' ';
    }
    loading[SCREEN_WIDTH] = '\0';
    
    // Update screen buffer line 23
    memcpy(screen_buffer[23], loading, SCREEN_WIDTH);
    
    // Print only line 23 with carriage return (no newline)
    char line[SCREEN_WIDTH + 1];
    memcpy(line, screen_buffer[23], SCREEN_WIDTH);
    line[SCREEN_WIDTH] = '\r';
    HAL_UART_Transmit(&huart1, (uint8_t*)line, SCREEN_WIDTH + 1, 1000);
    
    HAL_Delay(50); // Update every 50ms for smooth progress
    elapsed = HAL_GetTick() - start_time;
  }
  
  HAL_Delay(1000);
}

// Test screen
void test_screen(void) {
  // This tests the screen resolution of 90x24
  clear_screen();
  
  // Top border with line number
  char top_line[SCREEN_WIDTH + 1];
  sprintf(top_line, "1=========----------==========----------TESTSCREEN----------==========----------========90");
  print_to_line(0, 0, top_line);
  
  // Numbered lines
  for(int i = 2; i < 24; i++) {
    char line[20];
    sprintf(line, "%d", i);
    print_to_line(i - 1, 0, line);
  }
  
  // Bottom border
  char bottom_line[SCREEN_WIDTH + 1];
  sprintf(bottom_line, "24========----------==========----------[[90__24]]----------==========----------========90");
  print_to_line(23, 0, bottom_line);
  
  print_screen();
}

/**
 * Print the game title to the screen
 * From line 2 to line 12
 * From column 4 to column 40
    ____  _ __       __       __  ___      __       __    _
   / __ \(_) /______/ /_     /  |/  /___ _/ /______/ /_  (_)___  ____ _
  / /_/ / / __/ ___/ __ \   / /|_/ / __ `/ __/ ___/ __ \/ / __ \/ __ `/
 / ____/ / /_/ /__/ / / /  / /  / / /_/ / /_/ /__/ / / / / / / / /_/ /
/_/   /_/\__/\___/_/ /_/  /_/  /_/\__,_/\__/\___/_/ /_/_/_/ /_/\__, /
                                                              /____/
   _________    __  _________
  / ____/   |  /  |/  / ____/
 / / __/ /| | / /|_/ / __/
/ /_/ / ___ |/ /  / / /___
\____/_/  |_/_/  /_/_____/

 */
void print_game_title(void) {

   print_to_line(2, 4, "    ____  _ __       __       __  ___      __       __    _");
   print_to_line(3, 4, "   / __ \\(_) /______/ /_     /  |/  /___ _/ /______/ /_  (_)___  ____ _");
   print_to_line(4, 4, "  / /_/ / / __/ ___/ __ \\   / /|_/ / __ `/ __/ ___/ __ \\/ / __ \\/ __ `/");
   print_to_line(5, 4, " / ____/ / /_/ /__/ / / /  / /  / / /_/ / /_/ /__/ / / / / / / / /_/ /");
   print_to_line(6, 4, "/_/   /_/\\__/\\___/_/ /_/  /_/  /_/\\__,_/\\__/\\___/_/ /_/_/_/ /_/\\__, /");
   print_to_line(7, 4, "                                                              /____/");
   
   print_to_line(8 , 14, "   _________    __  _________");
   print_to_line(9 , 14, "  / ____/   |  /  |/  / ____/");
   print_to_line(10, 14, " / / __/ /| | / /|_/ / __/");
   print_to_line(11, 14, "/ /_/ / ___ |/ /  / / /___");
   print_to_line(12, 14, "\\____/_/  |_/_/  /_/_____/");
}


// Clear screen buffer with spaces
void clear_screen(void) {
  for(int row = 0; row < SCREEN_HEIGHT; row++) {
    for(int col = 0; col < SCREEN_WIDTH; col++) {
      screen_buffer[row][col] = ' ';
    }
    screen_buffer[row][SCREEN_WIDTH] = '\0'; // Null terminate
  }
}

// Print a string to a specific line (row) at a specific column
void print_to_line(uint8_t row, uint8_t col, const char* text) {
  if (row >= SCREEN_HEIGHT) return;
  
  int len = strlen(text);
  int max_len = SCREEN_WIDTH - col;
  if (len > max_len) len = max_len;
  
  for(int i = 0; i < len; i++) {
    if (col + i < SCREEN_WIDTH) {
      screen_buffer[row][col + i] = text[i];
    }
  }
}

// Print entire screen buffer to UART (clears screen first with newlines)
void print_screen(void) {
  // Clear screen by sending 24 newlines
  char newlines[SCREEN_HEIGHT + 1];
  for(int i = 0; i < SCREEN_HEIGHT; i++) {
    newlines[i] = '\n';
  }
  newlines[SCREEN_HEIGHT] = '\0';
  HAL_UART_Transmit(&huart1, (uint8_t*)newlines, SCREEN_HEIGHT, 1000);
  
  // Print each line
  for(int row = 0; row < SCREEN_HEIGHT; row++) {
    char line[SCREEN_WIDTH + 2];
    memcpy(line, screen_buffer[row], SCREEN_WIDTH);
    line[SCREEN_WIDTH] = '\r';
    if (row < SCREEN_HEIGHT - 1) {
      // Add newline for all lines except the last one
      line[SCREEN_WIDTH + 1] = '\n';
      HAL_UART_Transmit(&huart1, (uint8_t*)line, SCREEN_WIDTH + 2, 1000);
    } else {
      // Last line: only carriage return, no newline
      line[SCREEN_WIDTH + 1] = '\r';
      HAL_UART_Transmit(&huart1, (uint8_t*)line, SCREEN_WIDTH + 1, 1000);
    }
  }
}

// Helper function to center text on a line
void center_text(uint8_t row, const char* text) {
  int len = strlen(text);
  int start_col = (SCREEN_WIDTH - len) / 2;
  if (start_col < 0) start_col = 0;
  print_to_line(row, start_col, text);
}


// MENU state screen
void menu_screen(void) {
  clear_screen();
  
  print_game_title();

  
  // Menu options with decorative bullets
  center_text(16, "Enter 's' to start the game");
  
  // Footer
  char footer[] = "✨ Made by Group 21 of ECSE 444 - Fall 2025 ✨";
  center_text(23, footer);
  
  print_screen();
}

/**
 * Print the play tone screen
 * From line 2 to line 12
 * From column 4 to column 40
 , __  _                                                                /\/
/|/  \| |             o                                       o
 |___/| |  __,            _  _    __,    _  _  _           ,      __
 |    |/  /  |  |   | |  / |/ |  /  |   / |/ |/ |  |   |  / \_|  /
 |    |__/\_/|_/ \_/|/|_/  |  |_/\_/|/    |  |  |_/ \_/|_/ \/ |_/\___/
                   /|              /|
                   \|              \|
 */
void play_tone_screen(void) {
  clear_screen();
  
  print_to_line(2, 6, ", __  _                                                                /\\/");
  print_to_line(3, 6, "/|/  \\| |             o                                       o");
  print_to_line(4, 6, " |___/| |  __,            _  _    __,    _  _  _           ,      __");
  print_to_line(5, 6, " |    |/  /  |  |   | |  / |/ |  /  |   / |/ |/ |  |   |  / \\_|  /");
  print_to_line(6, 6, " |    |__/\\_/|_/ \\_/|/|_/  |  |_/\\_/|/    |  |  |_/ \\_/|_/ \\/ |_/\\___/");
  print_to_line(7, 6, "                   /|              /|");
  print_to_line(8, 6, "                   \\|              \\|");
  
  
  // char freq_msg[50];
  // sprintf(freq_msg, "Target Frequency: %.2f Hz", real_freq);
  // center_text(8, freq_msg);
  
  center_text(10, "Listen carefully...");
  
  print_screen();
}

/**
WAIT_FOR_RECORD state screen
 _
(_|   |                                              |
  |   |  __          ,_     _|_         ,_    _  _   |
  |   | /  \_|   |  /  |     |  |   |  /  |  / |/ |  |
   \_/|/\__/  \_/|_/   |_/   |_/ \_/|_/   |_/  |  |_/o
     /|
     \|
*/
void wait_for_record_screen(void) {
  clear_screen();
  
  print_to_line(2, 8, " _");
  print_to_line(3, 8, "(_|   |                                              |");
  print_to_line(4, 8, "  |   |  __          ,_     _|_         ,_    _  _   |");
  print_to_line(5, 8, "  |   | /  \\_|   |  /  |     |  |   |  /  |  / |/ |  |");
  print_to_line(6, 8, "   \\_/|/\\__/  \\_/|_/   |_/   |_/ \\_/|_/   |_/  |  |_/o");
  print_to_line(7, 8, "     /|");
  print_to_line(8, 8, "     \\|");
  
  center_text(16, "Try to match the pitch by singing into the microphone!");
  center_text(18, "Enter 'r' to start recording");
  
  print_screen();
}

/**
 * RECORD_SOUND state screen
 _ _ ___    ___    ___    ___    ___    ___    ___    ___    ___    ___  _ _
( | )   |  /   |  /   |  /   |  /   |  /   |  /   |  /   |  /   |  /   |( | )
|/|/ /| | / /| | / /| | / /| | / /| | / /| | / /| | / /| | / /| | / /| ||/|/
  / ___ |/ ___ |/ ___ |/ ___ |/ ___ |/ ___ |/ ___ |/ ___ |/ ___ |/ ___ |
 /_/  |_/_/  |_/_/  |_/_/  |_/_/  |_/_/  |_/_/  |_/_/  |_/_/  |_/_/  |_|
*/
void record_sound_screen(bool first_draw) {
  if (first_draw) {
    // Draw the screen only once
    clear_screen();
    
    print_to_line(2, 8, " _ _ ___    ___    ___    ___    ___    ___    ___    ___    ___    ___  _ _");
    print_to_line(3, 8, "( | )   |  /   |  /   |  /   |  /   |  /   |  /   |  /   |  /   |( | )");
    print_to_line(4, 8, "|/|/ /| | / /| | / /| | / /| | / /| | / /| | / /| | / /| | / /| ||/|/");
    print_to_line(5, 8, "  / ___ |/ ___ |/ ___ |/ ___ |/ ___ |/ ___ |/ ___ |/ ___ |/ ___ |");
    print_to_line(6, 8, " /_/  |_/_/  |_/_/  |_/_/  |_/_/  |_/_/  |_/_/  |_/_/  |_/_/  |_|");
    
    
    center_text(16, "Made you said that hahahaha...");
    
    print_screen();
  }
  
  // Update only line 23 (last line) with recording indicator using \r
  static int frame = 0;
  char indicator[SCREEN_WIDTH + 1];
  sprintf(indicator, "Recording %s", spinner[frame % SPINNER_FRAMES]);
  center_text(23, indicator);
  frame++;
  
  // Print only line 23 with carriage return (no newline)
  char line[SCREEN_WIDTH + 1];
  memcpy(line, screen_buffer[23], SCREEN_WIDTH);
  line[SCREEN_WIDTH] = '\r';
  // send_msg(line);
  HAL_UART_Transmit(&huart1, (uint8_t*)line, SCREEN_WIDTH + 1, 1000);
}

// ANALYZE_RECORDING state screen
void analyze_recording_screen(void) {
  // Update only line 23 (last line) with analysis message using \r
  center_text(23, "Analyzing your beautiful vocal...");
  
  // Print only line 23 with carriage return (no newline)
  char line[SCREEN_WIDTH + 1];
  memcpy(line, screen_buffer[23], SCREEN_WIDTH);
  line[SCREEN_WIDTH] = '\r';
  send_msg(line);
  // HAL_UART_Transmit(&huart1, (uint8_t*)line, SCREEN_WIDTH + 1, 1000);
}


/*&
 * SHOW_RESULT state screen

  ____                                            _ _
 / ___| __ _ _ __ ___   ___   _ __ ___  ___ _   _| | |_ ___
| |  _ / _` | '_ ` _ \ / _ \ | '__/ _ \/ __| | | | | __/ __|
| |_| | (_| | | | | | |  __/ | | |  __/\__ \ |_| | | |_\__ \
 \____|\__,_|_| |_| |_|\___| |_|  \___||___/\__,_|_|\__|___/

 _____ _____ _____ _____ _____ _____ _____ _____ _____ _____ _____ _____
|_____|_____|_____|_____|_____|_____|_____|_____|_____|_____|_____|_____|
*/
void show_result_screen(void) {
  clear_screen();
  
  print_to_line(2, 4, "  ____                                            _ _");
  print_to_line(3, 4, " / ___| __ _ _ __ ___   ___   _ __ ___  ___ _   _| | |_ ___");
  print_to_line(4, 4, "| |  _ / _` | '_ ` _ \\ / _ \\ | '__/ _ \\/ __| | | | | __/ __|");
  print_to_line(5, 4, "| |_| | (_| | | | | | |  __/ | | |  __/\\__ \\ |_| | | |_\\__ \\");
  print_to_line(6, 4, " \\____|\\__,_|_| |_| |_|\\___| |_|  \\___||___/\\__,_|_|\\__|___/");
  
  print_to_line(8, 4, " _____ _____ _____ _____ _____ _____ _____ _____ _____ _____ _____ _____");
  print_to_line(9, 4, "|_____|_____|_____|_____|_____|_____|_____|_____|_____|_____|_____|_____|");
  
  if (recorded_freq > 0.0f) {
    float32_t diff = fabsf(recorded_freq - real_freq);
    float32_t ratio = (recorded_freq < real_freq) ? recorded_freq/real_freq : real_freq/recorded_freq;
    float32_t accuracy = ratio * 100.0f;
    
    char target_msg[50];
    sprintf(target_msg, "Target: %.2f Hz", real_freq);
    center_text(14, target_msg);
    
    char your_msg[50];
    sprintf(your_msg, "Your pitch: %.2f Hz", recorded_freq);
    center_text(15, your_msg);
    
    char diff_msg[50];
    sprintf(diff_msg, "Difference: %.2f Hz", diff);
    center_text(16, diff_msg);
    
    char acc_msg[50];
    sprintf(acc_msg, "Accuracy: %.2f%%", accuracy);
    center_text(17, acc_msg);
    
    // Result message
    if (diff <= 10.0f) {
      center_text(20, "Excellent match!");
    } else if (diff <= 50.0f) {
      center_text(20, "Good effort!");
    } else {
      center_text(20, "Keep practicing!");
    }
  } else {
    center_text(20, "Womp womp :P Could not detect your pitch");
    center_text(21, "Try again!");
  }
  
  print_screen();
}

// ADD UART RECEIVE CALLBACK:
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//    if (huart->Instance == USART1)
//    {
//        uint8_t c = uart_rx_buffer[0];
//
//        if (c == '\r' || c == '\n') {
//            uart_line[uart_pos] = '\0';    // terminate string
//            uart_cmd_ready = true;
//            uart_pos = 0;                  // reset for next command
//        }
//        else if (uart_pos < sizeof(uart_line) - 1) {
//            uart_line[uart_pos++] = c;     // store character
//        }
//
//        HAL_UART_Receive_IT(&huart1, uart_rx_buffer, 1);  // restart interrupt
//    }
//}

// FUNCTION TO PROCESS UART COMMANDS:
void process_uart_command(char* cmd) {
    // Remove newline characters
    int len = strlen(cmd);
    while (len > 0 && (cmd[len-1] == '\r' || cmd[len-1] == '\n')) {
        cmd[len-1] = '\0';
        len--;
    }

    if (strcmp(cmd, "start") == 0 || strcmp(cmd, "s") == 0 || strcmp(cmd, "1") == 0) {
        if (gameState == MENU) {
            // Reset screen flags
            for(int i = 0; i < 7; i++) {
                screen_drawn[i] = false;
            }
            gameState = PLAY_TONE;
        }
    }
    else if (strcmp(cmd, "record") == 0 || strcmp(cmd, "r") == 0) {
        if (gameState == WAIT_FOR_RECORD) {
            send_msg("Starting recording...\r\n");
            HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
            if (HAL_DFSDM_FilterRegularStart_DMA(&hdfsdm1_filter0, dfsdmBuffer, RECORD_LEN) != HAL_OK) {
                send_msg("Error starting recording!\r\n");
                gameState = MENU;
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

  // Initialize console - print 24 lines and show loading animation
  init_console();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	if (uart_cmd_ready) {
    /**
     * possible commands includes:
     * start/s - Start the game
     * record/r - Start recording (when prompted)
     * help/h - Show this help
     */
	  process_uart_command(uart_line);
	  uart_cmd_ready = false;
	}
	switch(gameState) {
		case MENU: {
			if (!screen_drawn[MENU]) {
				menu_screen();
				screen_drawn[MENU] = true;
			}
			HAL_Delay(10);
			break;
    }
		case PLAY_TONE: {
			if (!screen_drawn[PLAY_TONE]) {
				// Play the target frequency tone
				set_random_frequency();
				play_tone_screen();
				play_sound();
				screen_drawn[PLAY_TONE] = true;
			}
			HAL_Delay(3000);  // Play tone for 3 seconds
			HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
			// Reset flags and move to next state
			screen_drawn[PLAY_TONE] = false;
			screen_drawn[WAIT_FOR_RECORD] = false;
			gameState = WAIT_FOR_RECORD;
			break;
		}
			
		case WAIT_FOR_RECORD: {
			if (!screen_drawn[WAIT_FOR_RECORD]) {
				wait_for_record_screen();
				screen_drawn[WAIT_FOR_RECORD] = true;
			}
			HAL_Delay(10);
			break;
		}
			
		case RECORD_SOUND:
			// Update screen periodically to show animation
			static uint32_t last_record_update = 0;
			if (HAL_GetTick() - last_record_update > 200) {
				record_sound_screen(!screen_drawn[RECORD_SOUND]);
				if (!screen_drawn[RECORD_SOUND]) {
					screen_drawn[RECORD_SOUND] = true;
				}
				last_record_update = HAL_GetTick();
			}
			// Recording is in progress (handled by interrupt)
			HAL_Delay(10);
			break;
			
		case ANALYZE_RECORDING:
			// Show analysis screen with animation
			static uint32_t last_analyze_update = 0;
			if (HAL_GetTick() - last_analyze_update > 200) {
				analyze_recording_screen();
				last_analyze_update = HAL_GetTick();
			}
			// Perform frequency analysis
			recorded_freq = analyze_frequency(dfsdmBuffer, RECORD_LEN);
			// Note: debug_harmonic_analysis() uses send_msg for debug output
			
			// Reset flags and move directly to SHOW_RESULT (skip playback)
			screen_drawn[ANALYZE_RECORDING] = false;
			screen_drawn[SHOW_RESULT] = false;
			gameState = SHOW_RESULT;
			break;
			
		case SHOW_RESULT: {
			if (!screen_drawn[SHOW_RESULT]) {
				show_result_screen();
				screen_drawn[SHOW_RESULT] = true;
			}
			// Reset all flags before go back to MENU state
			for(int i = 0; i < 7; i++) {
				screen_drawn[i] = false;
			}

			// INSERT_YOUR_CODE
			// Countdown from 10 to 0 with \r on result screen
			for (int sec = RESULT_WAIT; sec >= 0; sec--) {
				char countdown_line[SCREEN_WIDTH + 1];
				// Format message: "Returning to menu in 10...", centered
				char msg[32];
				sprintf(msg, "Returning to menu in %2d...", sec);
				memset(countdown_line, ' ', SCREEN_WIDTH);
				int msg_len = strlen(msg);
				int start_pos = (SCREEN_WIDTH - msg_len) / 2;
				if (start_pos < 0) start_pos = 0;
				memcpy(countdown_line + start_pos, msg, msg_len);
				countdown_line[SCREEN_WIDTH] = '\r'; // carriage return
				HAL_UART_Transmit(&huart1, (uint8_t*)countdown_line, SCREEN_WIDTH + 1, 1000);
				HAL_Delay(1000); // Wait 1 second
			}

			gameState = MENU;
			break;
		}
			
		default:
			// Reset all flags
			for(int i = 0; i < 7; i++) {
				screen_drawn[i] = false;
			}
			gameState = MENU;
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
  hdfsdm1_filter0.Init.FilterParam.SincOrder = DFSDM_FILTER_SINC1_ORDER;
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
	// send_msg("Recording complete!\r\n");

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
#ifdef USE_FULL_ASSERT
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
