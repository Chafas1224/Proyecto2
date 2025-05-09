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
#include "fatfs.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ili9341.h"
#include "bitmaps.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "fatfs_sd.h"
#include "string.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
SPI_HandleTypeDef hspi1;
FATFS fs;
FATFS *pfs;
FIL fil;
FRESULT fres;
DWORD fre_clust;
uint32_t totalSpace, freeSpace;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
extern const unsigned char sprite_rojo[];
extern const unsigned char sprite_azul[];
extern const unsigned char sprite_manzana[];
extern const unsigned char sprite_puntero[];

#define CELL_SIZE 20
#define GRID_ROWS 12
#define GRID_COLS 16

#define COLOR_LIGHT_GREEN 0x07E0
#define COLOR_DARK_GREEN  0x03E0
#define COLOR_SNAKE_RED   0xF800
#define COLOR_FUCHSIA 0xF81F
#define DIR_QUEUE_SIZE 2

uint8_t grid[GRID_ROWS][GRID_COLS];

typedef struct {
	int row;
	int col;
} SnakeSegment;

SnakeSegment snake[100]; // máximo 100
int snakeLength = 3;
uint8_t cuerpoExternoFlag[100];


#define COLOR_SNAKE_BLUE 0x001F
SnakeSegment snake2[100];
int snake2Length = 3;
char snake2Direction = 'L';
char lastDirection2 = 'L';

uint8_t rxData;

char directionQueue[DIR_QUEUE_SIZE];
uint8_t dirQueueIndex = 0;
char snakeDirection = 'R';
char lastDirection = 'R';

#define COLOR_FOOD 0xFFFF
#define FOOD_VALUE 3
uint8_t gameOver = 0;
uint8_t ganador = 0;

#define NUM_MANZANAS 4
int foodRow[NUM_MANZANAS];
int foodCol[NUM_MANZANAS];

#define NUM_MODOS 2
int puntero_pos = 0;
const int puntero_y[NUM_MODOS] = {100, 145};
const int puntero_x = 250;
#define PUNTERO_ANCHO 20
#define PUNTERO_ALTO 20
#define COLOR_FONDO_MENU 0x0584

uint8_t modoCooperativo = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
void SnakeGame_DrawGridBackground(void);
void Snake_Init(void);
void Snake_Move(char direction);
void DrawCell(int row, int col, uint16_t color);
void Snake2_Move(char direction);
void DibujarSegmentoSerpiente(int fila, int columna, int tipo, char direccion);
char CalcularDireccion(SnakeSegment desde, SnakeSegment hacia);
void DibujarSegmentoSerpienteAzul(int fila, int columna, int tipo, char direccion);
void ReponerUnaManzana(int index);
void DibujarManzana(int row, int col);
void SpawnAllFood(void);
void LeerYMostrarBMPDesdeSD(const char* filename);
void DibujarPunteroMenu(int nuevo, int anterior);
void VerificarColisiones(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	char uart_stop = 'h';
  /* USER CODE END 1 */

  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_FATFS_Init();

  /* USER CODE BEGIN 2 */
  LCD_Init();
  LCD_Clear(0x0000);

  if (f_mount(&USERFatFS, USERPath, 1) == FR_OK) {
      LeerYMostrarBMPDesdeSD("menu.bmp");

      HAL_UART_Transmit(&huart3, (uint8_t*)"h", 1, HAL_MAX_DELAY);  // detener música anterior
      HAL_Delay(200);
      HAL_UART_Transmit(&huart3, (uint8_t*)"K", 1, HAL_MAX_DELAY); // música de menú
      HAL_UART_Transmit(&huart2, (uint8_t*)"K", 1, HAL_MAX_DELAY);

  } else {
    LCD_Print("Error mount SD", 10, 10, 1, 0xFFFF, 0x0000);
  }

  char tecla = 0;
  int anterior_pos = puntero_pos;

  while (1)
  {
    DibujarPunteroMenu(puntero_pos, puntero_pos);
    HAL_UART_Transmit(&huart3, (uint8_t*)"h", 1, HAL_MAX_DELAY);
    HAL_Delay(200);
    HAL_UART_Transmit(&huart3, (uint8_t*)"K", 1, HAL_MAX_DELAY);

    // Esperar selección de modo
    while (1) {
      HAL_UART_Receive(&huart3, (uint8_t*)&tecla, 1, HAL_MAX_DELAY);

      if ((tecla == 'U' || tecla == 'u') && puntero_pos > 0) {
        anterior_pos = puntero_pos;
        puntero_pos--;
        DibujarPunteroMenu(puntero_pos, anterior_pos);
      } else if ((tecla == 'D' || tecla == 'd') && puntero_pos < NUM_MODOS - 1) {
        anterior_pos = puntero_pos;
        puntero_pos++;
        DibujarPunteroMenu(puntero_pos, anterior_pos);
      } else if (tecla == 'X' || tecla == 'x') {
        break;  // modo elegido
      }
    }
    modoCooperativo = (puntero_pos == 1);
    HAL_UART_Transmit(&huart3, (uint8_t*)&uart_stop, 1, HAL_MAX_DELAY);
    HAL_Delay(200);
    char uart_modo = modoCooperativo ? 'H' : 'J';
    HAL_UART_Transmit(&huart3, (uint8_t*)&uart_modo, 1, HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, (uint8_t*)&uart_modo, 1, HAL_MAX_DELAY);

    // Iniciar juego
    LCD_Clear(0x0000);
    SnakeGame_DrawGridBackground();
    Snake_Init();
    SpawnAllFood();
    HAL_UART_Receive_IT(&huart3, &rxData, 1);
    gameOver = 0;
    ganador = 255;

    while (!gameOver) {
        HAL_Delay(150);

        // Procesar dirección de Jugador 1
        if (dirQueueIndex > 0) {
            char nextDir = directionQueue[0];
            for (int i = 0; i < dirQueueIndex - 1; i++)
                directionQueue[i] = directionQueue[i + 1];
            dirQueueIndex--;

            if ((nextDir == 'U' && lastDirection != 'D') ||
                (nextDir == 'D' && lastDirection != 'U') ||
                (nextDir == 'L' && lastDirection != 'R') ||
                (nextDir == 'R' && lastDirection != 'L')) {
                snakeDirection = nextDir;
            }
        }

        // Mover ambas serpientes
        Snake_Move(snakeDirection);
        Snake2_Move(snake2Direction);

        // Verificar colisiones una sola vez después de ambos movimientos
        VerificarColisiones();

        // Salir si hubo colisión
        if (gameOver) {
            char mensaje[50];
            sprintf(mensaje, "Se detectó gameOver. Ganador: %d\r\n", ganador);
            HAL_UART_Transmit(&huart3, (uint8_t*)"h", 1, HAL_MAX_DELAY);
            HAL_Delay(200);
            HAL_UART_Transmit(&huart3, (uint8_t*)"K", 1, HAL_MAX_DELAY);
            HAL_UART_Transmit(&huart2, (uint8_t*)mensaje, strlen(mensaje), HAL_MAX_DELAY);
            break;
        }

        // Actualizar direcciones anteriores
        lastDirection = snakeDirection;
        lastDirection2 = snake2Direction;
    }


    LCD_Clear(0x0000);
    HAL_Delay(50);     // estabilizar

    HAL_Delay(50);     // esperar

    char msg[50];
    sprintf(msg, "Mostrando pantalla. Ganador: %d\r\n", ganador);
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    // Mostrar imagen según el ganador
    if (modoCooperativo) {
        LeerYMostrarBMPDesdeSD("Perdieron.bmp");
        char puntuacion[40];
        sprintf(puntuacion, "%02d", snakeLength);
        LCD_Print(puntuacion, 230, 55, 2, 0xFFFF, 0x0000);  //Poner la puntuacion en el fondo
    } else {
        if (ganador == 1) LeerYMostrarBMPDesdeSD("Gana_jugador_rojo.bmp");
        else if (ganador == 2) LeerYMostrarBMPDesdeSD("Gana_jugador_azul.bmp");
    }


//Si se presiona la letra o o O volverá al menu
    HAL_UART_AbortReceive_IT(&huart3);
    char teclaFin = 0;
    while (1) {
        HAL_UART_Receive(&huart3, (uint8_t*)&teclaFin, 1, HAL_MAX_DELAY);
        if (teclaFin == 'O' || teclaFin == 'o') break;
    }

    // Volver al menú
    LCD_Clear(0x0000);
    HAL_UART_Transmit(&huart3, (uint8_t*)"h", 1, HAL_MAX_DELAY);
    HAL_Delay(200);
    HAL_UART_Transmit(&huart3, (uint8_t*)"K", 1, HAL_MAX_DELAY);
    LeerYMostrarBMPDesdeSD("menu.bmp");

    puntero_pos = 0;
  }
  /* USER CODE END 2 */
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, F_CS_Pin|LCD_RST_Pin|LCD_D1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LCD_RD_Pin|LCD_WR_Pin|LCD_RS_Pin|LCD_D7_Pin
                          |LCD_D0_Pin|LCD_D2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LCD_CS_Pin|LCD_D6_Pin|LCD_D3_Pin|LCD_D5_Pin
                          |LCD_D4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SD_SS_GPIO_Port, SD_SS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : F_CS_Pin LCD_RST_Pin LCD_D1_Pin */
  GPIO_InitStruct.Pin = F_CS_Pin|LCD_RST_Pin|LCD_D1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_RD_Pin LCD_WR_Pin LCD_RS_Pin LCD_D7_Pin
                           LCD_D0_Pin LCD_D2_Pin */
  GPIO_InitStruct.Pin = LCD_RD_Pin|LCD_WR_Pin|LCD_RS_Pin|LCD_D7_Pin
                          |LCD_D0_Pin|LCD_D2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_CS_Pin LCD_D6_Pin LCD_D3_Pin LCD_D5_Pin
                           LCD_D4_Pin */
  GPIO_InitStruct.Pin = LCD_CS_Pin|LCD_D6_Pin|LCD_D3_Pin|LCD_D5_Pin
                          |LCD_D4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SD_SS_Pin */
  GPIO_InitStruct.Pin = SD_SS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(SD_SS_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void Snake_Init(void) { //Posiciona 2 serpientes en el grid, resetea variables y dibuja visualmente cada segmento
    // Limpiar grid
    for (int r = 0; r < GRID_ROWS; r++)
        for (int c = 0; c < GRID_COLS; c++)
            grid[r][c] = 0;

    // Limpiar las serpientes
    for (int i = 0; i < 100; i++) {
        snake[i].row = 0;
        snake[i].col = 0;
        snake2[i].row = 0;
        snake2[i].col = 0;
    }

    snakeLength = 3;
    snake[0].row = 8;  snake[0].col = 5;
    snake[1].row = 8;  snake[1].col = 4;
    snake[2].row = 8;  snake[2].col = 3;

    for (int i = 0; i < snakeLength; i++) {
        char tipo = (i == 0) ? 1 : (i == snakeLength - 1) ? 2 : 0;
        char direccion = (i == snakeLength - 1) ? CalcularDireccion(snake[i - 1], snake[i]) : snakeDirection;
        DibujarSegmentoSerpiente(snake[i].row, snake[i].col, tipo, direccion);
        grid[snake[i].row][snake[i].col] = 1;
    }

    snake2Length = 3;
    snake2[0].row = 4;  snake2[0].col = 10;
    snake2[1].row = 4;  snake2[1].col = 11;
    snake2[2].row = 4;  snake2[2].col = 12;

    for (int i = 0; i < snake2Length; i++) {
        char tipo = (i == 0) ? 1 : (i == snake2Length - 1) ? 2 : 0;
        char direccion = (i == snake2Length - 1) ? CalcularDireccion(snake2[i - 1], snake2[i]) : snake2Direction;
        DibujarSegmentoSerpienteAzul(snake2[i].row, snake2[i].col, tipo, direccion);
        grid[snake2[i].row][snake2[i].col] = 2;
    }

    // Reiniciar otras cosas también
    gameOver = 0;
    ganador = 255;
    snakeDirection = 'R';
    snake2Direction = 'L';
    lastDirection = 'R';
    lastDirection2 = 'L';
    dirQueueIndex = 0;
}



void SnakeGame_DrawGridBackground(void) //Dibuja el fondo del tablero
{
    for (int row = 0; row < GRID_ROWS; row++) {
        for (int col = 0; col < GRID_COLS; col++) {
            uint16_t color = ((row + col) % 2 == 0) ? COLOR_LIGHT_GREEN : COLOR_DARK_GREEN;
            FillRect(col * CELL_SIZE, row * CELL_SIZE, CELL_SIZE, CELL_SIZE, color);
            grid[row][col] = 0;
        }
    }
}


void DrawCell(int row, int col, uint16_t color) { //Limpia por donde pasa la serpiente
	int x = col * CELL_SIZE;
	int y = row * CELL_SIZE;
	FillRect(x, y, CELL_SIZE, CELL_SIZE, color);
}

void Snake_Move(char direction) { //Movimiento de serpientes
	if (gameOver) return;//Si hay gameover ya no se mueven las serpientes

    SnakeSegment tail = snake[snakeLength - 1];
    int checkerColor = ((tail.row + tail.col) % 2 == 0) ? COLOR_LIGHT_GREEN : COLOR_DARK_GREEN;
    DrawCell(tail.row, tail.col, checkerColor); //Para que el cuerpo se vea con el color de fondo
    grid[tail.row][tail.col] = 0;

    for (int i = snakeLength - 1; i > 0; i--) {
        snake[i] = snake[i - 1];
    }

    if (direction == 'U')
        snake[0].row = (snake[0].row - 1 + GRID_ROWS) % GRID_ROWS;
    else if (direction == 'D')
        snake[0].row = (snake[0].row + 1) % GRID_ROWS;
    else if (direction == 'L')
        snake[0].col = (snake[0].col - 1 + GRID_COLS) % GRID_COLS;
    else if (direction == 'R')
        snake[0].col = (snake[0].col + 1) % GRID_COLS;

    if (grid[snake[0].row][snake[0].col] == 1) {
        gameOver = 1;
        ganador = 2;  // Rojo se chocó consigo mismo
        return;
    } else if (grid[snake[0].row][snake[0].col] == 2) {
        gameOver = 1;
        ganador = 2;  // Rojo chocó con azul
        return;
    }

    for (int i = 0; i < NUM_MANZANAS; i++) {
    	if (snake[0].row == foodRow[i] && snake[0].col == foodCol[i]) {
    	    if (modoCooperativo) {
    	        SnakeSegment tail1 = snake[snakeLength - 1];
    	        snake[snakeLength] = tail1;
    	        snakeLength++;

    	        SnakeSegment tail2 = snake2[snake2Length - 1];
    	        snake2[snake2Length] = tail2;
    	        snake2Length++;
    	    } else {
    	        snake[snakeLength] = tail;
    	        snakeLength++;
    	    }

    	    ReponerUnaManzana(i);
    	    break;
    	}

    }


    for (int i = 1; i < snakeLength - 1; i++) {
        SnakeSegment prev = snake[i - 1];
        SnakeSegment curr = snake[i];
        SnakeSegment next = snake[i + 1];

        char dirPrev = CalcularDireccion(prev, curr);
        char dirNext = CalcularDireccion(curr, next);

        if (dirPrev != dirNext) {
            char curvaDir = 0;

            if ((dirPrev == 'U' && dirNext == 'R') || (dirPrev == 'L' && dirNext == 'D'))
                curvaDir = '2';
            else if ((dirPrev == 'R' && dirNext == 'D') || (dirPrev == 'U' && dirNext == 'L'))
                curvaDir = '3';
            else if ((dirPrev == 'D' && dirNext == 'L') || (dirPrev == 'R' && dirNext == 'U'))
                curvaDir = '0';
            else if ((dirPrev == 'L' && dirNext == 'U') || (dirPrev == 'D' && dirNext == 'R'))
                curvaDir = '1';

            DibujarSegmentoSerpiente(curr.row, curr.col, 3, curvaDir);
        } else {
        	DibujarSegmentoSerpiente(curr.row, curr.col, 0, dirPrev);
        }
    }

    int n = snakeLength;
    char direccionCabeza = CalcularDireccion(snake[1], snake[0]);
    char direccionCola = CalcularDireccion(snake[n - 2], snake[n - 1]);

    DibujarSegmentoSerpiente(snake[0].row, snake[0].col, 1, direccionCabeza);
    DibujarSegmentoSerpiente(snake[n - 1].row, snake[n - 1].col, 2, direccionCola);

    grid[snake[0].row][snake[0].col] = 1;
}



void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) { //Maneja la entrada de UART3
	if (huart->Instance == USART3) {
		char newDir = 0;

		// Dirección para movimiento
		if (rxData == 'U')
			newDir = 'U';
		else if (rxData == 'D')
			newDir = 'D';
		else if (rxData == 'L')
			newDir = 'L';
		else if (rxData == 'R')
			newDir = 'R';

		// Acciones (X y O)
		if (rxData == 'X') {

		} else if (rxData == 'O') {

		}

		char newDir2 = 0;

		// Controles de Jugador 2
		if (rxData == 'u')
			newDir2 = 'U';
		else if (rxData == 'd')
			newDir2 = 'D';
		else if (rxData == 'l')
			newDir2 = 'L';
		else if (rxData == 'r')
			newDir2 = 'R';

		if (newDir2
				&& !((newDir2 == 'U' && lastDirection2 == 'D')
						|| (newDir2 == 'D' && lastDirection2 == 'U')
						|| (newDir2 == 'L' && lastDirection2 == 'R')
						|| (newDir2 == 'R' && lastDirection2 == 'L'))) {
			snake2Direction = newDir2;
		}


		if (newDir && dirQueueIndex < DIR_QUEUE_SIZE)
			directionQueue[dirQueueIndex++] = newDir;

		HAL_UART_Transmit(&huart2, &rxData, 1, HAL_MAX_DELAY);

		HAL_UART_Receive_IT(&huart3, &rxData, 1);
	}
}

void SpawnAllFood(void) //coloca manzanas
{
    for (int i = 0; i < NUM_MANZANAS; i++) {
        do {
            foodRow[i] = rand() % GRID_ROWS;
            foodCol[i] = rand() % GRID_COLS;
        } while (grid[foodRow[i]][foodCol[i]] != 0);

        grid[foodRow[i]][foodCol[i]] = FOOD_VALUE;


        char msg[50];
        sprintf(msg, "Food %d: (%d, %d)\r\n", i, foodRow[i], foodCol[i]);
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

        DibujarManzana(foodRow[i], foodCol[i]);
    }
}



void ReponerUnaManzana(int index) //Coloca manzanas
{
    do {
        foodRow[index] = rand() % GRID_ROWS;
        foodCol[index] = rand() % GRID_COLS;
    } while (grid[foodRow[index]][foodCol[index]] != 0);

    grid[foodRow[index]][foodCol[index]] = FOOD_VALUE;
    DibujarManzana(foodRow[index], foodCol[index]);
}


void DibujarManzana(int row, int col) //Dibuja una manzana
{
    int x = col * CELL_SIZE;
    int y = row * CELL_SIZE;

    int indice = ((row + col) % 2 == 0) ? 1 : 0;

    LCD_Sprite(x, y, 20, 20, sprite_manzana, 2, indice, 0, 0, 0x0000, 0x0000);
}



void Snake2_Move(char direction) { //movimiento de serpiente azul
	if (gameOver) return;
    SnakeSegment tail = snake2[snake2Length - 1];
    SnakeSegment tailRojo = snake[snakeLength - 1];
    int checkerColor = ((tail.row + tail.col) % 2 == 0) ? COLOR_LIGHT_GREEN : COLOR_DARK_GREEN;
    DrawCell(tail.row, tail.col, checkerColor);
    grid[tail.row][tail.col] = 0;

    for (int i = snake2Length - 1; i > 0; i--) {
        snake2[i] = snake2[i - 1];
    }

    if (direction == 'U')
        snake2[0].row = (snake2[0].row - 1 + GRID_ROWS) % GRID_ROWS;
    else if (direction == 'D')
        snake2[0].row = (snake2[0].row + 1) % GRID_ROWS;
    else if (direction == 'L')
        snake2[0].col = (snake2[0].col - 1 + GRID_COLS) % GRID_COLS;
    else if (direction == 'R')
        snake2[0].col = (snake2[0].col + 1) % GRID_COLS;

    if (grid[snake2[0].row][snake2[0].col] == 2) {
        gameOver = 1;
        ganador = modoCooperativo ? 255 : 1;
        return;
    } else if (grid[snake2[0].row][snake2[0].col] == 1) {
        gameOver = 1;
        ganador = modoCooperativo ? 255 : 1;
        return;
    }



    for (int i = 0; i < NUM_MANZANAS; i++) {
    	if (snake2[0].row == foodRow[i] && snake2[0].col == foodCol[i]) {
    	    if (modoCooperativo) {
    	        snake[snakeLength] = tailRojo;
    	        snakeLength++;

    	        snake2[snake2Length] = tail;
    	        snake2Length++;
    	    } else {
    	        snake2[snake2Length] = tail;
    	        snake2Length++;
    	    }

    	    ReponerUnaManzana(i);
    	    break;
    	}

    }



    for (int i = 1; i < snake2Length - 1; i++) {
        SnakeSegment prev = snake2[i - 1];
        SnakeSegment curr = snake2[i];
        SnakeSegment next = snake2[i + 1];

        char dirPrev = CalcularDireccion(prev, curr);
        char dirNext = CalcularDireccion(curr, next);

        if (dirPrev != dirNext) {
            char curvaDir = 0;

            if ((dirPrev == 'U' && dirNext == 'R') || (dirPrev == 'L' && dirNext == 'D'))
                curvaDir = '2';
            else if ((dirPrev == 'R' && dirNext == 'D') || (dirPrev == 'U' && dirNext == 'L'))
                curvaDir = '3';
            else if ((dirPrev == 'D' && dirNext == 'L') || (dirPrev == 'R' && dirNext == 'U'))
                curvaDir = '0';
            else if ((dirPrev == 'L' && dirNext == 'U') || (dirPrev == 'D' && dirNext == 'R'))
                curvaDir = '1';

            DibujarSegmentoSerpienteAzul(curr.row, curr.col, 3, curvaDir);
        } else {
            DibujarSegmentoSerpienteAzul(curr.row, curr.col, 0, dirPrev);
        }
    }

    int n = snake2Length;
    char direccionCabeza = CalcularDireccion(snake2[1], snake2[0]);
    char direccionCola = CalcularDireccion(snake2[n - 2], snake2[n - 1]);

    DibujarSegmentoSerpienteAzul(snake2[0].row, snake2[0].col, 1, direccionCabeza);
    DibujarSegmentoSerpienteAzul(snake2[n - 1].row, snake2[n - 1].col, 2, direccionCola);

    grid[snake2[0].row][snake2[0].col] = 2;
}




void DibujarSegmentoSerpiente(int fila, int columna, int tipo, char direccion) //Dibuja cada parte de las serpientes
{
    int x = columna * CELL_SIZE;
    int y = fila * CELL_SIZE;

    int indiceBase = 0;
    char flip = 0;

    switch (tipo) {
        case 0: // Cuerpo recto
            if (direccion == 'L' || direccion == 'R') {
                indiceBase = 0 + ((fila + columna) % 2);  // 0 o 1
            } else {
                indiceBase = 4 + ((fila + columna) % 2);  // 8 o 9
            }
            break;

        case 1: // Cabeza
            if (direccion == 'R') {
                indiceBase = 2; flip = ((fila + columna) % 2) ? 0 : 2;
            } else if (direccion == 'L') {
                indiceBase = 2; flip = ((fila + columna) % 2) ? 3 : 1;
            } else if (direccion == 'U') {
                indiceBase = 6; flip = 0;  // apuntando hacia arriba
            } else if (direccion == 'D') {
                indiceBase = 6; flip = 2;  // apuntando hacia abajo
            }
            break;

        case 2: // Cola
            if (direccion == 'R') {
                indiceBase = 3; flip = ((fila + columna) % 2) ? 1 : 3;
            } else if (direccion == 'L') {
                indiceBase = 3; flip = ((fila + columna) % 2) ? 2 : 0;
            } else if (direccion == 'U') {
                indiceBase = 7; flip = 2;  // apuntando hacia arriba
            } else if (direccion == 'D') {
                indiceBase = 7; flip = 0;  // apuntando hacia abajo
            }
            break;


        case 3: // Curvas
                    if (direccion == '0') indiceBase = 8;
                    else if (direccion == '1') indiceBase = 11;
                    else if (direccion == '2') indiceBase = 10;
                    else if (direccion == '3') indiceBase = 9;
                    break;
    }

    // Determinar si usamos fondo claro u oscuro
    int offset = ((fila + columna) % 2 == 0) ? 12 : 0;  // Fondo claro: +12
    int finalIndex = indiceBase + offset;

    LCD_Sprite(x, y, CELL_SIZE, CELL_SIZE, sprite_rojo, 24, finalIndex, flip, 0,
               0x0000, 0x0000);  // Ya no se necesita transparencia ni fondo dinámico
}


void DibujarSegmentoSerpienteAzul(int fila, int columna, int tipo, char direccion) //Dibuja cada parte de las serpientes
{
    int x = columna * CELL_SIZE;
    int y = fila * CELL_SIZE;

    int indiceBase = 0;
    char flip = 0;

    switch (tipo) {
        case 0: // Cuerpo recto
            if (direccion == 'L' || direccion == 'R') {
                indiceBase = 0 + ((fila + columna) % 2);  // 0 o 1
            } else {
                indiceBase = 4 + ((fila + columna) % 2);  // 4 o 5
            }
            break;

        case 1: // Cabeza
            if (direccion == 'R') {
                indiceBase = 2; flip = ((fila + columna) % 2) ? 0 : 2;
            } else if (direccion == 'L') {
                indiceBase = 2; flip = ((fila + columna) % 2) ? 3 : 1;
            } else if (direccion == 'U') {
                indiceBase = 6; flip = 0;  // arriba
            } else if (direccion == 'D') {
                indiceBase = 6; flip = 2;  // abajo
            }
            break;

        case 2: // Cola
            if (direccion == 'R') {
                indiceBase = 3; flip = ((fila + columna) % 2) ? 1 : 3;
            } else if (direccion == 'L') {
                indiceBase = 3; flip = ((fila + columna) % 2) ? 2 : 0;
            } else if (direccion == 'U') {
                indiceBase = 7; flip = 2;
            } else if (direccion == 'D') {
                indiceBase = 7; flip = 0;
            }
            break;

        case 3: // Curvas
            if (direccion == '0') indiceBase = 8;   // ↖
            else if (direccion == '1') indiceBase = 11;  // ↗
            else if (direccion == '2') indiceBase = 10;  // ↘
            else if (direccion == '3') indiceBase = 9;   // ↙
            break;
    }

    int offset = ((fila + columna) % 2 == 0) ? 12 : 0;
    int finalIndex = indiceBase + offset;

    LCD_Sprite(x, y, CELL_SIZE, CELL_SIZE, sprite_azul, 24, finalIndex, flip, 0, 0x0000, 0x0000);
}




char CalcularDireccion(SnakeSegment desde, SnakeSegment hacia) //Determina la dirección entre dos segmentos
{
    if ((desde.row == 0 && hacia.row == GRID_ROWS - 1)) return 'U';
    if ((desde.row == GRID_ROWS - 1 && hacia.row == 0)) return 'D';
    if ((desde.col == 0 && hacia.col == GRID_COLS - 1)) return 'L';
    if ((desde.col == GRID_COLS - 1 && hacia.col == 0)) return 'R';

    if (desde.row < hacia.row) return 'D';
    if (desde.row > hacia.row) return 'U';
    if (desde.col < hacia.col) return 'R';
    return 'L';
}

void LeerYMostrarBMPDesdeSD(const char* filename) { //Muestra la imagen de 8bits desde la sd
    FIL file;
    UINT bytesRead;
    uint8_t header[54];

    if (f_open(&file, filename, FA_READ) != FR_OK) {
        LCD_Print("Error abriendo BMP", 10, 10, 1, 0xFFFF, 0x0000);
        return;
    }


    f_read(&file, header, 54, &bytesRead);
    if (bytesRead != 54 || header[0] != 'B' || header[1] != 'M') {
        LCD_Print("No es BMP válido", 10, 10, 1, 0xFFFF, 0x0000);
        f_close(&file);
        return;
    }

    uint32_t dataOffset = *(uint32_t*)&header[10];
    uint32_t width = *(uint32_t*)&header[18];
    int32_t height = *(int32_t*)&header[22];
    uint16_t bpp = *(uint16_t*)&header[28];

    if (bpp != 8) {
        LCD_Print("Solo BMP 8 bits", 10, 10, 1, 0xFFFF, 0x0000);
        f_close(&file);
        return;
    }

    int inverted = 1;
    if (height < 0) {
        height = -height;
        inverted = 0;
    }

    uint8_t palette[1024];
    f_read(&file, palette, 1024, &bytesRead);


    SetWindows(0, 0, width - 1, height - 1);
    LCD_CMD(0x2C);
    HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_SET);

    uint8_t row[320];

    for (int y = 0; y < height; y++) {
        int actualY = inverted ? (height - 1 - y) : y;
        f_lseek(&file, dataOffset + actualY * width);
        f_read(&file, row, width, &bytesRead);

        for (int x = 0; x < width; x++) {
            int index = row[x] * 4;
            uint8_t b = palette[index + 0];
            uint8_t g = palette[index + 1];
            uint8_t r = palette[index + 2];

            uint16_t color = ((r & 0xF8) << 8) |
                             ((g & 0xFC) << 3) |
                             (b >> 3);

            LCD_DATA(color >> 8);
            LCD_DATA(color);
        }
    }

    HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);
    f_close(&file);
}

void DibujarPunteroMenu(int nuevo, int anterior) { //Dibuja el puntero del menu
    // Borrar el puntero anterior
    FillRect(puntero_x, puntero_y[anterior], 20, 20, 0x0144);  // Color de fondo

    // Dibujar el puntero nuevo
    LCD_Sprite(puntero_x, puntero_y[nuevo], 20, 20, sprite_puntero, 1, 0, 0, 0, 0x0000, 0x0000);
}


void VerificarColisiones(void) { //Revisa y determina colisiones
    if (gameOver) return;

    // Verificar empate si ambas cabezas están en la misma celda
    if (snake[0].row == snake2[0].row && snake[0].col == snake2[0].col) {
        gameOver = 1;
        ganador = 0;  // Empate
        HAL_UART_Transmit(&huart3, (uint8_t*)"h", 1, HAL_MAX_DELAY);
        return;
    }

    // Rojo colisiona consigo mismo
    for (int i = 1; i < snakeLength; i++) {
        if (snake[0].row == snake[i].row && snake[0].col == snake[i].col) {
            gameOver = 1;
            ganador = 2;
            HAL_UART_Transmit(&huart3, (uint8_t*)"h", 1, HAL_MAX_DELAY);
            return;
        }
    }

    // Azul colisiona consigo mismo
    for (int i = 1; i < snake2Length; i++) {
        if (snake2[0].row == snake2[i].row && snake2[0].col == snake2[i].col) {
            gameOver = 1;
            ganador = 1;
            HAL_UART_Transmit(&huart3, (uint8_t*)"h", 1, HAL_MAX_DELAY);
            return;
        }
    }

    // Rojo colisiona con el cuerpo del azul
    for (int i = 1; i < snake2Length; i++) {
        if (snake[0].row == snake2[i].row && snake[0].col == snake2[i].col) {
            gameOver = 1;
            ganador = 2;
            HAL_UART_Transmit(&huart3, (uint8_t*)"h", 1, HAL_MAX_DELAY);
            return;
        }
    }

    // Azul colisiona con el cuerpo del rojo
    for (int i = 1; i < snakeLength; i++) {
        if (snake2[0].row == snake[i].row && snake2[0].col == snake[i].col) {
            gameOver = 1;
            ganador = 1;
            HAL_UART_Transmit(&huart3, (uint8_t*)"h", 1, HAL_MAX_DELAY);
            return;
        }
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
	while (1) {
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
