/* USER CODE BEGIN Header */
/*******************************************************************************
 * File Name          : GPS_parsing.c
 * Description        : Read GPS strings and decode longitude, latitude, and time data
 *
 * Author:              Prince Patel
 * Date:                12/13/2022
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "debounce.h"
#include "HD44780.h"
#include "math.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define recBufferSize 100
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static const int16_t chequingPbPin = 0; //setting the pin assigned to each pb
static const int16_t savingsPbPin = 1;		//don't use pin 2 as it's connected
static const int16_t okPbPin = 4;		//to VCP TX
static const int16_t cancelPbPin = 3;
enum pushButton {
	none, chequing, savings, ok, cancel
};
//enumerated values for use
// with if (pbPressed==value)
//type conditional statements

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void UTC_time(int32_t myVal, char *UTCtime);
void Latitude_degree(int32_t lat, char *Latitude);
void Longitude_degree(int32_t log, char *Longitude);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// FUNCTION      : setTone
// DESCRIPTION   : Calculates the PWM Period needed to obtain the freq
//				 : passed and the duty cycle of the PAM to
//				 : 50% (1/2 of the period)
// PARAMETERS    : None
// RETURNS       : nothing
void setTone(int32_t freq) {
	int32_t pwmPeriod = 1000000000 / (freq * 250); 	//value can vary
													//between 2 and 65535
	TIM_MasterConfigTypeDef sMasterConfig;
	TIM_OC_InitTypeDef sConfigOC;
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 0;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = pwmPeriod;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = pwmPeriod / 2;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	/* adding this as ST Tech Support said PWM should be stopped before
	 * calling HAL_TIM_PWM_ConfigChannel and I've been getting flakey start-up
	 * i.e.: sometime PWM starts up, other times the line remains stuck high.
	 **************************************/
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
	/*************************************/
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}

	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.BreakFilter = 0;
	sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
	sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
	sBreakDeadTimeConfig.Break2Filter = 0;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	HAL_TIM_MspPostInit(&htim1);
}

// FUNCTION      : waitForPBRelease
// DESCRIPTION   : Loops until the PB that is currently
//				 : pressed and at a logic low
//				 : is released. Release is debounced
// PARAMETERS    : pin and port
// RETURNS       : nothing
void waitForPBRelease(const int16_t pin, const char port) {
	while (deBounceReadPin(pin, port, 10) == 0) {
		//do nothing wait for key press to be released
	}
}

// FUNCTION      : startUpLCDSplashScreen()
// DESCRIPTION   : displays Debit Demo for 2s
//                 on line 1 of the display and
//				 : Disappears
// PARAMETERS    : None
// RETURNS       : nothing
void startUpLCDSplashScreen(void) {
	char stringBuffer[16] = { 0 };
	HD44780_GotoXY(0, 0);
	snprintf(stringBuffer, 16, "   GPS Recieve");
	HD44780_PutStr(stringBuffer);
	HAL_Delay(2000);
	HD44780_ClrScr();
}

// FUNCTION      : pulsePWM
// DESCRIPTION   : Turns on the PWM for the pulseTime in ms
//                 provided and then turns off PWM
// PARAMETERS    : address of Timer Handle var (e.g.: &htim1) and pulseTime in ms
// RETURNS       : nothing
void pulsePWM(TIM_HandleTypeDef *htim1, int32_t pulseTime) {
	HAL_TIMEx_PWMN_Start(htim1, TIM_CHANNEL_1);
	HAL_Delay(pulseTime);
	HAL_TIMEx_PWMN_Stop(htim1, TIM_CHANNEL_1);
}

//  FUNCTION      : pushButtonInit
//   DESCRIPTION   : Calls deBounceInit to initialize ports that
//                   will have pushbutton on them to be inputs.
//			         intitializing PA0,PA1,PA4 and PA3
//                   Switches are assigned as follows
//                   PA0			PA1			PA4			PA3
//                   chequing		savings		ok			cancel
//
//                   Note: Don't use PA2 as it is connected to VCP TX and you'll
//                   lose printf output ability.
//   PARAMETERS    : None
//   RETURNS       : nothing
void pushButtonInit() {
	deBounceInit(chequingPbPin, 'A', 1); 		//1 = pullup resistor enabled
	deBounceInit(savingsPbPin, 'A', 1); 		//1 = pullup resistor enabled
	deBounceInit(okPbPin, 'A', 1); 			//1 = pullup resistor enabled
	deBounceInit(cancelPbPin, 'A', 1); 		//1 = pullup resistor enabled
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
	MX_ADC1_Init();
	MX_TIM1_Init();
	MX_USART2_UART_Init();
	/* USER CODE BEGIN 2 */
	printf("Ready to Receive GPS string\r\n");
	HD44780_Init();
	/* setup Port A bits 0,1,2 and 3, i.e.: PA0-PA3 for input */
	pushButtonInit();
	startUpLCDSplashScreen();

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* recChar will hold a single char received on the serial port*/
		char recChar = 0;
		/* recBuffer will have characters that are received on the serial
		 * port copied into it*/

		static char recBuffer[recBufferSize] = { 0 }; //static so that the previous
													  //values of recBuffer aren't reset each
													  //time we get here

		/* recBufferIndex is used to keep track of where we are in recBuffer.  */
		static int8_t recBufferIndex = 0;//and again static so we don't reset our
										 //value each time we get here

		static int8_t sentenceReceived = 0;	//flag that indicates a complete sentence has
											//been received. static for the same reasons
											//above

		/* result will be checked to be ok or a timeout telling us whether
		 * the char should be copied over to recBuffer, or if comm should be reset */
		HAL_StatusTypeDef result = 0;

		/*check for a char on the serial port with a timeout value */
		result = HAL_UART_Receive(&huart2, (uint8_t*) &recChar, 1, 100000); //will wait up to 10s for
		// a char and then timeout
		if (result == HAL_OK)						//then we have rec'd a char
				{
			if (recChar == '$')	//STX! so let's start storing char in the recBuffer
					{
				recBufferIndex = 0;			//$ is the first char in a sentence.
				recBuffer[recBufferIndex] = recChar;//so we've got to store it at the
													//start of our recBuffer
				recBufferIndex++;			//and inc the index so the next char
											//is stored in the next position in
											//the buffer
			} else if (recBufferIndex > 0)//then we're receiving the rest of the sentence
					{
				recBuffer[recBufferIndex] = recChar;//so place the rec'd char in the buffer
				if (recBufferIndex < recBufferSize)	//and increment the index as long as
				{						//we don't exceed the size of the buffer
					recBufferIndex++;
				} else {
					recBufferIndex = 0;	//if we're we've exceeded the size of our
					printf("recBuffer Overflow\r\n");	//buffer so let's say so
				}
			}
			if (recChar == '\r' || recChar == '\n')	//if we get a cr then we know we're at the
					{									//end of our sentence
				recBuffer[recBufferIndex] = '\0';//put a null instead of cr in the buffer
												 //'cause that's what strings need
				recBufferIndex = 0;					//and reset the buffer index
				sentenceReceived = 1;	//and set the flag 'cause we've got a
										//complete senetence now.
			}
		} else if (result == HAL_TIMEOUT) {
			printf("serial port timeout\r\n");			//let's say so
			recBufferIndex = 0;							//and start over
		}

		if (sentenceReceived) {
			/* your code goes here to parse the string inside recBuffer */
			printf("recBuffer: %s\r\n", recBuffer);
			char *sentenceId = NULL;
			sentenceId = strtok(recBuffer, ",\n");
			printf("Data content: %s\r\n", sentenceId);
			printf("\n");

			/* UTC TIME */
			char *UTCtime = NULL;
			UTCtime = strtok(NULL, ",\n");     //to extract the time from string
			int32_t myVal = atoi(UTCtime);
			printf("UTCtime: %s\r\n", UTCtime);

			UTC_time(myVal, UTCtime); //calling function  to fetch data of time from string
			/* UTC TIME */

			/* Latitude */
			char *Latitude = NULL;
			Latitude = strtok(NULL, ",\n"); //to extract the latitude from string
			printf("Latitude: %s\r\n", Latitude);
			int32_t lat = atoi(Latitude);
			Latitude_degree(lat, Latitude); //calling function  to fetch data of Latitude from string
			/* Latitude */

			/* Latitude area */
			char *Latitude_area = NULL;
			Latitude_area = strtok(NULL, ",\n"); //to extract the latitude area from string
			printf("Latitude_area: %s\r\n", Latitude_area);
			printf("\n");
			/* Latitude area */

			/* Longitude */
			char *Longitude = NULL;
			Longitude = strtok(NULL, ",\n"); //to extract the logitude from string
			printf("Longitude: %s\r\n", Longitude);
			int32_t log = atoi(Longitude);
			Longitude_degree(log, Longitude); //calling function  to fetch data of Longitude from string
			/* Longitude */

			/* Longitude area */
			char *Longitude_area = NULL;
			Longitude_area = strtok(NULL, ",\n"); //to extract the logitude area from string
			printf("Longitude_area: %s\r\n", Longitude_area);
			printf("\n");
			/* Longitude area */

			/* Position */
			char *Position = NULL;
			Position = strtok(NULL, ",\n"); //to extract the position from string
			printf("Position: %s\r\n", Position);
			int32_t position = atoi(Position);
			switch (position) {
			case 0:
				printf("Invalid Position\r\n");
				break;
			case 1:
				printf("Valid SPS\r\n");
				break;
			case 2:
				printf("Valid DGPS\r\n");
				break;
			case 3:
				printf("Valid PPS\r\n");
				break;
			}
			printf("\n");
			/* Position */

			/* Satellite used */
			char *Satellite = NULL;
			Satellite = strtok(NULL, ",\n"); //to extract the used satelite from string
			printf("%s Satellites used\r\n", Satellite);
			printf("\n");
			/* Satellite used */

			/* Precision */
			char *Precision = NULL;
			Precision = strtok(NULL, ",\n");   //to extract the HDOP from string
			printf("Horizontal dilution of precision: %s\r\n", Precision);
			printf("\n");
			/* Precision */

			/* Altitude */
			char *Altitude = NULL;
			Altitude = strtok(NULL, ",\n"); //to extract the Altitude from string
			char *Altitude_unit = NULL;
			Altitude_unit = strtok(NULL, ",\n");
			printf("Altitude: %s Meters\r\n", Altitude);
			printf("\n");
			/* Altitude */

			/* Geoid */
			char *Geoid = NULL;
			Geoid = strtok(NULL, ",\n");      //to extract the Geiod from string
			char *Geoid_Unit = NULL;
			Geoid_Unit = strtok(NULL, ",\n");
			printf("Geoid Seperation: 0%s Meters\r\n", Geoid);
			printf("\n");
			/* Geoid */

			/* DGPS Age */
			char *DGPS = NULL;
			DGPS = strtok(NULL, ",\n"); //to extract the age of DGPS from string
			printf("Age of DGPS data 0%s Seconds\r\n", DGPS);
			printf("\n");
			/* DGPS Age */

			/*Check Sum*/
			char *checkSum = NULL;
			checkSum = strtok(NULL, ",*\n"); //to extract the checksum from string
			int16_t checksum = strtoul(checkSum, NULL, 16);
			printf("checksum dec:%d hex: %x\r\n", checksum, checksum);
			/*Check Sum*/

			sentenceReceived = 0;	//as we've finished processing the sentence
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
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

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
	RCC_OscInitStruct.PLL.PLLN = 16;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV16;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2
			| RCC_PERIPHCLK_ADC;
	PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
	PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
	PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
	PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
	PeriphClkInit.PLLSAI1.PLLSAI1N = 16;
	PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
	PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
	PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
	PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_ADC1CLK;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}
	/** Configure the main internal regulator output voltage
	 */
	if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1)
			!= HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */
	/** Common config
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc1.Init.LowPowerAutoWait = DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
	hadc1.Init.OversamplingMode = DISABLE;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}
	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_5;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = { 0 };

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 0;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 9090;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 4045;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.BreakFilter = 0;
	sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
	sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
	sBreakDeadTimeConfig.Break2Filter = 0;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */
	HAL_TIM_MspPostInit(&htim1);

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

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
	huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);

	/*Configure GPIO pin : PB3 */
	GPIO_InitStruct.Pin = GPIO_PIN_3;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
// FUNCTION      : UTC_time
// DESCRIPTION   : It extracts the time from the given string
// PARAMETERS    : int32_t myVal, char *UTCtime
// RETURNS       : nothing
void UTC_time(int32_t myVal, char *UTCtime) {

	int16_t digit = 0;
	while (myVal > 0) {
		myVal = myVal / 10;
		digit++;
	}
	myVal = atoi(UTCtime);

	int16_t count = digit;
	char arr[count];
	while (count--) {
		arr[count] = myVal % 10;
		myVal /= 10;
	}
	double decimal = atof(UTCtime);
	int16_t second = (decimal - (int) decimal) * 100;

	if (digit == 5) {
		printf("hour: %d hr \r\n", arr[0]);
		printf("Minute: %d min \r\n", (arr[1] * 10) + (arr[2]));
		printf("Second: %d.%d sec \r\n", (arr[3] * 10) + (arr[4]), second);
	}

	else if (digit == 6) {
		printf("hour: %d hr \r\n", (arr[0] * 10) + (arr[1]));
		printf("Minute: %d min \r\n", (arr[1] * 10) + (arr[2]));
		printf("Second: %d sec \r\n", (arr[3] * 10) + (arr[4]));
	}

	else {
		printf("Time digits are invalid\r\n");
	}
	printf("\n");

}
// FUNCTION      : Latitude_degree
// DESCRIPTION   : It extracts the latitude from the given string
// PARAMETERS    : int32_t lat, char *Latitude
// RETURNS       : nothing
void Latitude_degree(int32_t lat, char *Latitude) {
	int16_t digit = 0;
	while (lat > 0) {
		lat = lat / 10;
		digit++;
	}
	lat = atoi(Latitude);

	int16_t count = digit;
	char arr[count];
	while (count--) {
		arr[count] = lat % 10;
		lat /= 10;
	}
	double decimal = atof(Latitude);
	int16_t second = (decimal - (int) decimal) * 10000;

	if (digit == 4) {
		printf("Degrees: %d degrees \r\n", (arr[0] * 10) + (arr[1]));
		printf("Minutes: %d minutes \r\n", (arr[2] * 10) + (arr[3]));
		printf("Decimal Degree: %d decimal degree \r\n", second);
	}

	else {
		printf("Latitude data are invalid\r\n");
	}
	printf("\n");
}
// FUNCTION      : Longitude_degree
// DESCRIPTION   : It extracts the logitude data from the given string
// PARAMETERS    : int32_t log, char *Longitude
// RETURNS       : nothing
void Longitude_degree(int32_t log, char *Longitude) {
	int16_t digit = 0;
	while (log > 0) {
		log = log / 10;
		digit++;
	}
	log = atoi(Longitude);

	int16_t count = digit;
	char arr[count];
	while (count--) {
		arr[count] = log % 10;
		log /= 10;
	}
	double decimal = atof(Longitude);
	int16_t second = (decimal - (int) decimal) * 10000;

	if (digit == 4) {
		printf("Degrees: %d degrees \r\n", (arr[0] * 10) + (arr[1]));
		printf("Minutes: %d minutes \r\n", (arr[2] * 10) + (arr[3]));
		printf("Decimal Degree: %d decimal degree \r\n", second);
	}

	else {
		printf("Latitude data are invalid\r\n");
	}
	printf("\n");
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
