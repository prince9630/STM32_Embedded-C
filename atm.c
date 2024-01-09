/* USER CODE BEGIN Header */
/*******************************************************************************
  * File Name          : atm.c
  * Description        : Debit machine
  *
  * Author:              Prince Patel
  * Date:                2022-11-30
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <unistd.h>
#include <stdio.h>
#include "debounce.h"
#include "ssd1331.h"
#include "fonts.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

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

int pin ;
//enumerated values for use with if
//(pbPressed == value) type conditional
//statements

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// FUNCTION      : setTone
// DESCRIPTION   : Calculates the PWM Period needed to obtain the freq
//				 : passed and the duty cycle of the PAM to
//				 : 50% (1/2 of the period)
// PARAMETERS    : int32 freq - frequency of the output
// RETURNS       : nothing
void setTone(int32_t freq) {
	int32_t pwmPeriod = 1000000000 / (freq * 250); //value can vary between 2 and 65535
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
// PARAMETERS    : pin - pin number
//                 port- port letter ie 'A'
// RETURNS       : nothing
void waitForPBRelease(const int16_t pin, const char port) {
	while (deBounceReadPin(pin, port, 10) == 0) {
		//do nothing wait for key press to be released
	}
}

// FUNCTION      : startUpOLEDSplashScreen()
// DESCRIPTION   : displays Debit Demo for 2s
//                 on line 1 of the display and
//				 : Disappears
// PARAMETERS    : None
// RETURNS       : nothing
void startUpOLEDSplashScreen(void) {
	char stringBuffer[16] = { 0 };
	snprintf(stringBuffer, 16, "   Debit Demo");
	ssd1331_display_string(0, 0, stringBuffer, FONT_1206, WHITE);
	HAL_Delay(2000);
	ssd1331_clear_screen(BLACK);
}

// FUNCTION      : pulsePWM
// DESCRIPTION   : Turns on the PWM for the pulseTime in ms
//                 provided and then turns off PWM
// PARAMETERS    : address of Timer Handle var (e.g.: &htim1)
//                 pulseTime in ms
// RETURNS       : nothing
void pulsePWM(TIM_HandleTypeDef *htim1, int32_t pulseTime) {
	HAL_TIMEx_PWMN_Start(htim1, TIM_CHANNEL_1);
	HAL_Delay(pulseTime);
	HAL_TIMEx_PWMN_Stop(htim1, TIM_CHANNEL_1);
}

//  FUNCTION      : pushButtonInit
//   DESCRIPTION   : Calls deBounceInit to initialize ports that
//                   will have pushbutton on them to be inputs.
//			         Initializing PA0,PA1,PA4 and PA3
//                   Switches are assigned as follows
//                   PA0			PA1			PA4			PA3
//                   chequing		savings		ok			cancel
//
//                   Note: Don't use PA2 as it is connected to VCP TX and you'll
//                   lose printf output ability.
//   PARAMETERS    : None
//   RETURNS       : nothing
void pushButtonInit(void) {
	deBounceInit(chequingPbPin, 'A', 1); 		//1 = pullup resistor enabled
	deBounceInit(savingsPbPin, 'A', 1); 		//1 = pullup resistor enabled
	deBounceInit(okPbPin, 'A', 1); 			//1 = pullup resistor enabled
	deBounceInit(cancelPbPin, 'A', 1); 		//1 = pullup resistor enabled
}

// FUNCTION      : displayWelcome()
// DESCRIPTION   : clears the OLED display and displays
//                 Welcome on line 1 of the display
// PARAMETERS    : None
// RETURNS       : nothing
void displayWelcome(void) {
	char stringBuffer[16] = { 0 };

	ssd1331_clear_screen(BLACK);
	snprintf(stringBuffer, 16, "Welcome ");
	ssd1331_display_string(0, 0, stringBuffer, FONT_1206, WHITE);
}

// FUNCTION      : displayAmount()
// DESCRIPTION   : clears the OLED display and displays
//                 the $amount received on line 1 of the display
// PARAMETERS    : float - amount to display
// RETURNS       : nothing
void displayAmount(float amount) {
	char stringBuffer[16] = { 0 };

	ssd1331_clear_screen(BLACK);
	snprintf(stringBuffer, 16, "$%.2f", amount);
	ssd1331_display_string(0, 0, stringBuffer, FONT_1206, WHITE);
}

// FUNCTION      : checkIfAmountRecd()
// DESCRIPTION   :
// PARAMETERS    : none
// RETURNS       : float, the amount in $ to be debited
float checkIfAmountRecd() {
	float debitAmount = 0;
	printf("waiting for debitAmount to be received on serial port\r\n");
	int16_t result = 0;
	result = scanf("%f", &debitAmount);
	if (result == 0)		//then somehow non-float chars were entered
	{						//and nothing was assigned to %f
		fpurge(STDIN_FILENO); //clear the last erroneous char(s) from the input stream
	}
	return debitAmount;
}

// FUNCTION      : checkOkOrCancel()
// DESCRIPTION   : Checks whether the OK or Cancel
//                 button has been pressed.
// PARAMETERS    : none
// RETURNS       : int8_t, 3 if cancel pressed, 4 if ok
//                 ok pressed. 0 returned if neither
//                 has pressed.
enum pushButton checkOkOrCancel(void) {
	if (deBounceReadPin(cancelPbPin, 'A', 10) == 0) {
		//then the cancel pushbutton has been pressed
		return cancel;
	} else if (deBounceReadPin(okPbPin, 'A', 10) == 0) {
		//then ok pressed
		return ok;
	}
	return none; //as ok or cancel was not pressed.
}

// FUNCTION      : displayOkOrCancel()
// DESCRIPTION   : displays "OK or Cancel?" on line 2 of OLED
// PARAMETERS    : none
// RETURNS       : nothing.
void displayOkCancel(void) {
	char stringBuffer[16] = { 0 };
	snprintf(stringBuffer, 16, "OK or Cancel?");
	ssd1331_display_string(0, 10, stringBuffer, FONT_1206, WHITE);
}
// FUNCTION      : checkchequingOrSavings()
// DESCRIPTION   : Checks whether the Chequing or Saving
//                 button has been pressed.
// PARAMETERS    : none
// RETURNS       : int8_t, 3 if cancel pressed,  0 if
//                 cgequing is pressed, 1 if savings is pressed . 0 returned if neither
//                 has pressed.

enum pushButton checkchequingOrSavings(void) {
	if (deBounceReadPin(chequingPbPin, 'A', 10) == 0) {
		                              //then the cancel pushbutton has been pressed
		return chequing;
	} else if (deBounceReadPin(savingsPbPin, 'A', 10) == 0) {
		                              //then savings pressed
		return savings;
	} else if (deBounceReadPin(cancelPbPin, 'A', 10) == 0) {
				                      //then the cancel pushbutton has been pressed
		return cancel;
	}
	return none; //as ok or cancel was not pressed.
}
// FUNCTION      : displaychequeingOrsaving()
	// DESCRIPTION   : displays "Cheq or saving?" on line 2 of OLED
	// PARAMETERS    : none
	// RETURNS       : nothing.
	void displayChequingorSaving(void) {
		char stringBuffer[25] = { 0 };
		snprintf(stringBuffer, 20, "chequing or saving?");
		ssd1331_display_string(0, 10, stringBuffer, FONT_1206, WHITE);

	}
	// FUNCTION      : displayChequeing()
		// DESCRIPTION   : displays "Chequing" on line 2 of OLED
		// PARAMETERS    : none
		// RETURNS       : nothing.
	void displayChequing(void) {
			char stringBuffer[10] = { 0 };
			snprintf(stringBuffer, 10, "Chequing");
			ssd1331_display_string(0, 10, "                    ", FONT_1206, WHITE);
			ssd1331_display_string(0, 20, stringBuffer, FONT_1206, WHITE);
		}
	 // FUNCTION      : displaySavings()
			// DESCRIPTION   : displays "saving" on OLED
	        // PARAMETERS    : none
			// RETURNS       : nothing.
		void displaySavings(void) {
				char stringBuffer[10] = { 0 };
				snprintf(stringBuffer, 10, "Savings");
				ssd1331_display_string(0, 10, "                   ", FONT_1206, WHITE);
				ssd1331_display_string(0, 20, stringBuffer, FONT_1206, WHITE);
			}
		// FUNCTION      : displayTransactioncancelled()
		// DESCRIPTION   : clears the OLED display and displays
		//                 Welcome on line 1 of the display
		// PARAMETERS    : None
		// RETURNS       : nothing
		void displayTransactioncancelled(void) {
			char stringBuffer[26] = { 0 };
			snprintf(stringBuffer, 26, "Transaction     cancelled");
			ssd1331_display_string(0, 10, "                    ", FONT_1206, WHITE);
			ssd1331_display_string(0, 20, stringBuffer, FONT_1206, WHITE);
		}
		// FUNCTION      : displayenteryourpin()
				// DESCRIPTION   : clears the OLED display and displays
				//                 Welcome on line 1 of the display
				// PARAMETERS    : None
				// RETURNS       : nothing

void enterPin(void) {
	char stringBuffer[10] = { 0 };
	ssd1331_clear_screen(BLACK);
	snprintf(stringBuffer, 10, "Enter Pin");
	ssd1331_display_string(0, 20, stringBuffer, FONT_1206, WHITE);
}

// FUNCTION      : DisplayTransactioncomplete()
// DESCRIPTION   : displays "Display transation successful" on line 2 of OLED
// PARAMETERS    : none
// RETURNS       : nothing.
void DisplayTranscation(void) {
	char stringBuffer[100] = { 0 };
	ssd1331_clear_screen(BLACK);
	snprintf(stringBuffer, 22, "Connecting with Bank", WHITE);
	ssd1331_display_string(0, 0, stringBuffer, FONT_1206, WHITE);
	snprintf(stringBuffer, 25, "Transaction Sucessful", WHITE);
	ssd1331_display_string(20, 20, stringBuffer, FONT_1206, WHITE);
}
// FUNCTION      : TransactionEnd()
// DESCRIPTION   : displays "Display Thank you..." on line 2 of OLED
// PARAMETERS    : none
// RETURNS       : nothing.
void TransactionEnd(void){
	char stringBuffer[20] = { 0 };
	ssd1331_clear_screen(BLACK);
	snprintf(stringBuffer, 20, "Thank You...", WHITE);
	ssd1331_display_string(0, 0, stringBuffer, FONT_1206, WHITE);
}
// FUNCTION      : Welcome()
// DESCRIPTION   : displays "welcome..." on line 2 of OLED
// PARAMETERS    : none
// RETURNS       : nothing.
void Welcome(void){
	char stringBuffer[10] = { 0 };
	ssd1331_clear_screen(BLACK);
	snprintf(stringBuffer, 10, "WELCOME...");
	ssd1331_display_string(0, 20, stringBuffer, FONT_1206, WHITE);
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
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

	printf("Debit Card State Machine\r\n");
	// initialize the OLED
	ssd1331_init();

	/* setup Port A bits 0,1,2 and 3, i.e.: PA0-PA3 for input */
	pushButtonInit();
	displayWelcome();

  /* USER CODE END 2 */
	printf ("\033[2J");
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
		float amount = 0;             	//used to hold the transaction amount
		static int8_t transactionState = 1;
		enum pushButton pbPressed = none;  //will hold pushbutton defined above depending on
							   	   	   	   //the pushbutton pressed
		/*states:   1   display Welcome Screen, wait for $ amount input from Serial port
		 2   @ amount Received, waiting for Ok or Cancel button
		 3   OK received, waiting for chequing or Savings button
		 4   C or S received, waiting for PIN to be entered from Serial Port
		 5   Pin Correct, send transaction data to bank. Waiting
		 for OK back from Bank If OK from Bank received. Print
		 Receipt, Record transaction. Move back to State 1.
		 6   Cancel Pressed. Display "Transaction Cancelled" back to state 1
		 */
		switch (transactionState) {
		case 1: //checking if an amount has been received
			printf("WELCOME...\r\n");
			amount = checkIfAmountRecd();
			if (amount != 0)        //returns a 0 if an transaction amount has
			{ 						//NOT been received on the serial port.
				displayAmount(amount); //but if we're we've received a debitAmount
				displayOkCancel();	//so display it and the prompt ok or cancel
				printf("Amount Entered: $%.2f\r\n",amount);
				transactionState = 2;//and do that before we move on to state 2
			}
			break;
		case 2: 						//amount has been received waiting for
			pbPressed = checkOkOrCancel();

			if (pbPressed != none) {
				if (pbPressed == cancel) {
				    //then cancel was pressed.
					printf("Cancel Pressed\r\n");
					displayTransactioncancelled();
					HAL_Delay(2000);
					transactionState = 6;
				} else if (pbPressed == ok) {
					//then ok pressed
					printf("OK Pressed\r\n");
					transactionState = 3;
					//more code needed to get to state 3 goes here
					displayChequingorSaving();
				}
			}
			break;
		case 3:
			pbPressed = checkchequingOrSavings();            // check if push button is pushed for chequing or saving
			if (pbPressed != none) {
				if (pbPressed == cancel){
					printf("Cancel Pressed\r\n");             //print cancel pressed
					displayTransactioncancelled();
					HAL_Delay(1000);
					transactionState = 6;
				}else if (pbPressed == chequing) {              //print chequing pressed
					printf("Chequing Pressed\n");
					displayChequing();
					HAL_Delay(1000);
					transactionState = 4;
				}else if (pbPressed == savings) {               //print saving pressed
					printf("Savings Pressed\n");
					displaySavings();
					HAL_Delay(1000);
					transactionState = 4;
				}
			}
			break;
		case 4:
				HAL_Delay(1000);
				enterPin();
				printf("Enter four Digit pin\r\n");          //print enter a pin on putty
				scanf("%d",&pin);
				printf("****\r\n");
				if (pin==1234)
				{                                             // pin is correct , so moving to next state
					printf("pin accepted\r\n");
					printf("transaction processing\r\n");
				    transactionState = 5;
				} else
				{
					printf("incorrect pin\r\n");          // goes to cancelled
					printf("transaction cancelled\r\n");
					displayTransactioncancelled();
					transactionState = 6;
				}
			break;
		case 5:
			//code needed here
			HAL_Delay(1000);
			printf("Sending transaction Data to bank\n\r");
			HAL_Delay(1000);
			printf("Transaction is sucessful\r\n");
			HAL_Delay(1000);
			DisplayTranscation();              //to display transaction is done on OLED
			transactionState = 6;
			break;
		case 6:
			//code needed here
			HAL_Delay(1000);
			printf("ThankYou\r\n");
			HAL_Delay(2000);
			TransactionEnd();                  //thank you message on screen
			Welcome();
			transactionState = 1;
			break;
		default:
			transactionState = 1;
			break;
		} //closing brace for switch

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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
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
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 100;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
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
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
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
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD3_Pin|SSD1331_CS_Pin|SSD1331_DC_Pin|SSD1331_RES_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LD3_Pin SSD1331_CS_Pin SSD1331_DC_Pin SSD1331_RES_Pin */
  GPIO_InitStruct.Pin = LD3_Pin|SSD1331_CS_Pin|SSD1331_DC_Pin|SSD1331_RES_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
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
