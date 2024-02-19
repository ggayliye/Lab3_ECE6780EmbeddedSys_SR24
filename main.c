/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

volatile short button_count=0;
volatile short led_lit_count=0;
volatile short timer=0;
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
	
	
	
	// Helper function for toggle handler:
void ToggleFunction(int button_count) {
 // TIM1_BRK_UP_TRG_COM_IRQHandler();
    if (button_count == 1){
			
        if (led_lit_count == 0){

            //GPIOC->BSRR  |= 0x100 ;  // ORANGE ON! GPIOC->BSRR  |= 0x100 | 0x2FF ; 

            GPIOC->ODR &= 0x2FF; //Green On
            GPIOC->ODR |= 0x200;// orange off.(GPIOC->BSRR&0x2FF) | 0x200; //make GREEN back ON. //GPIOC->ODR |= (1<<9);

        }
        led_lit_count++;
        if (led_lit_count > 1){

            GPIOC->ODR &= 0x1FF;// make GREEN OFF.  
            GPIOC->ODR |= 0x100; //orange ON

            led_lit_count = 0;
        }

        button_count = 0;
    }
}

	
	
		// Toggle between the green (PC8) and orange (PC9) LEDs in the interrupt handler
	
void TIM2_IRQHandler(void) {

		
       //button_count = 1;
			 ToggleFunction(1);

    //Don’t forget to clear the pending flag for the update interrupt in the status register:
	 
	//	TIM2->SR |= 0x1;  //Bit 0 UIF: Update interrupt flag	 
	TIM2->SR &= ~TIM_SR_UIF;
	
	
	//	
//	 timer++;
//	
//	//if(timer == 6000){
//if(led_lit_count == 0)
//{

//            //GPIOC->BSRR  |= 0x100 ;  // ORANGE ON! GPIOC->BSRR  |= 0x100 | 0x2FF ; 

//            GPIOC->ODR &= 0x2FF; //Green On
//            GPIOC->ODR |= 0x200;// orange off.(GPIOC->BSRR&0x2FF) | 0x200; //make GREEN back ON. //GPIOC->ODR |= (1<<9);

//        }
//        led_lit_count++;
//        if (led_lit_count > 1){

//            GPIOC->ODR &= 0x1FF;// make GREEN OFF.  
//            GPIOC->ODR |= 0x100; //orange ON

//            led_lit_count = 0;
//				}
//timer = 0;
////}
 
	
	


}	



	
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
  /* USER CODE BEGIN 2 */

	
	
	
	// START: Exersize 3.1 — Using Timer Interrupts:

		//Enable the timer 2 peripheral (TIM2) in the RCC
   RCC->APB1ENR |= 0x1;  //Bit 0 TIM2EN: TIM2 timer clock enable
	
//	RCC->APB2ENR |=0x800; 
	 
	 //Configure the timer to trigger an update event:
	 //The default processor frequency of the STM32F072 is 8 MHz
	 //a 16-bit timer can only count up to 65535. 
	 //If your target ARR is outside of that range, 
	 //you’ll need to adjust the prescaler (change units) to scale the ARR appropriately.
	 
	 //PSC = fCLK/(ARR * fTARGET) -1.
	 // fCLK=8MHz; fTARGET = 4Hz. 1/4 =0.25sec=250 ms.
	 //Set ARR = 250 (only 8 bits); PSC = 8MHz/(250 * 4Hz) -1 =7999 (occupies 13 bit only out of 16 bit).
		 TIM2->PSC = 7999; //0x1F3F; //7999
		 TIM2->ARR = 250;//0xFA;   //250
	 
	 //Configure the timer to generate an interrupt on the UEV event (Update Event).
	 //Use the DMA/Interrupt Enable Register (DIER) to enable the Update Interrupt:
		 TIM2->DIER |= 0x1; //Bit 0 UIE: Update interrupt enable
		// TIM2->DIER |= TIM_DIER_UIE; //
	 //egr, rcr, CR1 (have that) feedback
	 //Configure and enable/start the timer
	 //Although the RCC enabled the timer’s clock source, 
	 //the timer has its own enable/start bit in the control registers
	 
	   TIM2->CR1 |= 0x1; //Bit 0 CEN: Counter enable
	 
	 //Set up the timer’s interrupt handler, and enable in the NVIC.
	 
			NVIC_EnableIRQ(TIM2_IRQn); // In the stm32f072xb.h file.
	 
	    NVIC_SetPriority(TIM2_IRQn,1);// Set the priority for the interrupt to 1 (high-priority). 
		
	 
	 // Toggle between the green (PC8) and orange (PC9) LEDs in the interrupt handler
	 
	 		//Initializing all of the LED pins

		RCC->AHBENR |= 0x80000; //For C port (LED ports), "Bit 19 IOPCEN: I/O port C clock enable". 19th bit position. 
		// We know 19th bit from data sheet RM0091 on page 122.
		// PC6 is connected to RED LED. 
		// PC7 is connected to BLUE LED.
		// PC8 is connected to ORANGE LED.
		// PC9 is connected to GREEN LED.
	 
		//Enable ALL LEDs for "General purpose output mode, "01" as bits"
		GPIOC->MODER |= (85<<12); // This means: Move Right to left as 12 digits (the digits are in bits).
		//then start adjusting from right to left converting 85 to binary and applying OR "|".
		// 85 in binary will be 01 01 01 01.

		GPIOC->OTYPER &= 0x0; //no OR (|) . GPIO port output type register

		GPIOC->OSPEEDR &= 0x0; //GPIO port output speed register to Low speed

		GPIOC->PUPDR &= 0x0; //GPIO port pull-up/pull-down register
		
		//Don’t forget to clear the pending flag for the update interrupt in the status register:
	 
		//TIM1->SR |= 0x1;  //Bit 0 UIF: Update interrupt flag
		
		
		GPIOC->ODR |= 0x100; //Set the orange ON
	 
	 // END: Exersize 3.1 — Using Timer Interrupts:
	 
	
	// START: Exersize 3.2 — Configuring Timer Channels to PWM Mode.
	 
	 //1. Enable the timer 3 peripheral (TIM3) in the RCC.
	  RCC->APB1ENR |= 0x2;  //Bit 1 TIM3EN: TIM3 timer clock enable
	 
	 //2. The timer’s update period determines the period of the PWM signal; 
	 //configure the timer to a UEV period related to 800 Hz (T = 1/f).
	 // T = 0.00125
	 
	 //Configure the timer to trigger an update event:
	 //The default processor frequency of the STM32F072 is 8 MHz
	 //a 16-bit timer can only count up to 65535. 
	 //If your target ARR is outside of that range, 
	 //you’ll need to adjust the prescaler (change units) to scale the ARR appropriately.
	 
	 //PSC = fCLK/(ARR * fTARGET) -1.
	 // fCLK=8MHz; fTARGET = 800Hz. 1/800 =0.00125sec= 1.25 ms.
	 //Set ARR = 1.25 (only 8 bits); PSC = 8MHz/(1.25 * 800Hz) -1 =7999 (occupies 13 bit only out of 16 bit).
		 TIM3->PSC = 1999; //0x1F3F; //79
		 TIM3->ARR = 5;//0xFA;   //800Hz

	// 3. Use the Capture/Compare Mode Register 1 (CCMR1) 
	// register to configure the output channels to PWM mode
	
	//(a) The CCMR1 register configures channels 1 & 2, 
	//and the CCMR2 register for channels 3 & 4. 
	//(b) Examine the bit definitions for the CC1S[1:0] 
	//and CC2S[1:0] bit fields; ensure that you set the channels to output.
	
		 TIM3->CCMR1 &=0xFCFC; // Bits 1:0 CC1S: Capture/Compare 1 selection. Bits 9:8 CC2S: Capture/compare 2 selection
		
	//(c) Examine the bit definitions for the OC1M[2:0] bit field; set output channel 1 to PWM Mode 2.		
	// Bits 6:4 OC1M: Output compare 1 mode 
	// 111: PWM mode 2 - In upcounting, channel 1 is inactive as long as TIMx_CNT<TIMx_CCR1 
	//else active. In downcounting, channel 1 is active as long as TIMx_CNT>TIMx_CCR1 else inactive.
	
	   TIM3->CCMR1 |=0x70;
		 // (d) Use the OC2M[2:0] bit field to set channel 2 to PWM Mode 1.
		 TIM3->CCMR1 = (0x7000 & 0xEFFF);
		 
		 // (e) Enable the output compare preload for both channels.
		 
		  TIM3->CCMR1 |=0x8; // Bit 3 OC1PE: Output compare 1 preload enable
			TIM3->CCMR1 |=0x800; // Bit 11 OC2PE: Output compare 2 preload enable
			
			//4. Set the output enable bits for channels 1 & 2 in the CCER register.
				
			TIM3->CCER |=0x1;   //Bit 0 CC1E: Capture/Compare 1 output enable.
			TIM3->CCER |=0x10;  // Bit 4 CC2E: Capture/Compare 2 output enable.
			
			//5. Set the capture/compare registers (CCRx) for both channels to 20% of your ARR value.
			TIM3->CCR1 =0x32; //ARR=250 x 0.2 = 50.
			TIM3->CCR2 =1; //ARR=5 x 0.2 = 1.
			
			
			// 3.3 — Configuring Pin Alternate Functions
			//1. Look up the alternate functions of the red (PC6) and blue (PC7) LEDs
			// Look at the table in "STM32F072x8 STM32F072xB" datasheet, page 46.
			// AF0=> PC6: TIM3_CH1; PC7: TIM3_CH2
			GPIOC->MODER &= 0xFFFF0FFF; // make PC7 and PC6 zero
			GPIOC->MODER |= 0xA000; //10: Alternate function mode. 
			//GPIOC->MODER |= 0xA000;
			//made moder6 and moder7 "0" first and multiply that 
			//by appropriate binary so we can activatered (PC6)& (PC7).
		
		  //[only red is on right now since "3.3 — Configuring Pin Alternate Functions"]
			
			//TIM2 and TIM3 control register 2 (TIM2_CR2 and TIM3_CR2)
			//0: The TIMx_CH1 pin is connected to TI1 input . 
			//1: The TIMx_CH1, CH2 and CH3 pins are connected to the TI1 input (XOR combination)
			// TI1S is 7th bit.
		//	__IO uint32_t AFR[2]
		
		//From the 
			 GPIOC->AFR[0] |= (0x0 << GPIO_AFRL_AFRL6_Pos) |(0x0 << GPIO_AFRL_AFRL7_Pos); 
  		// GPIOC->AFR[0] |= (0x0 << GPIO_AFRL_AFRL6_Pos) ; 
			// GPIOC->AFR[1] |= (0x0 << GPIO_AFRL_AFRL7_Pos); 
			 
			 
			//GPIOC->AFR[0] |= (0x0 << GPIO_AFRL_AFRL6_Pos) | (0x0 << GPIO_AFRL_AFRL7_Pos); 
			// IN binary:00000000111111111111111111111111 for AF0 and PC6& PC7.
			// GPIOC->AFR[0] &= 0xFFFFFF; // IN binary:00000000111111111111111111111111 for AF0 and PC6& PC7. 
			//Located in  AFRL not in AFRH
			
		//	3.4 — Measuring PWM Output.
			
			
			
			
			
			
			
			
			
			
			
			
			
			
			
			
			
			
			
			
			
			
			
			
			
			
			
			
			
			
	
	
	
	
	
	

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	// int i;
  while (1)
  {
    /* USER CODE END WHILE */
	
    //Toggle the red LED (PC6) with a moderately-slow delay (400-600ms) in the infinite loop
    // GPIOC->BSRR |= (1<<9);
//		
//			
	//		HAL_Delay(250);
//			
     // TIM1_BRK_UP_TRG_COM_IRQHandler(); 	
					
//			for (i = 0; i < 2500000; i++) {
//					// delay of one and half second
//				if(i==2499998) {
//				//
//						ToggleFunction();
//				}
//			}
//		HAL_Delay(350);
//		ToggleFunction(1);
	//	TIM1_BRK_UP_TRG_COM_IRQHandler();

//			
//    if (GPIOC->ODR | (0x40)) // if red is on, red LED (PC6)
//    {
//        GPIOC->ODR &= 0x3BF;  // make it off
//        HAL_Delay(460); //wait 550 ms so that program can catch the off signal
//    }

//    GPIOC->ODR |= (0x40); //make it on back 
//		
	//	HAL_Delay(460);
		
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
