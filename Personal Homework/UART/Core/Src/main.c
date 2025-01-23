/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include<string.h>
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
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

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
	const char *html=	"HTTP/1.1 200 OK \r\n"
										"Content-Type: text/html;charset=utf-8\r\n"
										"Content-Length:2053\r\n\r\n"
										//"<!DOCTYPE html><html><title>WEB SERVER</title><style>#led1{width: 200px;height: 40px;font-size: 21px;color: #FFF;border-radius: 10px; }#led2{width: 200px;height: 40px;font-size: 21px;color: #FFF;border-radius: 10px;}p{font-size: 35px;}td{ padding-right: 20px; padding-bottom: 30px; padding-top:20px}.ledstatus{width: 50px;height: 50px;border-radius: 50%;background-color: #ffffff;border-width: 2px;}.ledstatus1{width: 50px;height: 50px;border-radius: 50%;background-color: #ffffff;border-width: 2px;}</style><body><h1 style=\"font-size: 50px; margin-left: 40%;\" >Control LED</h1><table align=\"center\" ><tr><td ><a href=\"led1\" ><button id=\"led1\" style=\"background-color:#4caf50;\">Toggle LED 1</button></a></td><td> <p>LED1 STATUS: <button class=\"ledstatus\" type=\"button\"></button><img style=\"height: 40px; margin-right:6px\" src=\"data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAADIAAAAyCAQAAAC0NkA6AAAAIGNIUk0AAHomAACAhAAA+gAAAIDoAAB1MAAA6mAAADqYAAAXcJy6UTwAAAACYktHRAD/h4/MvwAAAAlwSFlzAAAOwwAADsMBx2+oZAAAAAd0SU1FB+cJDQsqCFUKkM0AAAQaSURBVFjD7ZdbaFxlEMd/c/Zs0pDNrdA01jbaChoJpGANFaWWmE3qg1oEL2+lSBGEWnzQboigNC9mEx+Ean2pFEQs9EUtFFqziBeMtWmrG0xoHmprkzRsxLTdmG2yl298OJt1k5yT7GJ9UPI/L+d8Z2b+M/PNzPkOrGIV/2lIMcLvYGWV0nQWoWcXQ2JnvVL8RUWyIkl37q4Dg0EeB/3WWvRmeVgFOLKGjWIJYYQ0skN2pHGexGIja1Y2UAhJHV3aqCScJ3H2MYGijXRRdwdIBBlnnL2UlmHwQ4KEjaEMStnLuIyvXDseezKf7w4MkuJjetgmg9rIA3ofyB5GZEib2MJBTekCeXdHlyVxRKYJtLONBqq4ziRQywZucYkLE19uAASzLIl3ddXyMAP8LiiBWlpo4DO+kklJgfq1lid4lsq7ohoTMsg6mjnPpLspn/tyEGp4md3MyqhU8jYVvKVfS5yMopAhrlEZoJ3H+FGUXYTYwvfEI8WQtJOKW/34eU5uaBubeZNRC6GDCBEitCFwg/M8TZ2U8xKn+SAds+lzteZRXUoJTMkxDmiSVg4zYQGh3PuQozrBYVo1yavmGFMlqEfmPSKJ0EcQMH/K88xYn2DyKRyJIIIV063UpM/YQIiIB4lLdYXz4qGCDznBSR9vuCj3koFneEFeYfrv1dASOffqEmwEfOlMNQGukCvRhTAAVwhotW/G2ICSdsuZO8n9uo9yxFzkB5SEm3eOz93OfKkyT+pDKDNylJFCSW7KAKXAVW4jlELYlaYboBThNpdkFpjjpps5FxJFYnoim7dqnaOe4bRHVhXqmeMPvcx3uZUlcCnhEvzZS5A4w7RYtr1g0MzHoWDTwpAVl5xOiaszHgjjIw3QRJj3OAMLJ1OWdBevEWIQ/KQ9ds6jT8IEnaEXoEEuk2IfvzIGwVwnhB3vHuEgn9LPg8yYJARpc+0Vj44X8LOdd9mv5XKcU3TStFBNoYlOTlnHKWc/vWz3/vC7RtKGJeZFDvAzR5jASJR6nuIbZh1Pwxio4RBReZ80CaJsYg8Z3y/qGonHqDfICJ0yaAyAJjnKEXbyeZ7ITgJ8pElAuWb1mtNie80uDxJRcyGXOBQmOUezdVKzrW9ZpplzJuYkQlFDVD2ryJUkRE92s5x66cHAb7QaP3PdjlE/6xi08PF6VkLy5AuM5OCi5AHXqeUeJrAAw92sZwwyrvJL8kJB6IYqutgs007+tJIRDjG90rFu2UgWY5T1t2bC9lZxvkyiydRPFdPDhSkXSnIRRZrZnbdUIl9oYcqFkgjlJGIM5S3F1jJ1Z0kgAZtozg1Z4drU2UJ1i/l16KOMezGAxVWPg8k/JklgszZLMjZ//i4EBf9pPQpg4XOqiwwG+v+FSMB4nChWsYr/N/4COpRbd+1In7YAAAAldEVYdGRhdGU6Y3JlYXRlADIwMjMtMDktMTNUMTE6NDI6MDMrMDA6MDCAlKgPAAAAJXRFWHRkYXRlOm1vZGlmeQAyMDIzLTA5LTEzVDExOjQyOjAzKzAwOjAw8ckQswAAACh0RVh0ZGF0ZTp0aW1lc3RhbXAAMjAyMy0wOS0xM1QxMTo0MjowOCswMDowMKTbZZYAAAAASUVORK5CYII=\" ></p></td></tr><tr><td> <a href=\"led2\" ><button style=\"background-color:#271bcc;\" id=\"led2\">Toggle LED 2</button></a></td><td><p>LED2 STATUS: <button class=\"ledstatus1\" type=\"button\"></button></p></td></tr></table><form style=\"margin-left: 36%;\"  id=\"comForm\"><label for=\"comInput\" style=\"font-size: 25px;\">LCD: </label><input size=45px type=\"text\" id=\"comInput\" name=\"comInput\"> <input type=\"submit\"></form></body></html>"
										//"<!DOCTYPE html><html lang=\"en\"><head><meta charset=\"UTF-8\"><meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\"><title>WEB SERVER</title></head><body><style>h1{margin-right: 45%;margin-left: 45%;margin-top: 100px;margin-bottom: 150px;}#Btn{width: 200px;height: 40px;font-size: 21px;color: #FFF;background-color:#f44336;border-radius: 10px;}</style><h1 align=\"center\"  >Web Server </h1><table align='center'><tr><td><button id=\"Btn\" type=\"button\" onclick=\"changeLed()\">Toggle Led</button></td><td><p style=\"font-size: 21px; margin-left:50px\">LED STATUS <p></p> </p></td></tr></table><script>var count =0;function sendToCom(data) {fetch('/sendToCom', {method: 'POST',body: data }).then(response => {if (response.ok) {console.log('Success');} if(!response.status){sendDelete(response) }}).catch(error => {console.log('ERROR');});} document.getElementById('Btn').addEventListener('click',function(){count++});function changeLed(){const ledStatus = count%2;sendToCom(ledStatus);console.log(ledStatus)}</script></body></html>"
//"<!DOCTYPE html><html><title>WEB SERVER</title><style>#led1{width: 200px;height: 40px;font-size: 21px;color: #FFF;border-radius: 10px; }#led2{width: 200px;height: 40px;font-size: 21px;color: #FFF;border-radius: 10px;}p{font-size: 35px;}td{ padding-right: 20px; padding-bottom: 30px; padding-top:20px}</style><body><h1 style=\"font-size: 50px; margin-left: 40%;\" >Control LED</h1><table align=\"center\" ><tr><td ><a href=\"led1\" ><button id=\"led1\"  style=\"background-color:#4caf50;\" onclick=\"changeLed\">Toggle LED 1</button></a></td><td> <p>LED1 STATUS: <img style=\"height: 40px; border-radius:50%;border-style:dotted\" src=\"data:image/png;base64,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\"></p></td></tr><tr><td> <a href=\"led2\"><button style=\"background-color:#271bcc;\" id=\"led2\">Toggle LED 2</button></a></td><td><p>LED2 STATUS: <img style=\"height: 40px; border-radius:50%;border-style:dotted\" src=\"data:image/png;base64,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\"></p></td></tr></table><form style=\"margin-left: 36%;\"  id=\"comForm\"><label for=\"comInput\" style=\"font-size: 25px\">LCD: </label><input size=45px type=\"text\" id=\"comInput\" name=\"comInput\"></form></body></html>"
	"<!DOCTYPE html>"
"<html>"
    "<head>"
        "<style>"
            "body{display:flex;justify-content:left;align-items:left;flex-direction:column;}"
            "table{width:100%;border-collapse:collapse;}"
            "td{padding:15px;}"
            "button{padding:10px 20px;font-size:20px;background-color:#33bac6;color:white;border:none;cursor:pointer;}"
            ".led-status{font-size:22px;font-weight:bold;}"
            ".led-image{width:100px;margin:5px 0;}"
            "td:nth-child(1){width:20%;}"
            "td:nth-child(2){width:20%;}"
            "#comInput{height: 30px; width: 42%;margin-bottom: 40px; margin-top: 20px;}"
            "div{margin-left: 35%; margin-top: 100px;max-width: 100%;}"
        "</style>"
    "</head>"
    "<body>"
    "<div>"
        "<h1 style=\"margin-left:15%; font-size:40px\">Control LED</h1>"
        "<span class=\"led-status\"><form id=\"comForm\"><label for=\"comInput\">LCD:</label><input type=\"text\" id=\"comInput\" name=\"comInput\"></form></span>"
        "<table>"
            "<tr>"
                "<td><a href=\"led1\"><button>Toggle LED 1</button></a></td>"
                "<td><span class=\"led-status\">LED1 STATUS: ON</span></td>"
                "<td><img class=\"led-image\" src=\"data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAAFoAAABaAQMAAAACZtNBAAAABlBMVEXhtBrhtBp3d0CWAAAAAnRSTlMBoc5ZbwcAAAAJcEhZcwAADsMAAA7DAcdvqGQAAADaSURBVDjL3dI9jsIwEIbhd2QJC8liW5Ai4AiUbtBeJcUeICWluZkr6DiDKxqKpVpRILLFxpmkyPJT4sqPbGfGnwNvP9xDiI7YIjlRlE5Ci5UzesZ/WsWs/lHY00UxZ56nEjyWskGqMNvULG0Sss9V1xG+86FxgK8ME6BqAazamoDPGAHT+5h08HEAiusfFkdgdnvua70Oer31upZAjgAkollPS2SXsamwNuSoPFaaqEwsmHBuVnA4lnqhQoMfaWv/w3dergtTnwdAHQd+N6nTAF7Z1q8zWPSNxi+SbTVQMY1DqQAAAABJRU5ErkJggg==\"></td>"
            "</tr>"
            "<tr>"
                "<td><a href=\"led2\"><button>Toggle LED 2</button></a></td>"
                "<td><span class=\"led-status\">LED2 STATUS: ON</span></td>"
                "<td><img class=\"led-image\" src=\"data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAAFoAAABaAQMAAAACZtNBAAAABlBMVEXhtBrhtBp3d0CWAAAAAnRSTlMBoc5ZbwcAAAAJcEhZcwAADsMAAA7DAcdvqGQAAADaSURBVDjL3dI9jsIwEIbhd2QJC8liW5Ai4AiUbtBeJcUeICWluZkr6DiDKxqKpVpRILLFxpmkyPJT4sqPbGfGnwNvP9xDiI7YIjlRlE5Ci5UzesZ/WsWs/lHY00UxZ56nEjyWskGqMNvULG0Sss9V1xG+86FxgK8ME6BqAazamoDPGAHT+5h08HEAiusfFkdgdnvua70Oer31upZAjgAkollPS2SXsamwNuSoPFaaqEwsmHBuVnA4lnqhQoMfaWv/w3dergtTnwdAHQd+N6nTAF7Z1q8zWPSNxi+SbTVQMY1DqQAAAABJRU5ErkJggg==\"></td>"
            "</tr>"
        "</table>"
    "</div>"
    "</body>"
"</html>"	;								//"<script>var count =0; function sendToCom(data) {fetch('/sendToCom', {method: 'POST',body: data }).then(response => {if (response.ok) {console.log('Success');} else {console.log('ERROR');}}).catch(error => {console.log('ERROR');});} document.getElementById('Btn').addEventListener('click',function(){count++});function changeLed(){const ledStatus = count%2;sendToCom(ledStatus);console.log(ledStatus)}</script>"

	//const char string_request[] ="CONNECT 192.168.2.197:4000 HTTP/1.1";
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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */
	HAL_UART_Transmit(&huart1,(uint8_t *)html,strlen(html),HAL_MAX_DELAY);

		
		
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
  while (1)
  {
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
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
