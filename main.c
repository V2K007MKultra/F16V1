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
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "MS5611.h"
#include "fonts.h"
#include "ssd1306.h"
#include "test.h"
#include "bitmap.h"
#include "horse_anim.h"
#include "mpu6050.h"
#include <stdbool.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define HMC5883L_ADDRESS    0x1E

#define HMC5883L_REG_CONFIGA    0x00
#define HMC5883L_REG_MODE       0x02
#define HMC5883L_REG_DATA       0x03

#define HMC5883L_MODEREG_BIT        1
#define HMC5883L_MODEREG_LENGTH     2
#define PI                  3.14159265358979323846
#define RAD_TO_DEG          (180.0 / PI)
float altbaro = 0.0;
int	Angle3;
int Angle1;

float alt_arr[10] = {0};
int alt_cnt = 0;
/* USER CODE END Includes */
int _write(int file, char *ptr, int len)
{
	HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, HAL_MAX_DELAY);
	return len;
}
MS5611_t MS5611;
MPU6050_t MPU6050;
/* USER CODE END Includes */
uint8_t indata[1];
uint8_t SBUS_in[25];
int num=0;
uint16_t ch[16];
void SBUS_intoCH(void){
ch[0]  = ((SBUS_in[1]|SBUS_in[2]<<8)                      & 0x07FF);
ch[1]  = ((SBUS_in[2]>>3 |SBUS_in[3]<<5)                 & 0x07FF);
ch[2]  = ((SBUS_in[3]>>6 |SBUS_in[4]<<2 |SBUS_in[5]<<10)  & 0x07FF);
ch[3]  = ((SBUS_in[5]>>1 |SBUS_in[6]<<7)                 & 0x07FF);
ch[4]  = ((SBUS_in[6]>>4 |SBUS_in[7]<<4)                 & 0x07FF);
ch[5]  = ((SBUS_in[7]>>7 |SBUS_in[8]<<1 |SBUS_in[9]<<9)   & 0x07FF);
ch[6]  = ((SBUS_in[9]>>2 |SBUS_in[10]<<6)                & 0x07FF);
ch[7]  = ((SBUS_in[10]>>5|SBUS_in[11]<<3)                & 0x07FF);
ch[8]  = ((SBUS_in[12]   |SBUS_in[13]<<8)                & 0x07FF);
ch[9]  = ((SBUS_in[13]>>3|SBUS_in[14]<<5)                & 0x07FF);
ch[10] = ((SBUS_in[14]>>6|SBUS_in[15]<<2|SBUS_in[16]<<10) & 0x07FF);
ch[11] = ((SBUS_in[16]>>1|SBUS_in[17]<<7)                & 0x07FF);
ch[12] = ((SBUS_in[17]>>4|SBUS_in[18]<<4)                & 0x07FF);
ch[13] = ((SBUS_in[18]>>7|SBUS_in[19]<<1|SBUS_in[20]<<9)  & 0x07FF);
ch[14] = ((SBUS_in[20]>>2|SBUS_in[21]<<6)                & 0x07FF);
ch[15] = ((SBUS_in[21]>>5|SBUS_in[22]<<3)                & 0x07FF);
}
/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
void HMC5883L_Init(void)
{




    uint8_t data[2];

    data[0] = 0x02;     // 0x0C in your example
    data[1] = 0x00;    // MSB byte of 16bit data
//    HAL_I2C_Mem_Write(&hi2c1, HMC5883L_ADDRESS<<1, HMC5883L_REG_MODE, I2C_MEMADD_SIZE_8BIT, &config_data, 1, 1000);
//
     HAL_I2C_Master_Transmit(&hi2c1, 0x1E<<1, data, 2, HAL_MAX_DELAY); // SET DEVICE MODE - continuous mode


//     HAL_I2C_Master_Transmit(&hi2c1, kk<<1, 0x00, 1, 5000); //SET START RANGE
}

//read the data from sensor
void HMC5883L_ReadData(int16_t* x, int16_t* y, int16_t* z)
{
    uint8_t buffer[6];

    HAL_I2C_Mem_Read(&hi2c1, HMC5883L_ADDRESS<<1, HMC5883L_REG_DATA, I2C_MEMADD_SIZE_8BIT, buffer, 6, HAL_MAX_DELAY);

    *x = (((int16_t)buffer[0]) << 8) | buffer[1];
    *y = (((int16_t)buffer[4]) << 8) | buffer[5];
    *z = (((int16_t)buffer[2]) << 8) | buffer[3];
}

//send the sensor data using serial port.
float HMC5883L_CalculateHeading(int16_t mx, int16_t my) {
    float heading = atan2(my, mx);
    if (heading < 0) {
        heading += 2 * PI;
    }
    return heading * RAD_TO_DEG;
}
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
void float_to_string(float value,char* buffer,int buffer_size);
float map(float x,float in_min,float in_max,float out_min,float out_max);
int servo_polarity(int Angle1);
int constrainvalue(int value);
/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define mid1 150
#define mid2 80
/* USER CODE END 0 */
#define ANGULAR_KPX 0.2
#define ANGULAR_KIX 0.05
#define ANGULAR_KDX 0.01

#define ANGULAR_RATE_KPX 0.004
#define ANGULAR_RATE_KIX 0.004
#define ANGULAR_RATE_KDX 0.003
#define ANGULAR_KPY 0.2
#define ANGULAR_KIY 0.05
#define ANGULAR_KDY 0.01

#define ANGULAR_RATE_KPY 0.004
#define ANGULAR_RATE_KIY 0.004
#define ANGULAR_RATE_KDY 0.004
float target_angleX = 0.0; // Target angle for stabilization
float target_angleY = 0.0;
float current_angleX ; // Current angle read from sensor
float current_angleY ;
float current_rateX ; // Current angular rate read from sensor
float current_rateY ;
float angular_error_integralX = 0.0; // Integral of angular error for PID control
float angular_rate_error_integralX= 0.0; // Integral of angular rate error for PID control
float last_angular_errorX = 0.0; // Previous angular error for derivative term
float last_angular_rate_errorX = 0.0; // Previous angular rate error for derivative term
float angular_error_integralY= 0.0; // Integral of angular error for PID control
float angular_rate_error_integralY = 0.0; // Integral of angular rate error for PID control
float last_angular_errorY= 0.0; // Previous angular error for derivative term
float last_angular_rate_errorY = 0.0; // Previous angular rate error for derivative term
bool modeselect();
bool parachuteselect();
float angular_pid_controlX(float angleX);
float angular_pid_controlY(float angleY);
float angular_rate_pid_controlX(float rateX);
float angular_rate_pid_controlY(float rateY);
float ch1pid;
float ch2pid;
int ch1pwm;
int ch1pwm2;
int ch2pwm;
#define pid_dtX 0.01;
#define pid_dtY 0.01;
void motorinit();
void motor();
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
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM6_Init();
  MX_UART4_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
    HAL_Delay(1000);

    HMC5883L_Init();
    HAL_Delay(2000);

    MPU6050_Init(&hi2c1);
      /// BAROMETER
   	MS5611_Reset(&hi2c1, &MS5611);
   	MS5611_ReadProm(&hi2c1, &MS5611);
     short int x, y, z;
     float heading;
     char buffers[50];
     char bufferx[50];
  /* USER CODE END 2 */
     HAL_Delay(1000);
     SSD1306_Init();  // initialise
     HAL_Delay (2000);
     SSD1306_Clear();
     /// lets print some string

      HAL_UART_Receive_IT(&huart1,indata, 1);
      HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
      HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
      HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
      HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
 	 __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1 ,150);
     __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2, 150);
      motorinit();
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	               motor();
	               MPU6050_Read_All(&hi2c1, &MPU6050);
	                printf("%f\r\n",  MPU6050.KalmanAngleY);
		            HMC5883L_ReadData(&x, &y, &z);
		            HMC5883L_CalculateHeading(x,y);
		            heading = HMC5883L_CalculateHeading(x, y);
		            printf("Heading: %.2f degrees\n", heading);
		             float_to_string(MPU6050.KalmanAngleY,buffers,sizeof(buffers));

		             SSD1306_GotoXY (10, 30);
		             SSD1306_Puts(buffers,&Font_11x18, 1);
		             SSD1306_UpdateScreen(); //display
		             if( parachuteselect())
		           				  {
		           					  htim2.Instance->CCR2=mid2;
		           				  }
		           	 if( !parachuteselect())
		           						  {
		           					 htim2.Instance->CCR2=mid1;
		           						  }
		             if(  ch[0]>1553)
		                     {
		            	 ch[0]=1553;
		                                                   }
		             else if(  ch[0]<446)
		                      {
		            	 ch[0]=446;
		                                                     }
		             if(  ch[1]>1552)
		           		                     {
		            	 ch[1]=1552;
		           		                                                   }
		           		             else if(   ch[1]<435)
		           		                      {
		           		            	 ch[1]=435;
		           		                                                     }
		             if( modeselect())
		        {
		             int	 pwmpulse3= (map(ch[0],446,1553,-45,45));
		             int   pwmpulse1= map(ch[1],435,1552,-45,45);
		             int	 pwpulse2= pwmpulse1- pwmpulse3;
		             int pwpulse5=pwmpulse1+pwmpulse3;
		             int pulse1=map( constrainvalue(pwpulse2),-45,45,80,220);
		             int  pulse3=map( constrainvalue(pwpulse5),-45,45,80,220);
		             int pulse4=servo_polarity(pulse1);
		           	 __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1 ,pulse3);
		           	 __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2, pulse4);
		        }
		             if(!modeselect())
		         {
		             current_angleX= MPU6050.KalmanAngleX;
		             current_angleY= MPU6050.KalmanAngleY;
		             current_rateX=MPU6050.Gx;
		             current_rateY=MPU6050.Gy;
		             float rate_outputX = angular_rate_pid_controlX(current_rateX);
                     float rate_outputY = angular_rate_pid_controlY(current_rateY);
                     float ch1pids= angular_pid_controlX(current_angleX);
                    int ch1pid=rate_outputX-rate_outputY;
                      if(   ch1pid>20)
                      {
                    	 ch1pid=20;
                      }
                      else if(  ch1pid<-20)
                      {
                    	  ch1pid=-20;
                      }
                   int ch2pid=rate_outputY+rate_outputX;
                         if(   ch2pid>20)
                                          {
                                        	 ch2pid=20;
                                          }
                                          else if(  ch2pid<-20)
                                          {
                                        	  ch2pid=-20;
                                          }
                     int ch1pwm=map( ch1pid,-20,20,80,220);
                     int ch2pwm=map( ch2pid,-20,20,80,220);
                 	 int ch2pwm2=servo_polarity(ch2pwm);/*reverse servo ccr to polarity angle*/
                 	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1, ch1pwm);
                    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2, ch2pwm);

                    float_to_string(rate_outputX,bufferx,sizeof(bufferx));
                  		             SSD1306_GotoXY (0,0);
                  		             SSD1306_Puts (bufferx, &Font_11x18, 1);
                  		             SSD1306_UpdateScreen(); //display
		         }
		         if(alt_cnt == 0)
		         			{
		         				MS5611_RequestTemperature(&hi2c1,OSR_4096);
		         				alt_cnt = 1;
		         			}

		         			else if (alt_cnt == 1)
		         			{
		         				MS5611_ReadTemperature(&hi2c1,&MS5611);
		         				MS5611_CalculateTemperature(&MS5611);
		         				MS5611_RequestPressure(&hi2c1, OSR_4096);
		         				alt_cnt = 2;
		         			}
		         			else
		         			{
		         				MS5611_ReadPressure(&hi2c1,&MS5611);
		         				MS5611_CalculatePressure(&MS5611);
		         				MS5611_RequestTemperature(&hi2c1,OSR_4096);
		         				MS5611.Alt = (MS5611_getAltitude1((float)MS5611.P/100.f))*100;

		         				#define X 0.90f
		         				MS5611.Alt_Filt = MS5611.Alt_Filt * X + MS5611.Alt * (1.0f-X);
		         				alt_cnt = 1;

		         		printf("%.1f \t %.1f \n", MS5611.Alt, MS5611.Alt_Filt);
		         			}
		  printf("hello\r\n");
		  /* HAL_GPIO_WritePin(GPIOD,  GPIO_PIN_11,GPIO_PIN_SET);
		  HAL_Delay(40);
		  HAL_GPIO_WritePin(GPIOD,  GPIO_PIN_11,GPIO_PIN_RESET);
		  HAL_Delay(40);*/
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
// PID control function for angle stabilization
void motor()
{

int  power=map(ch[2],195,1788,100,200);
	 __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,power);

}
void motorinit()
{



	 __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,200);
	 HAL_Delay(4000);
	 __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,100);
	 HAL_Delay(4000);


}
bool modeselect(){
	if (ch[5]<200)
	{
		return true;
	}
	else if( ch[5]>1700)
	{
	return false;
	}
	return false;
}
bool parachuteselect(){
	if (ch[4]<1500)
	{
		return true;
	}
	else if( ch[4]>1500)
	{
	return false;
	}
	return false;
}
float angular_pid_controlX(float angleX) {

    float errorX = target_angleX - angleX;

    /*   angular_error_integralX += errorX*pid_dtX;
    if(  angular_error_integralX>30)
    {
    	angular_error_integralX=30;
    }
    else if( angular_error_integralX<-30)
    {
    	 angular_error_integralX=-30;
    } */

    float derivativeX = (errorX - last_angular_errorX)/pid_dtX;


    float pid_outputX = ANGULAR_KPX * errorX + ANGULAR_KDX * derivativeX;
     if( pid_outputX >15)
      {
  	   pid_outputX =15;
      }
      if( pid_outputX <-15)
      {
      	 pid_outputX =-15;
      }
      last_angular_errorX = errorX;
    return pid_outputX;
}

// PID control function for angular rate stabilization
float angular_rate_pid_controlX(float rateX) {
    float error_rateX = angular_pid_controlX(current_angleX) - rateX;
    angular_rate_error_integralX += error_rateX*pid_dtX;
    float derivative_rateX = (error_rateX - last_angular_rate_errorX)/pid_dtX;


    float pid_output_rateX = ANGULAR_RATE_KPX * error_rateX + ANGULAR_RATE_KIX * angular_rate_error_integralX + ANGULAR_RATE_KDX * derivative_rateX;
        if( pid_output_rateX >25)
         {
    	pid_output_rateX=25;
         }
         if( pid_output_rateX <-25)
         {
        	 pid_output_rateX=-25;
         }
    last_angular_rate_errorX = error_rateX;
    return pid_output_rateX;
}
float angular_pid_controlY(float angleY) {

    float errorY = target_angleY - angleY;

    /*   angular_error_integralY += errorY*pid_dtY;
    if(  angular_error_integralY>30)
      {
      	angular_error_integralY=30;
      }
      else if( angular_error_integralY<-30)
      {
      	 angular_error_integralY=-30;
      } */

    float derivativeY =(errorY - last_angular_errorY)/pid_dtY;


    float pid_outputY = ANGULAR_KPY * errorY + ANGULAR_KDY * derivativeY;
    if( pid_outputY >15)
      {
  	   pid_outputY =15;
      }
      if( pid_outputY <-15)
      {
      	 pid_outputY =-15;
      }
      last_angular_errorY = errorY;
    return pid_outputY;
}

// PID control function for angular rate stabilization
float angular_rate_pid_controlY(float rateY) {
    float error_rateY = angular_pid_controlY(current_angleY) - rateY;
    angular_rate_error_integralY += error_rateY*pid_dtY;
    float derivative_rateY = (error_rateY - last_angular_rate_errorY)/pid_dtY;


    float pid_output_rateY = ANGULAR_RATE_KPY * error_rateY + ANGULAR_RATE_KIY * angular_rate_error_integralY + ANGULAR_RATE_KDY * derivative_rateY;
    if(  pid_output_rateY >25)
         {
    	 pid_output_rateY  =25;
         }
         if(  pid_output_rateY  <-25)
         {
        	 pid_output_rateY =-25;
         }
         last_angular_rate_errorY = error_rateY;
    return pid_output_rateY;

}
float map(float x,float in_min,float in_max,float out_min,float out_max)
{
	return (x-in_min)*(out_max-out_min)/(in_max-in_min)+out_min;
}
int constrainvalue(int value)
{
	if(value<-45)
	{
		return -45;
	}
	else if(value>45)
	{
		return 45;
	}
	else
	{
		return value;
}
}
int servo_polarity(int Angle1)
{
	if(Angle1>150)
		 	 	 	{

		 	 	Angle3=(220-	Angle1)+80;

		 	 	 	}
	else	if(Angle1<150)
		 	 		{
		Angle3=220-(Angle1-80);

		 	 		}
	else	if(Angle1==150)
		 	 		{
	Angle3=150;

		 	 		}
	return 	Angle3;

}
void float_to_string(float value,char* buffer,int buffer_size)
{

	snprintf(buffer,buffer_size,"%f",value);
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
   {
   	if(num!=0||indata[0]==0x0f){
   		if(indata[0]==0x0f)num=0;
   		SBUS_in[num++]=indata[0];
   		if(num==25){num=0;SBUS_intoCH();}
   	}
   	HAL_UART_Receive_IT(&huart1,indata, 1);
   }
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
