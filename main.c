/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  * 功能：初始化STM32微控制器和所有外设，然后进入无限循环执行PID闭环控制
  * 控制流程：遥控器输入 → 目标速度映射 → PID计算 → CAN发送控制电流 → 电机响应 → 编码器反馈 → 闭环控制
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_can.h"
#include "CAN_receive.h"
#include "pid.h"
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
RC_Ctl_t RC_Ctl;   						// 遥控器数据结构体，存储解析后的遥控器数据
uint8_t sbus_rx_buffer[18]; 		// SBUS遥控器数据接收缓冲区，18字节对应SBUS协议数据包
pid_type_def motor_pid1;				// 电机1 PID控制器结构体
pid_type_def motor_pid2;				// 电机2 PID控制器结构体
pid_type_def motor_pid3;				// 电机3 PID控制器结构体
pid_type_def motor_pid4;				// 电机4 PID控制器结构体
const motor_measure_t *motor_data1;	// 电机1数据指针，指向ID为1的3508电机实时数据
const motor_measure_t *motor_data2;	// 电机2数据指针，指向ID为2的3508电机实时数据
const motor_measure_t *motor_data3;	// 电机3数据指针，指向ID为3的3508电机实时数据
const motor_measure_t *motor_data4;	// 电机4数据指针，指向ID为4的3508电机实时数据
int set_speed = 0;						// 目标速度设定值，通过遥控器通道0映射得到

const fp32 PID[3]={5,0.01,0};	// PID参数初始化数组：比例系数P=5，积分系数I=0.01，微分系数D=0
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
 * @brief 线性映射函数
 * @param x: 输入值，需要映射的原始值
 * @param in_min: 输入范围的最小值
 * @param in_max: 输入范围的最大值
 * @param out_min: 输出范围的最小值
 * @param out_max: 输出范围的最大值
 * @return int: 映射后的输出值
 * 
 * 功能：将输入值x从输入范围[in_min, in_max]线性映射到输出范围[out_min, out_max]
 * 数学公式：output = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
 * 
 * 应用场景：
 * - 将遥控器通道值（364-1684）映射为电机转速设定值（-2000到2000 RPM）
 * - 实现输入到输出的比例缩放-
 * 
 * 示例：
 * map(1024, 364, 1684, -2000, 2000) 将遥控器中位值1024映射为转速0 RPM
 * map(364, 364, 1684, -2000, 2000) 将遥控器最小值364映射为转速-2000 RPM
 * map(1684, 364, 1684, -2000, 2000) 将遥控器最大值1684映射为转速2000 RPM
 */
int map(int x, int in_min, int in_max, int out_min, int out_max) {
	// 线性映射计算：先计算输入值在输入范围内的相对位置，然后按比例映射到输出范围
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
/**
 * @brief 主函数 - 程序入口点
 * 
 * 功能：初始化STM32微控制器和所有外设，然后进入无限循环执行PID闭环控制
 * 控制流程：遥控器输入 → 目标速度映射 → PID计算 → CAN发送控制电流 → 电机响应 → 编码器反馈 → 闭环控制
 * 
 * @retval int 程序退出码（实际不会退出，因为程序在无限循环中运行）
 */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();  // 初始化HAL库，配置系统时钟和SysTick定时器

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();  // 配置系统时钟，HSE=8MHz，PLL倍频到168MHz

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  // 初始化所有配置的外设：GPIO、DMA、定时器、串口、CAN等
  MX_GPIO_Init();                // 初始化GPIO引脚
  MX_DMA_Init();                 // 初始化DMA控制器
  MX_TIM8_Init();                // 初始化定时器8
  MX_TIM1_Init();                // 初始化定时器1
  MX_USART3_UART_Init();         // 初始化USART3串口（用于SBUS遥控器通信）
  MX_CAN1_Init();                // 初始化CAN1总线
  MX_CAN2_Init();                // 初始化CAN2总线
  
  /* USER CODE BEGIN 2 */
    // 应用层初始化：CAN过滤器配置、遥控器接收、PID控制器初始化
    can_filter_init();                                  // 初始化CAN过滤器，配置接收ID
		HAL_UART_Receive_DMA(&huart3,sbus_rx_buffer,18);      // 启动DMA接收SBUS遥控器数据（18字节数据包）
		
		// 初始化四个电机的PID控制器：位置式模式，参数P=5/I=0.01/D=0，输出限制±16000，积分限制±2000
		PID_init(&motor_pid1,PID_POSITION,PID,16000,2000);   // 电机1 PID控制器
		PID_init(&motor_pid2,PID_POSITION,PID,16000,2000);   // 电机2 PID控制器
		PID_init(&motor_pid3,PID_POSITION,PID,16000,2000);   // 电机3 PID控制器
		PID_init(&motor_pid4,PID_POSITION,PID,16000,2000);   // 电机4 PID控制器
		
		// 获取四个底盘电机数据指针
		motor_data1 = get_chassis_motor_measure_point(0);  // 电机1数据指针
		motor_data2 = get_chassis_motor_measure_point(1);  // 电机2数据指针
		motor_data3 = get_chassis_motor_measure_point(2);  // 电机3数据指针
		motor_data4 = get_chassis_motor_measure_point(3);  // 电机4数据指针 		
	/* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		/* 闭环控制流程开始 */
		
		/* 1. 遥控器输入处理：将遥控器通道0的原始值映射为目标转速 */
		set_speed=map(RC_Ctl.rc.ch0,364,1684,-2000,2000);					// 将遥控器通道值映射为转速单位rpm，确保中位时转速为0
		
		/* 2. PID控制器计算：比较目标速度和实际速度，计算四个电机的控制量 */
		PID_calc(&motor_pid1,motor_data1->speed_rpm,set_speed);			// 电机1 PID计算
		PID_calc(&motor_pid2,motor_data2->speed_rpm,set_speed);			// 电机2 PID计算
		PID_calc(&motor_pid3,motor_data3->speed_rpm,set_speed);			// 电机3 PID计算
		PID_calc(&motor_pid4,motor_data4->speed_rpm,set_speed);			// 电机4 PID计算
		
		/* 3. 执行控制输出：将四个电机的PID计算结果发送给电机驱动器 */
		CAN_cmd_chassis(motor_pid1.out,motor_pid2.out,motor_pid3.out,motor_pid4.out);	// 发送四个电机的控制电流
		
		/* 4. 控制周期延时：维持2ms的控制周期，确保系统稳定性 */
		HAL_Delay(2);														// 同步PID计算与循环时间
		
		/* 闭环控制流程结束，进入下一轮计算 */
		
		/* 
		 * 完整的闭环控制流程（四个电机独立控制）：
		 * 遥控器输入 → 目标速度映射 → 四个电机PID计算 → CAN发送四个电机控制电流 → 电机响应 → 
		 * 编码器反馈 → CAN接收四个电机反馈数据 → 获取四个电机实际速度 → 下一轮PID计算
		 * 
		 * 关键函数调用顺序：
		 * 1. HAL_UART_RxCpltCallback() - 接收遥控器SBUS数据
		 * 2. map() - 将遥控器值映射为目标转速
		 * 3. get_chassis_motor_measure_point() - 获取四个电机反馈数据
		 * 4. PID_calc() - 四个电机PID控制器计算
		 * 5. CAN_cmd_chassis() - 发送四个电机控制电流
		 * 6. HAL_CAN_RxFifo0MsgPendingCallback() - 接收四个电机反馈数据(中断)
		 */
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/**
 * @brief UART接收完成中断回调函数
 * @param UartHandle: UART句柄指针
 * 
 * 功能：解析SBUS遥控器协议数据，将18字节的SBUS数据包解析为遥控器通道值、鼠标数据和键盘数据
 * SBUS协议特点：
 * - 18字节数据包包含16个通道数据，每个通道11位，范围0-2047（实际有效范围364-1684）
 * - 数据采用小端模式存储
 * - 使用位操作提取各个通道数据
 * 
 * 数据包结构：
 * 字节0-1: 通道0数据（11位）
 * 字节1-2: 通道1数据（11位）
 * 字节2-4: 通道2数据（11位）
 * 字节4-5: 通道3数据（11位）
 * 字节5: 开关状态（s1和s2）
 * 字节6-13: 鼠标数据（X、Y、Z移动和按键状态）
 * 字节14-15: 键盘按键状态（位编码）
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle) 
{  
		// 解析遥控器通道数据，每个通道11位数据，范围0-2047（实际有效范围364-1684）
		
		// 通道0：摇杆水平方向（左摇杆左右移动）
		// 操作：sbus_rx_buffer[0]的低8位 | sbus_rx_buffer[1]的高3位（左移8位），然后与0x07FF进行与操作取11位
		RC_Ctl.rc.ch0 = (sbus_rx_buffer[0]| (sbus_rx_buffer[1] << 8)) & 0x07ff;          
		
		// 通道1：摇杆垂直方向（左摇杆上下移动）
		// 操作：sbus_rx_buffer[1]的低5位（右移3位） | sbus_rx_buffer[2]的高6位（左移5位），取11位
		RC_Ctl.rc.ch1 = ((sbus_rx_buffer[1] >> 3) | (sbus_rx_buffer[2] << 5)) & 0x07ff;       
		
		// 通道2：右摇杆水平方向
		// 操作：sbus_rx_buffer[2]的低2位（右移6位） | sbus_rx_buffer[3]的中间8位（左移2位） | sbus_rx_buffer[4]的高1位（左移10位）
		RC_Ctl.rc.ch2 = ((sbus_rx_buffer[2] >> 6) | (sbus_rx_buffer[3] << 2) | (sbus_rx_buffer[4] << 10)) & 0x07ff;          
		
		// 通道3：右摇杆垂直方向
		// 操作：sbus_rx_buffer[4]的低7位（右移1位） | sbus_rx_buffer[5]的高4位（左移7位）
		RC_Ctl.rc.ch3 = ((sbus_rx_buffer[4] >> 1) | (sbus_rx_buffer[5] << 7)) & 0x07ff;           
		
		// 开关1状态：取sbus_rx_buffer[5]的第4-5位（右移4位后与0x000C，再右移2位）
		RC_Ctl.rc.s1  = ((sbus_rx_buffer[5] >> 4)& 0x000C) >> 2;                           
		
		// 开关2状态：取sbus_rx_buffer[5]的第4-5位（右移4位后与0x0003）
		RC_Ctl.rc.s2  = ((sbus_rx_buffer[5] >> 4)& 0x0003);                               

		// 解析鼠标数据
		RC_Ctl.mouse.x = sbus_rx_buffer[6] | (sbus_rx_buffer[7] << 8);                    // 鼠标X轴移动量（16位有符号数）
		RC_Ctl.mouse.y = sbus_rx_buffer[8] | (sbus_rx_buffer[9] << 8);                    // 鼠标Y轴移动量（16位有符号数）
		RC_Ctl.mouse.z = sbus_rx_buffer[10] | (sbus_rx_buffer[11] << 8);                  // 鼠标滚轮移动量（16位有符号数）
		RC_Ctl.mouse.press_l = sbus_rx_buffer[12];                                        // 鼠标左键按下状态（0/1）
		RC_Ctl.mouse.press_r = sbus_rx_buffer[13];                                        // 鼠标右键按下状态（0/1）
		
		// 解析键盘数据：16位位编码，每个位对应一个按键状态
		RC_Ctl.key.v = sbus_rx_buffer[14] | (sbus_rx_buffer[15] << 8);    			// 键盘按键状态（位编码）
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