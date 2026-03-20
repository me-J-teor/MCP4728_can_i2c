/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Elevator CAN to MCP4728 DAC Bridge
  ******************************************************************************
  
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "can.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define DEBUG_UART_ENABLE      0U
#define DEBUG_CAN_RX_VERBOSE   0U
#define DEBUG_I2C_VERBOSE      0U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

// MCP4728 默认 7-bit 地址为 0x60，HAL 发送时使用左移后的地址格式
#define MCP4728_ADDR (0x60U << 1)

// --- 4~20mA 映射用常数 ---
// 假设硬件把 DAC (0~4095) 的输出电压(0~Vref)转换为 0~20mA
// 所以 4mA 对应的 DAC 值为: (4mA/20mA) * 4095 ≈ 819，20mA对应 4095
// (*如果您的硬件本身是将 0~Vref 转化为 4~20mA, 则此处应改为 0 和 4095)
#define DAC_4MA_VAL  0
#define DAC_20MA_VAL 4095

// 定义 CAN 接收到数据的对应量程
// 假设收到的数据是 0 到 10000 对应 4~20mA。(可根据您的电梯协议实际极值随意更改)
#define SENSOR_MIN 0
#define SENSOR_MAX 10000

// MCP4728 LDAC 引脚：高电平保持输出不更新，低脉冲后同步更新各通道
#define MCP4728_LDAC_GPIO_Port GPIOA
#define MCP4728_LDAC_Pin GPIO_PIN_8
// -----------------------

static volatile uint8_t dac_update_pending = 0;
static volatile uint16_t dac_chA_pending = 0;
static volatile uint16_t dac_chB_pending = 0;
static volatile uint16_t dac_chC_pending = 0;

/* CAN 正常模式下不再发送环回测试帧，先保留计数变量定义 */
/*
static volatile uint32_t g_can_tx_ok = 0U;
static volatile uint32_t g_can_tx_fail = 0U;
*/
static volatile uint32_t g_can_rx_ok = 0U;
static volatile uint32_t g_i2c_tx_ok = 0U;
static volatile uint32_t g_i2c_tx_fail = 0U;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void MCP4728_Write_3Channels(uint16_t chA, uint16_t chB, uint16_t chC);
/* CAN 正常模式下不再发送环回测试帧 */
/* static void CAN_Send_Loopback_TestFrame(void); */
/* 串口调试函数保留注释，按需恢复 */
/* static void Debug_Printf(const char *fmt, ...); */
/* static void Debug_PrintCanPayload(const CAN_RxHeaderTypeDef *rxHeader, const uint8_t *rxData); */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*
static void Debug_Printf(const char *fmt, ...)
{
#if DEBUG_UART_ENABLE
  char buf[160];
  va_list args;

  va_start(args, fmt);
  int len = vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);

  if (len <= 0) {
    return;
  }

  if (len > (int)sizeof(buf)) {
    len = (int)sizeof(buf);
  }

  (void)HAL_UART_Transmit(&huart2, (uint8_t *)buf, (uint16_t)len, 50U);
#else
  (void)fmt;
#endif
}

static void Debug_PrintCanPayload(const CAN_RxHeaderTypeDef *rxHeader, const uint8_t *rxData)
{
#if DEBUG_UART_ENABLE && DEBUG_CAN_RX_VERBOSE
  Debug_Printf("[CAN RX] ID=0x%03lX DLC=%lu DATA=%02X %02X %02X %02X %02X %02X %02X %02X\r\n",
               rxHeader->StdId,
               rxHeader->DLC,
               rxData[0], rxData[1], rxData[2], rxData[3],
               rxData[4], rxData[5], rxData[6], rxData[7]);
#else
  (void)rxHeader;
  (void)rxData;
#endif
}
*/

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
  MX_CAN1_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  // 1. 配置 CAN 过滤器
  CAN_FilterTypeDef sFilterConfig;
  sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = (0x181 << 5);      // 帧 ID: 0x181
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0xFFE0;        // 精确匹配
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.SlaveStartFilterBank = 14;

  if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK) {
      Error_Handler();
  }

  // 2. 启动 CAN
  if (HAL_CAN_Start(&hcan1) != HAL_OK) {
      Error_Handler();
  }

  // 3. 开启接收中断
  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
      Error_Handler();
  }

  /*
   * Normal mode 下关闭 I2C 扫描自检，不影响 CAN -> DAC 主流程。
   * 如需排查硬件连线/地址问题，可临时恢复此段。
   * Debug_Printf("[I2C SCAN] Scanning 0x01~0x7F...\r\n");
   * for (uint8_t addr = 0x01; addr <= 0x7F; addr++) {
   *   HAL_StatusTypeDef ret = HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(addr << 1), 2, 10);
   *   if (ret == HAL_OK) {
   *     Debug_Printf("[I2C SCAN] addr=0x%02X -> ACK <<<\r\n", addr);
   *   }
   * }
   * Debug_Printf("[I2C SCAN] Done\r\n");
   */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* static uint32_t lastDebugTick = 0U; */

    /* CAN 正常模式：关闭环回测试发包 */
    /* CAN_Send_Loopback_TestFrame(); */

    if (dac_update_pending != 0U) {
      uint16_t chA = dac_chA_pending;
      uint16_t chB = dac_chB_pending;
      uint16_t chC = dac_chC_pending;

      dac_update_pending = 0U;
      MCP4728_Write_3Channels(chA, chB, chC);
    }

    /*if ((HAL_GetTick() - lastDebugTick) >= 1000U) {
      lastDebugTick = HAL_GetTick();
      Debug_Printf("[STAT] CAN_TX ok=%lu fail=%lu | CAN_RX=%lu | I2C ok=%lu fail=%lu\r\n",
                   g_can_tx_ok,
                   g_can_tx_fail,
                   g_can_rx_ok,
                   g_i2c_tx_ok,
                   g_i2c_tx_fail);
    } */
    
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

/* USER CODE BEGIN 4 */

/**
  * @brief 环回调试发包：周期发送标准帧 0x181，数据格式为 A(4B)+B(2B)+C(2B)
  */
/*
static void CAN_Send_Loopback_TestFrame(void) {
  static uint32_t lastTickMs = 0U;

  if ((HAL_GetTick() - lastTickMs) < 500U) {
    return;
  }
  lastTickMs = HAL_GetTick();

  const uint32_t a = 5000U;
  const uint16_t b = 5000U;
  const uint16_t c = 5000U;

  CAN_TxHeaderTypeDef txHeader = {0};
  uint8_t txData[8];
  uint32_t txMailbox = 0U;

  txHeader.StdId = 0x181U;
  txHeader.ExtId = 0U;
  txHeader.IDE = CAN_ID_STD;
  txHeader.RTR = CAN_RTR_DATA;
  txHeader.DLC = 8U;
  txHeader.TransmitGlobalTime = DISABLE;

  txData[0] = (uint8_t)(a >> 24);
  txData[1] = (uint8_t)(a >> 16);
  txData[2] = (uint8_t)(a >> 8);
  txData[3] = (uint8_t)(a);
  txData[4] = (uint8_t)(b >> 8);
  txData[5] = (uint8_t)(b);
  txData[6] = (uint8_t)(c >> 8);
  txData[7] = (uint8_t)(c);

  if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0U) {
    if (HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, &txMailbox) == HAL_OK) {
      g_can_tx_ok++;
    } else {
      g_can_tx_fail++;
    }
  } else {
    g_can_tx_fail++;
  }
}
*/

/**
  * @brief 辅助映射函数：将 10 十进制的实际信号值线性映射入 4~20mA (默认 819~4095) 的 DAC 数据字
  */
static uint16_t Map_To_4_20mA(int32_t val) {
    if (val <= SENSOR_MIN) return DAC_4MA_VAL;
    if (val >= SENSOR_MAX) return DAC_20MA_VAL;
    // 线性映射插值算法: 结果 = 最小值 + (当前-最小值) * (极差) / (满量程-最小值)
    return DAC_4MA_VAL + (uint16_t)(((int64_t)(val - SENSOR_MIN) * (int64_t)(DAC_20MA_VAL - DAC_4MA_VAL)) / (SENSOR_MAX - SENSOR_MIN));
}

/**
  * @brief CAN 接收回调：监听到电梯数据后直接输出到 DAC
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef rxHeader;
    uint8_t rxData[8];

  if ((hcan->Instance == CAN1) && (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxData) == HAL_OK)) {
    if ((rxHeader.StdId == 0x181U) && (rxHeader.DLC >= 8U)) {
            g_can_rx_ok++;
            /* Debug_PrintCanPayload(&rxHeader, rxData); */
            // 将 16 进制存储格式转换为 10 进制整型变量的值
            // 通道A: 前4个字节组成32位整型变量
            int32_t rawA = (int32_t)(((uint32_t)rxData[0] << 24) | ((uint32_t)rxData[1] << 16) | ((uint32_t)rxData[2] << 8) | rxData[3]);
            
            // 通道B: 中间2个字节组成16位整型（如果有符号可转 (int16_t) ）
            int32_t rawB = (int32_t)((rxData[4] << 8) | rxData[5]);
            
            // 通道C: 最后2个字节组成16位整型
            int32_t rawC = (int32_t)((rxData[6] << 8) | rxData[7]);

            // 通过映射公式，将 10进制变量 换算为 4-20mA 模拟电压对 DAC 的 12位 (819 - 4095)
            uint16_t valA = Map_To_4_20mA(rawA);
            uint16_t valB = Map_To_4_20mA(rawB);
            uint16_t valC = Map_To_4_20mA(rawC);

            /*
#if DEBUG_UART_ENABLE && DEBUG_CAN_RX_VERBOSE
            Debug_Printf("[MAP] RAW A=%ld B=%ld C=%ld -> DAC A=%u B=%u C=%u\r\n",
                         (long)rawA,
                         (long)rawB,
                         (long)rawC,
                         valA,
                         valB,
                         valC);
#endif
            */

      dac_chA_pending = valA;
      dac_chB_pending = valB;
      dac_chC_pending = valC;
      dac_update_pending = 1U;
            
            // 翻转 PA5 状态灯表示收到数据
            HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
        }
    }
}

/**
  * @brief 向 MCP4728 发送 I2C 指令 (Sequential Write 模式，从 Ch.A 顺序写三通道)
  *
  * 帧格式 (datasheet §5.6.2 Sequential Write):
  *   Byte 0: 0x50  = 0101_0000  命令字，从 Ch.A 开始顺序写
  *   Byte 1: cfg  | (D11~D8)   Ch.A 高字节: VRef=VDD(0), PD=Normal(00), Gain=x1(0), D11-8
  *   Byte 2: D7~D0             Ch.A 低字节
  *   Byte 3: cfg  | (D11~D8)   Ch.B 高字节 (同上)
  *   Byte 4: D7~D0             Ch.B 低字节
  *   Byte 5: cfg  | (D11~D8)   Ch.C 高字节 (同上)
  *   Byte 6: D7~D0             Ch.C 低字节
  *
  *   cfg = 0x00 时表示: VRef=VDD, PD=Normal, Gain=x1
  *
  * LDAC 时序: 先完成 I2C 传输，再给低脉冲触发三通道同步更新输出。
  */
void MCP4728_Write_3Channels(uint16_t chA, uint16_t chB, uint16_t chC) {
    // 1个命令字节 + 3个通道 × 2字节 = 7字节
    uint8_t data[7];
  const uint8_t cfg = 0x00U; // Vref=VDD, PD=Normal, Gain=x1
 
    // Byte 0: Sequential Write 命令，从 Ch.A 开始
    data[0] = 0x50;
 
    // Byte 1~2: 通道 A
  data[1] = cfg | ((chA >> 8) & 0x0F);   // 高字节: 配置位 + D11~D8
    data[2] = chA & 0xFF;                   // 低字节: D7~D0
 
    // Byte 3~4: 通道 B
  data[3] = cfg | ((chB >> 8) & 0x0F);   // 高字节: 配置位 + D11~D8
    data[4] = chB & 0xFF;                   // 低字节: D7~D0
 
    // Byte 5~6: 通道 C
  data[5] = cfg | ((chC >> 8) & 0x0F);   // 高字节: 配置位 + D11~D8
    data[6] = chC & 0xFF;                   // 低字节: D7~D0
 
  /*
#if DEBUG_UART_ENABLE && DEBUG_I2C_VERBOSE
  Debug_Printf("[I2C TX] MCP4728 data=%02X %02X %02X %02X %02X %02X %02X\r\n",
         data[0], data[1], data[2], data[3], data[4], data[5], data[6]);
#endif
  */
 
    // 先完成 I2C 传输，再触发 LDAC 低脉冲使三通道同步更新
    if (HAL_I2C_Master_Transmit(&hi2c1, MCP4728_ADDR, data, 7, 50) == HAL_OK) {
      g_i2c_tx_ok++;
      // LDAC 低脉冲：将 A/B/C 输入寄存器同步推送到物理输出引脚
      HAL_GPIO_WritePin(MCP4728_LDAC_GPIO_Port, MCP4728_LDAC_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(MCP4728_LDAC_GPIO_Port, MCP4728_LDAC_Pin, GPIO_PIN_SET);
    } else {
      g_i2c_tx_fail++;
  /*
#if DEBUG_UART_ENABLE
  Debug_Printf("[I2C ERR] HAL_I2C_Master_Transmit failed, err=0x%08lX\r\n", (unsigned long)HAL_I2C_GetError(&hi2c1));
#endif
  */
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
