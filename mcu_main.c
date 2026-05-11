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

#include "main.h"
#include "gpio.h"
#include "dma.h"
#include "can.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "YIS130.h"
#include "arm_math.h"
#include "IM_TEST.h"
#include "stdio.h"
#include "AS5048.h"
#include "odom_protocol.h"
#include <math.h>
#include <string.h>

extern CAN_HandleTypeDef hcan1;
extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;
extern TIM_HandleTypeDef htim11;
extern TIM_HandleTypeDef htim13;
extern TIM_HandleTypeDef htim14;
extern UART_HandleTypeDef huart1;

void MX_GPIO_Init(void);
void MX_DMA_Init(void);
void MX_CAN1_Init(void);
void MX_TIM11_Init(void);
void MX_TIM13_Init(void);
void MX_TIM14_Init(void);
void MX_USART1_UART_Init(void);
void MX_SPI1_Init(void);
void MX_SPI2_Init(void);

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define USART_CR3_RXFTIE ((uint32_t)0x00008000)

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

uint8_t receive_buff[8] = {0}; // init
int receivefactor[2];
// header x_low x_high y y yaw yaw footer
uint8_t tx_buff[255];


extern MPU_DATA mpu_data[4];
extern AS5048 AS5048s[AS5048_NUMBER];
float i = 0;
extern float ACCX,ACCY,ACCZ;
//float tt_x = 0;
//float tt_y = 0;
//float tt_x_real = 0;
//float tt_y_real = 0;


int add = 0;
int times = 0;

int fputc(int ch, FILE *f){
        //HAL_UART_Transmit_DMA(&huart1, (uint8_t *)&ch, 1);
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xffff);
  return ch;
}

typedef struct struct_message
{
  uint8_t header;
  uint8_t parity;
  uint8_t data[6];
  uint8_t footer;
} DataPacket;

DataPacket DataRe;
uint8_t USART_FLAG = 0;


uint8_t USART1_RX_BUF[100]; 
uint16_t USART1_RX_STA = 0; 
uint8_t aRxBuffer1[1];            
UART_HandleTypeDef UART1_Handler; 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* RX 缓冲扩大到 64 字节以兼容旧 8 字节协议 + 新 0xAA 0x55 上行帧 (最长 25 字节) */
#define RCV_BUF_SIZE 64
uint8_t rcv_buf[RCV_BUF_SIZE] = {0};
volatile uint16_t rcv_len = 0;   /* 实际收到字节数, 由 idle 中断记录 */
int rcv_err = 3;
char mpu_buff[220];
uint16_t rxclear = 0;
int rcv_flag = 0;
int rst_temp = 0;

/* ODOM protocol variables */
static uint8_t odom_seq = 0;
static uint8_t odom_frame_buf[ODOM_STATE_FRAME_LEN];
/* 上行响应使用独立缓冲, 避免与下行 ODOM_STATE 共用 */
static uint8_t odom_resp_buf[ODOM_TIME_SYNC_RESP_FRAME_LEN];
/* 累计成功归零次数 (event_counter) */
static uint32_t origin_event_counter = 0;
static float odom_dx_world_acc = 0.0f;
static float odom_dy_world_acc = 0.0f;
static float odom_dyaw_acc = 0.0f;
static uint16_t odom_vel_ticks = 0;

/* ODOM output rate: ticks between frames (ISR=20kHz, 200 ticks=10ms=100Hz) */
#define ODOM_OUTPUT_TICKS 200
/* ISR period in microseconds */
#define ODOM_ISR_PERIOD_US 50

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
  MX_CAN1_Init();
  MX_TIM11_Init();
  MX_TIM13_Init();
  MX_TIM14_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */

        __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
        HAL_UART_Receive_DMA(&huart1, rcv_buf, RCV_BUF_SIZE);

        AS5048_init(1,&hspi1,GPIOA,GPIO_PIN_4);
        AS5048_init(2,&hspi2,GPIOB,GPIO_PIN_12);

        mpu_data[0].cali = 1;
        mpu_data[0].vel[0] = 0;
        mpu_data[0].vel[1] = 0;
        mpu_data[0].REAL_YAW_SET = 0;
        mpu_data[0].REAL_YAW_MARK = 0;

        can_filter_init();
        IM_TEST_initialize();

        HAL_TIM_Base_Start_IT(&htim13);
        HAL_TIM_Base_Start_IT(&htim14);
        HAL_TIM_Base_Start_IT(&htim11);


//      SelfCalibration();

//      HAL_Delay(100);


//  mpu_data[0].PITCH_ANGLE_BEG = mpu_data[0].PITCH_ANGLE;
//  mpu_data[0].YAW_ANGLE_BEG =   mpu_data[0].YAW_ANGLE;
//  mpu_data[0].ROLL_ANGLE_BEG =  mpu_data[0].ROLL_ANGLE;
//
  /* USER CODE END 2 */

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
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) //串口接收中断
//{
//                      while  (huart->Instance == USART1)
//              {
//                              USART1_RX_BUF[USART1_RX_STA] = aRxBuffer1[0];
//                              if (USART1_RX_STA == 0 && USART1_RX_BUF[USART1_RX_STA] != 0x0F)
//                              {
//                                      HAL_UART_Receive_DMA(&huart1,aRxBuffer1,1);
//                                      break; //
//                              }
//                              USART1_RX_STA++;
//                      HAL_UART_Receive_DMA(&huart1,aRxBuffer1,1);
//                      if (USART1_RX_STA > 100) USART1_RX_STA = 0;  //
//                      if (USART1_RX_BUF[0] == 0x0F && USART1_RX_BUF[7] == 0xAA && USART1_RX_STA == 8)
//                      {
//                              DATARELOAD(USART1_RX_BUF);
//                              receivefactor[1]=1;
//                              USART1_RX_STA = 0;
//                      }
//                      else if(!(USART1_RX_BUF[0] == 0x0F && USART1_RX_BUF[7] == 0xAA) && USART1_RX_STA == 8){
//                              for(int i=0;i<8;i++)
//                                      USART1_RX_BUF[i] = 0;
//                              USART1_RX_STA = 0;
//                      }
//                      break;
//              }
//}

//DMA+空闲中断接收
void Rcv_IdleCallback(void){
        /* 判断是否为空闲中断 */
        if(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE) == SET){
                /* 清除空闲中断标志并停止 DMA */
                __HAL_UART_CLEAR_IDLEFLAG(&huart1);
                HAL_UART_DMAStop(&huart1);
                /* 计算实际接收字节数 = 缓冲总长 - 剩余 NDTR */
                uint16_t remaining = (uint16_t)__HAL_DMA_GET_COUNTER(huart1.hdmarx);
                rcv_len = (remaining > RCV_BUF_SIZE) ? 0 : (uint16_t)(RCV_BUF_SIZE - remaining);
                /* 标记收到一帧 */
                rcv_flag = 1;
        }
}

/* 等待上一次串口 TX (DMA 或阻塞) 完成, 然后用 DMA 发送响应帧。
 * 在 TIM11 ISR 里调用, 100Hz 下 ODOM_STATE 占用约 4ms, 用轮询保护避免覆盖。*/
static void send_upstream_response(const uint8_t *buf, uint16_t len)
{
        uint32_t guard = 0;
        /* gState != HAL_UART_STATE_READY 表示 TX 还在进行 */
        while(huart1.gState != HAL_UART_STATE_READY){
                if(++guard > 200000U){ return; }  /* 超时直接放弃, 避免 ISR 死循环 */
        }
        HAL_UART_Transmit_DMA(&huart1, (uint8_t *)buf, len);
}

/* 处理上位机发来的 0xAA 0x55 帧 */
static void handle_upstream_frame(const OdomUpstreamFrame_t *fr)
{
        if(fr->msg_type == ODOM_MSG_TIME_SYNC_REQ){
                if(fr->payload_len != ODOM_TIME_SYNC_REQ_PAYLOAD_LEN) return;
                /* 提取 echoed_host_time_us */
                uint64_t host_us = 0;
                for(int b = 0; b < 8; b++){
                        host_us |= ((uint64_t)fr->payload[b]) << (8 * b);
                }
                OdomTimeSyncRespPayload_t resp;
                resp.echoed_host_time_us = host_us;
                resp.mcu_time_us = odom_isr_tick * (uint64_t)ODOM_ISR_PERIOD_US;
                uint16_t flen = odom_pack_time_sync_resp(odom_resp_buf, fr->seq, &resp);
                send_upstream_response(odom_resp_buf, flen);
        } else if(fr->msg_type == ODOM_MSG_SET_LOCAL_ORIGIN){
                if(fr->payload_len != ODOM_SET_ORIGIN_PAYLOAD_LEN) return;
                /* payload: float x, y, yaw + uint8 flags + 3 reserved */
                float new_x, new_y, new_yaw;
                memcpy(&new_x,   &fr->payload[0],  4);
                memcpy(&new_y,   &fr->payload[4],  4);
                memcpy(&new_yaw, &fr->payload[8],  4);
                /* uint8_t flags = fr->payload[12]; -- 当前忽略, 全字段重置 */

                /* 应用归零: 直接覆盖累积位姿 */
                mpu_data[0].X_tt = new_x;
                mpu_data[0].Y_tt = new_y;
                mpu_data[0].REAL_X = new_x;
                mpu_data[0].REAL_Y = new_y;
                /* 重置连续 yaw: 让下次 unwrap 输出 = new_yaw, prev_deg 锚定到当前角度 */
                g_yaw_unwrap.continuous_rad = new_yaw;
                g_yaw_unwrap.prev_deg       = mpu_data[0].YAW_ANGLE;
                g_yaw_unwrap.initialized    = 1;
                origin_event_counter++;

                OdomSetLocalOriginAckPayload_t ack;
                ack.acked_seq     = (uint16_t)fr->seq;
                ack.result_code   = 0;
                ack.event_counter = origin_event_counter;
                uint16_t flen = odom_pack_set_origin_ack(odom_resp_buf, fr->seq, &ack);
                send_upstream_response(odom_resp_buf, flen);
        }
        /* 其它类型忽略 */
}

int Rcv_DealData(void){
        if(1==rcv_flag){
                uint16_t len = rcv_len;
                /* --- 1) 老协议: 8 字节固定包 --- */
                if(len >= 8 && 0x0F==rcv_buf[0] && 0xAA==rcv_buf[7]){
                        DATARELOAD(rcv_buf);
                }else if(len >= 8 && 0xBB==rcv_buf[0] && 0xCC==rcv_buf[7]){
                        HAL_GPIO_WritePin(RST_CTRL_GPIO_Port,RST_CTRL_Pin,GPIO_PIN_SET);
                        HAL_Delay(500);
                        HAL_GPIO_WritePin(RST_CTRL_GPIO_Port,RST_CTRL_Pin,GPIO_PIN_RESET);
                        DATARELOAD(rcv_buf);
                }else{
                        /* --- 2) 新协议: 0xAA 0x55 上行帧 --- */
                        OdomUpstreamFrame_t fr;
                        if(odom_parse_upstream(rcv_buf, len, &fr)){
                                handle_upstream_frame(&fr);
                        }
                }
                /* 清缓冲并重启 DMA */
                memset(rcv_buf, 0, RCV_BUF_SIZE);
                rcv_len  = 0;
                rcv_flag = 0;
                HAL_UART_Receive_DMA(&huart1, rcv_buf, RCV_BUF_SIZE);
                return 0;
        }else{
                return -1;
        }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == (&htim14)){
                        //if(rcv_err>0){
      //ClearUARTErrors(USART1);//清除串口错误标志，目前不再使用
                        //}

    }

                if (htim == (&htim13)){
//                      arm_fir_f32_lp();

                 }

                 if (htim == (&htim11)){

                         odom_isr_tick++;

                         if(mpu_data[0].cali == 1){
//                                      rtU.X_ACCIN  = mpu_data[0].acc_cali[0];
//                                      rtU.Y_ACCIN  = mpu_data[0].acc_cali[1];

                                        if(times >= 500){

                                                add++;
                                                AS5048_getREGValue(1);
                                                AS5048_dataUpdate(1);
                                                AS5048_getREGValue(2);
                                                AS5048_dataUpdate(2);

                                                mpu_data[0].REAL_YAW = mpu_data[0].YAW_ANGLE;

            rtU.W1 = -AS5048s[1].delta_dis * AS5048_LEFT_METERS_PER_COUNT;
            rtU.W2 = AS5048s[0].delta_dis * AS5048_RIGHT_METERS_PER_COUNT;
            rtU.DEG = mpu_data[0].REAL_YAW;

            mpu_data[0].Y_tt += rtY.YOUT * IM_TEST_ODOM_OUTPUT_SCALE;
            mpu_data[0].X_tt += rtY.XOUT * IM_TEST_ODOM_OUTPUT_SCALE;
                                                mpu_data[0].REAL_Y = mpu_data[0].Y_tt;
                                                mpu_data[0].REAL_X = mpu_data[0].X_tt;

                                                /* 累积世界系增量用于速度计算 */
                                                odom_dx_world_acc += rtY.XOUT * IM_TEST_ODOM_OUTPUT_SCALE;
                                                odom_dy_world_acc += rtY.YOUT * IM_TEST_ODOM_OUTPUT_SCALE;
                                                odom_vel_ticks++;

                                                Rcv_DealData();

#if ODOM_BINARY_MODE
                                                if(add >= ODOM_OUTPUT_TICKS){
                                                        /* 连续 yaw (rad) */
                                                        float yaw_cont = odom_yaw_unwrap(&g_yaw_unwrap, mpu_data[0].YAW_ANGLE);

                                                        /* 计算 body-frame 速度 */
                                                        float dt = (float)odom_vel_ticks * (float)ODOM_ISR_PERIOD_US * 1e-6f;
                                                        float vx_world = 0.0f, vy_world = 0.0f;
                                                        if(dt > 0.0f){
                                                            vx_world = odom_dx_world_acc / dt;
                                                            vy_world = odom_dy_world_acc / dt;
                                                        }
                                                        /* 旋转到 body frame */
                                                        float cos_yaw = arm_cos_f32(yaw_cont);
                                                        float sin_yaw = arm_sin_f32(yaw_cont);
                                                        float vx_body =  vx_world * cos_yaw + vy_world * sin_yaw;
                                                        float vy_body = -vx_world * sin_yaw + vy_world * cos_yaw;
                                                        float wz = mpu_data[0].gyro[2]; /* IMU gyro Z, rad/s */

                                                        /* 时间戳 */
                                                        uint64_t t_us = odom_isr_tick * (uint64_t)ODOM_ISR_PERIOD_US;

                                                        /* 状态位 */
                                                        uint16_t status = ODOM_STATUS_ENC_VALID | ODOM_STATUS_IMU_VALID
                                                                        | ODOM_STATUS_YAW_VALID | ODOM_STATUS_POS_VALID
                                                                        | ODOM_STATUS_VEL_VALID;

                                                        /* 打包 ODOM_STATE */
                                                        OdomStatePayload_t payload;
                                                        payload.t_sample_us = t_us;
                                                        payload.x = mpu_data[0].REAL_X;
                                                        payload.y = mpu_data[0].REAL_Y;
                                                        payload.yaw = yaw_cont;
                                                        payload.vx = vx_body;
                                                        payload.vy = vy_body;
                                                        payload.wz = wz;
                                                        payload.status_bits = status;
                                                        payload.quality = ODOM_QUALITY_NORMAL;
                                                        payload.reserved = 0;

                                                        uint16_t frame_len = odom_pack_state(odom_frame_buf, odom_seq++, &payload);
                                                        HAL_UART_Transmit_DMA(&huart1, odom_frame_buf, frame_len);

                                                        /* 重置累积器 */
                                                        odom_dx_world_acc = 0.0f;
                                                        odom_dy_world_acc = 0.0f;
                                                        odom_vel_ticks = 0;
                                                        add = 0;
                                                }
#else
                                                if(add >= 50){
                                                        float enc_left_total_m = -AS5048s[1].total_angle * AS5048_LEFT_METERS_PER_COUNT;
                                                        float enc_right_total_m = AS5048s[0].total_angle * AS5048_RIGHT_METERS_PER_COUNT;
                                                        float enc_avg_total_m = 0.5f * (enc_left_total_m + enc_right_total_m);
                                                        memset(mpu_buff, 0, sizeof(mpu_buff));
                                                        int mpu_len = snprintf(mpu_buff, sizeof(mpu_buff), "%f,%f,%f,%f,%f,%f,%f,%ld,%ld\r\n", mpu_data[0].REAL_X, mpu_data[0].REAL_Y, mpu_data[0].REAL_YAW, mpu_data[0].ROLL_ANGLE, enc_left_total_m, enc_right_total_m, enc_avg_total_m, (long)AS5048s[1].total_angle, (long)AS5048s[0].total_angle);
                                                        HAL_UART_Transmit_DMA(&huart1, (uint8_t *)&mpu_buff, mpu_len);
                                                        add = 0;
                                                }
#endif

                                        }else{

                                                        AS5048_getREGValue(1);
                                                        AS5048_dataUpdate(1);
                                                        AS5048_getREGValue(2);
                                                        AS5048_dataUpdate(2);
                                          mpu_data[0].vel[0] = 0;
                                    mpu_data[0].vel[1] = 0;
                                                times ++ ;
                                        }

                                IM_TEST_step();

                 }
                }
}

void ClearUARTErrors(USART_TypeDef *USARTx) {
    // 清除奇偶校验错误
    if (USARTx->SR & USART_SR_PE) {
        (void)USARTx->DR;
    }
    // 清除帧错误
    if (USARTx->SR & USART_SR_FE) {
        (void)USARTx->DR;
    }
    // 清除噪声错误
    if (USARTx->SR & USART_SR_NE) {
        (void)USARTx->DR;
    }
    // 清除溢出错误
    if (USARTx->SR & USART_SR_ORE) {
        (void)USARTx->DR;
    }
    // 重新使能串口
    USARTx->CR1 |= USART_CR1_UE;
    // 重新使能错误中断
    USARTx->CR3 |= USART_CR3_EIE;
    // 重新使能接收 FIFO 阈值中断
    USARTx->CR3 |= USART_CR3_RXFTIE;
                // 重新打开 DMA 接收
                HAL_UART_Receive_DMA(&huart1,rcv_buf,RCV_BUF_SIZE);
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






