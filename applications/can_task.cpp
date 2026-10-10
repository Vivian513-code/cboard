#include <cmath>

#include "cmsis_os.h"
#include "io/buzzer/buzzer.hpp"
#include "io/can/can.hpp"
#include "io/dbus/dbus.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "tim.h"
#include "tools/mahony/mahony.hpp"
// ===== 外部对象引用 =====
extern sp::Mahony imu;

// ===== 全局对象 =====
sp::CAN can1(&hcan1);
sp::RM_Motor motorA(1, sp::RM_Motors::GM6020);
sp::RM_Motor motorB(2, sp::RM_Motors::GM6020);
sp::DBus remote(&huart3);
sp::Buzzer buzzer(&htim4, TIM_CHANNEL_3, 84e6f);

// ===== 参数 =====
constexpr float KP_LINK = 2.0f;
constexpr float KP_RESET = 3.0f;
constexpr float MANUAL_THRESHOLD = 0.3f;

constexpr float RATIO_DOWN = 0.5f;
constexpr float RATIO_MID = -1.0f;
constexpr float RATIO_UP = 3.0f;

constexpr uint32_t LED_CYCLE = 200;

constexpr float BUZZER_FREQ = 4000.0f;
constexpr uint32_t BUZZER_DURATION = 100;

// ===== 零点 =====
static float yaw_zero = 0.0f;
static float motorA_zero = 0.0f;
static float motorB_zero = 0.0f;

// ===== LED 状态 =====
static uint32_t led_counter = 0;
static uint8_t led_state = 0;  // 0=红, 1=绿, 2=蓝

// ===== LED 切换函数 =====
static void led_set(uint8_t state)
{
  // 先全灭
  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 0);
  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, 0);

  // 点亮对应颜色
  // 红=CH3, 绿=CH2, 蓝=CH1
  if (state == 0) __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, 65535);
  if (state == 1) __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 65535);
  if (state == 2) __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 65535);
}

extern "C" void can_task(void const * argument)
{
  // 蜂鸣器上电响一下
  buzzer.start();
  buzzer.set(BUZZER_FREQ, 0.5f);
  osDelay(BUZZER_DURATION);
  buzzer.stop();

  // 启动 CAN
  can1.config();
  can1.start();

  // 启动 DBus 接收
  remote.request();

  // 启动 LED PWM
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
  led_set(0);

  // 等电机反馈稳定
  osDelay(400);

  // 记录上电时的 R 标位置
  yaw_zero = imu.yaw;
  motorA_zero = motorA.angle;
  motorB_zero = motorB.angle;

  for (;;) {
    float vA = 0.0f;
    float vB = 0.0f;

    // ===== 读拨杆 =====
    auto sw_r = remote.sw_r;
    auto sw_l = remote.sw_l;

    if (sw_r == sp::DBusSwitchMode::DOWN) {
      // ===== 失能 =====
      vA = 0.0f;
      vB = 0.0f;
    }
    else if (sw_r == sp::DBusSwitchMode::MID) {
      // ===== 联动 =====
      float ratio_B = 0.0f;
      if (sw_l == sp::DBusSwitchMode::DOWN)
        ratio_B = RATIO_DOWN;
      else if (sw_l == sp::DBusSwitchMode::MID)
        ratio_B = RATIO_MID;
      else if (sw_l == sp::DBusSwitchMode::UP)
        ratio_B = RATIO_UP;

      float yaw_delta = imu.yaw - yaw_zero;
      float aA = motorA.angle;
      float aB = motorB.angle;

      float targetA = motorA_zero + yaw_delta;
      float targetB = motorB_zero + yaw_delta * ratio_B;

      // 手动检测
      float errA = targetA - aA;
      if (fabsf(errA) > MANUAL_THRESHOLD) {
        motorA_zero = aA - yaw_delta;
        motorB_zero = aB - yaw_delta * ratio_B;
        targetA = aA;
        targetB = aB;
      }

      vA = (targetA - aA) * KP_LINK;
      vB = (targetB - aB) * KP_LINK;
    }
    else if (sw_r == sp::DBusSwitchMode::UP) {
      // ===== 复位 =====
      vA = (motorA_zero - motorA.angle) * KP_RESET;
      vB = (motorB_zero - motorB.angle) * KP_RESET;
    }

    // ===== 发送 CAN =====
    for (int i = 0; i < 8; i++) can1.tx_data[i] = 0;
    motorA.cmd(vA);
    motorB.cmd(vB);
    motorA.write(can1.tx_data);
    motorB.write(can1.tx_data);
    can1.send(motorA.tx_id);  // 0x1FF

    // ===== LED 切换 =====
    led_counter++;
    if (led_counter >= LED_CYCLE) {
      led_counter = 0;
      led_state = (led_state + 1) % 3;
      led_set(led_state);
    }

    osDelay(1);
  }
}

// ===== CAN 接收回调 =====
extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef * hcan)
{
  auto stamp_ms = osKernelSysTick();
  while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0) {
    if (hcan == &hcan1) {
      can1.recv();
      if (can1.rx_id == motorA.rx_id) motorA.read(can1.rx_data, stamp_ms);
      if (can1.rx_id == motorB.rx_id) motorB.read(can1.rx_data, stamp_ms);
    }
  }
}

// ===== DBus 接收回调 =====
extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t Size)
{
  auto stamp_ms = osKernelSysTick();
  if (huart == &huart3) {
    remote.update(Size, stamp_ms);
    remote.request();
  }
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart)
{
  if (huart == &huart3) {
    remote.request();
  }
}