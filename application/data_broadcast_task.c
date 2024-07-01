
#include "data_broadcast_task.h"

#include "can.h"
#include "cmsis_os.h"
#include "usart.h"

#include <string.h>

#define M_PI 3.14159265358979323846

// imu data[14 bytes]: quaternion_wxyz(0~1 mapped to int16) [4x int16]
//                     euler_rpy(-2pi~2pi mapped to int16)  [3x int16]
static char uart_buf[14] = {0};

static fp32 degree_to_radian(fp32 degree) { return degree * M_PI / 180.0f; }

/**
 * @brief map angle values(-2pi, 2pi) to int16_t(-32768, 32767)
 */
static int16_t map_angle_to_int16(fp32 angle, angle_unit_e unit) {
  if (unit == ANGLE_DEGREE) {
    angle = degree_to_radian(angle);
  }
  return (int16_t)(angle / (2 * M_PI) * 32767);
}

/**
 * @brief update the buffer to be broadcasted
 * @note  quaternions should be normalized!
 */
void broadcast_buffer_update(angle_unit_e unit, fp32 quat_w, fp32 quat_x,
                             fp32 quat_y, fp32 quat_z, fp32 roll, fp32 pitch,
                             fp32 yaw) {
  int16_t quat_w_int16 = (int16_t)(quat_w * 32767);
  int16_t quat_x_int16 = (int16_t)(quat_x * 32767);
  int16_t quat_y_int16 = (int16_t)(quat_y * 32767);
  int16_t quat_z_int16 = (int16_t)(quat_z * 32767);
  int16_t roll_int16 = map_angle_to_int16(roll, unit);
  int16_t pitch_int16 = map_angle_to_int16(pitch, unit);
  int16_t yaw_int16 = map_angle_to_int16(yaw, unit);
  memcpy(uart_buf, &quat_w_int16, 2);
  memcpy(&uart_buf[2], &quat_x_int16, 2);
  memcpy(&uart_buf[4], &quat_y_int16, 2);
  memcpy(&uart_buf[6], &quat_z_int16, 2);
  memcpy(&uart_buf[8], &roll_int16, 2);
  memcpy(&uart_buf[10], &pitch_int16, 2);
  memcpy(&uart_buf[12], &yaw_int16, 2);
}

/**
 * @brief broadcast the buffer to uart1
 */
void uart1_broadcast_task(void *arg) {
  osDelay(100);
  while (1) {
    osDelay(1);
    // wait for the previous transmission to finish
    while (HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY) {
    }
    HAL_UART_Transmit(&huart1, (uint8_t *)uart_buf, 14, 0xff);
  }
}

/**
 * @brief broadcast the buffer to uart6
 */
void uart6_broadcast_task(void *arg) {
  osDelay(100);
  while (1) {
    osDelay(1);
    // wait for the previous transmission to finish
    while (HAL_UART_GetState(&huart6) != HAL_UART_STATE_READY) {
    }
    HAL_UART_Transmit(&huart6, (uint8_t *)uart_buf, 14, 0xff);
  }
}

void broadcast_can() {
  // TODO
}
