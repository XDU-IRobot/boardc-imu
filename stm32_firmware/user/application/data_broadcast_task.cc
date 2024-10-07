
#include "cmsis_os.h"
#include "usbd_cdc_if.h"
#include "can.h"

#include "typedef.h"

#include "librm.hpp"

extern fp32 GLOB_ins_quat_wxyz[4];

/**
 * @note  packet structure
 * @note  [SOF(0xA5), SEQ, CRC8(SOF+SEQ),
 * @note  quat_w_l, quat_w_h, quat_x_l, quat_x_h, quat_y_l, quat_y_h, quat_z_l, quat_z_h,
 * @note  CRC16_L, CRC16_H]
 * @note  total 13 bytes
 */
static unsigned char buf[13] = {0xa5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

extern SemaphoreHandle_t GLOB_imu_data_cv;
rm::hal::Can *can1, *can2;

/**
 * @brief update the buffer to be broadcasted
 * @note  quaternions should be normalized!
 */
static void Update(fp32 quat_w, fp32 quat_x, fp32 quat_y, fp32 quat_z) {
  *(uint16_t *)(&buf[3]) = rm::modules::algorithm::utils::FloatToInt(quat_w, -1.f, 1.f, 16);
  *(uint16_t *)(&buf[5]) = rm::modules::algorithm::utils::FloatToInt(quat_x, -1.f, 1.f, 16);
  *(uint16_t *)(&buf[7]) = rm::modules::algorithm::utils::FloatToInt(quat_y, -1.f, 1.f, 16);
  *(uint16_t *)(&buf[9]) = rm::modules::algorithm::utils::FloatToInt(quat_z, -1.f, 1.f, 16);
  ++(buf[1]);
  buf[2] = rm::modules::algorithm::Crc8(buf, 2, rm::modules::algorithm::CRC8_INIT);
  *(uint16_t *)(&buf[11]) = rm::modules::algorithm::Crc16(buf, 11, rm::modules::algorithm::CRC16_INIT);
}

/**
 * @brief broadcast the buffer to usb
 */
extern "C" void DataBroadcastTask(void *unused) {
  osDelay(500);  // 这里的延时不要太短！否则会导致信号量还未初始化就被take；初始化过程见ins_task.cc

  can1 = new rm::hal::Can{hcan1};
  can2 = new rm::hal::Can{hcan2};
  can1->Begin();
  can2->Begin();

  while (true) {
    xSemaphoreTake(GLOB_imu_data_cv, portMAX_DELAY);  // wait for data update
    Update(GLOB_ins_quat_wxyz[0], GLOB_ins_quat_wxyz[1], GLOB_ins_quat_wxyz[2], GLOB_ins_quat_wxyz[3]);

    CDC_Transmit_FS(buf, 13);        // broadcast via usb
    can1->Write(0xaaa, &buf[3], 8);  // broadcast via can1
    can2->Write(0xaaa, &buf[3], 8);  // broadcast via can2
  }
}
