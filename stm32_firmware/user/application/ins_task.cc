
#include "spi.h"
#include "tim.h"
#include "cmsis_os.h"

#include "typedef.h"

#include "librm.hpp"

extern "C" fp32 GLOB_ins_quat_wxyz[4] = {0.0f, 0.0f, 0.0f, 0.0f};
SemaphoreHandle_t GLOB_imu_data_cv;
rm::modules::algorithm::PID imu_temp_pid{rm::modules::algorithm::PIDType::kPosition, 1600.f, 0.2f, 0.f, 5000.f, 200.f};

void BMI088TempControl(fp32 ref_temp, fp32 target_temp) {
  imu_temp_pid.Update(target_temp, ref_temp);
  __HAL_TIM_SetCompare(&htim10, TIM_CHANNEL_1, imu_temp_pid.value());
}

extern "C" void InsTask(void const *unused) {
  GLOB_imu_data_cv = xSemaphoreCreateBinary();
  osDelay(500);

  rm::device::BMI088 bmi088{hspi1, CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, CS1_GYRO_GPIO_Port, CS1_GYRO_Pin};
  rm::modules::algorithm::MahonyAhrs mahony{500.f};

  while (true) {
    osDelay(1);
    BMI088TempControl(bmi088.temperature(), 45.f);
    bmi088.Update();
    mahony.Update(rm::modules::algorithm::ImuData6Dof{bmi088.gyro_x(), bmi088.gyro_y(), bmi088.gyro_z(),
                                                      bmi088.accel_x(), bmi088.accel_y(), bmi088.accel_z()});
    xSemaphoreGive(GLOB_imu_data_cv);
    GLOB_ins_quat_wxyz[0] = mahony.quaternion().w;
    GLOB_ins_quat_wxyz[1] = mahony.quaternion().x;
    GLOB_ins_quat_wxyz[2] = mahony.quaternion().y;
    GLOB_ins_quat_wxyz[3] = mahony.quaternion().z;
  }
}
