
#include "spi.h"
#include "tim.h"

#include "typedef.h"

#include "librm.hpp"

rm::modules::algorithm::PID imu_temp_pid{rm::modules::algorithm::PIDType::kPosition, 1600.f, 0.2f, 0.f, 4998.f, 200.f};

void BMI088TempControl(fp32 ref_temp, fp32 target_temp) {
  imu_temp_pid.Update(target_temp, ref_temp);
  __HAL_TIM_SetCompare(&htim10, TIM_CHANNEL_1, imu_temp_pid.value());
}

static rm::device::BMI088 *bmi088{nullptr};
static rm::modules::algorithm::AhrsInterface *ahrs_solver = new rm::modules::algorithm::MahonyAhrs{1000.f};
const rm::modules::algorithm::AhrsInterface *p_ahrs_solver = ahrs_solver;

extern "C" {

void InsInit() {
  HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
  bmi088 = new rm::device::BMI088{hspi1, CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, CS1_GYRO_GPIO_Port, CS1_GYRO_Pin};
}

void InsLoop() {
  BMI088TempControl(bmi088->temperature(), 45.f);
  bmi088->Update();
  ahrs_solver->Update(rm::modules::algorithm::ImuData6Dof{bmi088->gyro_x(), bmi088->gyro_y(), bmi088->gyro_z(),
                                                          bmi088->accel_x(), bmi088->accel_y(), bmi088->accel_z()});
}
}
