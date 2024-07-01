
#include "struct_typedef.h"

typedef enum {
  ANGLE_DEGREE,
  ANGLE_RADIAN,
} angle_unit_e;

void broadcast_buffer_update(angle_unit_e unit, fp32 quat_w, fp32 quat_x,
                             fp32 quat_y, fp32 quat_z, fp32 roll, fp32 pitch,
                             fp32 yaw);
void uart1_broadcast_task(void *arg);
void uart6_broadcast_task(void *arg);