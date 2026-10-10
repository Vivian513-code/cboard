#include "cmsis_os.h"
#include "io/bmi088/bmi088.hpp"
#include "io/plotter/plotter.hpp"

extern sp::BMI088 bmi088;

sp::Plotter plotter(&huart1);

extern "C" void plotter_task(void const * argument)
{
  for (;;) {
    plotter.plot(
      bmi088.acc[0], bmi088.acc[1], bmi088.acc[2], bmi088.gyro[0], bmi088.gyro[1], bmi088.gyro[2]);
    osDelay(1);
  }
}