// Full ZED MCU reset (sensors + video module). The UVC video link sometimes stays wedged
// (`serial_num error` / no frames) after hobot_zed_cam exits; this re-enumerates the camera.
#include <chrono>
#include <cstdio>
#include <thread>

#include "sensorcapture.hpp"

int main(int argc, char ** argv)
{
  int sn = argc > 1 ? std::atoi(argv[1]) : 0;
  if (!sl_oc::sensors::SensorCapture::resetSensorModule(sn)) {
    std::fprintf(stderr, "zed_reset: reset failed (camera not found?)\n");
    return 1;
  }
  std::this_thread::sleep_for(std::chrono::seconds(4));  // wait for USB re-enumeration
  std::printf("zed_reset: camera reset done\n");
  return 0;
}
