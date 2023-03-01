#include <alproxies/alvideodeviceproxy.h>
#include <alvision/alvisiondefinitions.h>

#include <iostream>
#include "rosnao/imager.hpp"

int main(int argc, char **argv)
{
  if (argc < 5) {
    std::cerr << "nao_ip, shm_image_label, res {1=QVGA, 2=VGA}" << std::endl;
    return 1;
  }
  const std::string nao_ip = argv[1];
  const std::string shm_id = argv[2];

  // Proxy to ALVideoDevice.
  AL::ALVideoDeviceProxy cameraProxy(pIp);

  // Subscribe a Vision Module to ALVideoDevice, starting the
  // frame grabber if it was not started before.
  std::string subscriberID = "subscriberID";
  int fps = 5;
  // The subscriberID can be altered if other instances are already running
  subscriberID = cameraProxy.subscribe(subscriberID, AL::kVGA, AL::kRGBColorSpace, fps);

  // Do something...

  // Unsubscribe the V.M.
  cameraProxy.unsubscribe(subscriberID);

  return 0;
}