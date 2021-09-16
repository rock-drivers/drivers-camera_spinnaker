
#ifndef CAMERA_SPINNAKER_CM3_H
#define CAMERA_SPINNAKER_CM3_H

#include <camera_spinnaker/camera.h>

namespace camera_spinnaker
{
class Cm3 : public Camera
{
public:
  explicit Cm3(Spinnaker::GenApi::INodeMap* node_map);
  ~Cm3();
  void setFrameRate(const float frame_rate);
  void setNewConfiguration(const SpinnakerConfig& config, const uint32_t& level);

private:
  void setImageControlFormats(const camera_spinnaker::SpinnakerConfig& config);
};
}  // namespace camera_spinnaker
#endif  // CAMERA_SPINNAKER_CM3_H
