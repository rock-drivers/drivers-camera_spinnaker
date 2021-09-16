
#ifndef CAMERA_SPINNAKER_CAMERA_H
#define CAMERA_SPINNAKER_CAMERA_H

// Header generated by dynamic_reconfigure
#include <camera_spinnaker/SpinnakerConfig.h>
#include <camera_spinnaker/set_property.h>

// Spinnaker SDK
#include <Spinnaker.h>
#include <SpinGenApi/SpinnakerGenApi.h>

//*******************************************
// This Class contains camera control functions.
// This base Class is based on the BlackFly S.
// Different cameras can extend/ override
// this class for any specific differences.
//*******************************************

namespace camera_spinnaker
{
class Camera
{
public:
  explicit Camera(Spinnaker::GenApi::INodeMap* node_map);
  ~Camera()
  {
  }
  virtual void setNewConfiguration(const camera_spinnaker::SpinnakerConfig& config, const uint32_t& level);

  /** Parameters that need a sensor to be stopped completely when changed. */
  static const uint8_t LEVEL_RECONFIGURE_CLOSE = 3;

  /** Parameters that need a sensor to stop streaming when changed. */
  static const uint8_t LEVEL_RECONFIGURE_STOP = 1;

  /** Parameters that can be changed while a sensor is streaming. */
  static const uint8_t LEVEL_RECONFIGURE_RUNNING = 0;

  virtual void setGain(const float& gain);
  int getHeightMax();
  int getWidthMax();

  Spinnaker::GenApi::CNodePtr
  readProperty(const Spinnaker::GenICam::gcstring property_name);

protected:
  Spinnaker::GenApi::INodeMap* node_map_;

  virtual void init();

  int height_max_;
  int width_max_;

  /*!
  * \brief Changes the video mode of the connected camera.
  *
  * This function will change the camera to the desired videomode and allow up the maximum framerate for that mode.
  * \param videoMode string of desired video mode, will be changed if unsupported.
  */
  virtual void setFrameRate(const float frame_rate);
  virtual void setImageControlFormats(const camera_spinnaker::SpinnakerConfig& config);
  /*!
  * \brief Set parameters relative to GigE cameras.
  *
  * \param auto_packet_size Flag stating if packet size should be automatically determined or not.
  * \param packet_size The packet size value to use if auto_packet_size is false.
  */
  // TODO(mhosmar): Implement later
  // void setGigEParameters(bool auto_packet_size, unsigned int packet_size, unsigned int packet_delay);

  /*!
  * \brief Will autoconfigure the packet size of the GigECamera with the given GUID.
  *
  * Note that this is expected only to work for GigE cameras, and only if the camera
  * is not connected.
  *
  * \param guid the camera to autoconfigure
  */
  // TODO(mhosmar): Implement later
  // void setupGigEPacketSize(FlyCapture2::PGRGuid & guid);

  /*!
  * \brief Will configure the packet size of the GigECamera with the given GUID to a given value.
  *
  * Note that this is expected only to work for GigE cameras, and only if the camera
  * is not connected.
  *
  * \param guid the camera to autoconfigure
  * \param packet_size The packet size value to use.
  */
  // TODO(mhosmar): Implement later
  // void setupGigEPacketSize(FlyCapture2::PGRGuid & guid, unsigned int packet_size);

  /*!
  * \brief Will configure the packet delay of the GigECamera with the given GUID to a given value.
  *
  * Note that this is expected only to work for GigE cameras, and only if the camera
  * is not connected.
  *
  * \param guid the camera to autoconfigure
  * \param packet_delay The packet delay value to use.
  */
  // TODO(mhosmar): Implement later
  // void setupGigEPacketDelay(FlyCapture2::PGRGuid & guid, unsigned int packet_delay);

  /*!
  * \brief Gets the current frame rate.
  *
  * Gets the camera's current reported frame rate.
  *
  * \return The reported frame rate.
  */
  // TODO(mhosmar): Implement later
  // float getCameraFrameRate();

  /*!
  * \brief Gets the current operating temperature.
  *
  * Gets the camera's current reported operating temperature.
  *
  * \return The reported temperature in Celsius.
  */
  // TODO(mhosmar): Implement later
  // float getCameraTemperature();

  // TODO(mhosmar): Implement the following methods later
  // void setBRWhiteBalance(bool auto_white_balance, uint16_t &blue, uint16_t &red);

  // uint getGain();

  // uint getShutter();

  // uint getBrightness();

  // uint getExposure();

  // uint getWhiteBalance();

  // uint getROIPosition();
};
}  // namespace camera_spinnaker
#endif  // CAMERA_SPINNAKER_CAMERA_H
