#ifndef CAMERA_SPINNAKER_SPINNAKERCAMERA_H
#define CAMERA_SPINNAKER_SPINNAKERCAMERA_H

#include <camera_spinnaker/camera_exceptions.h>

#include <sstream>
#include <mutex>
#include <string>

// Header generated by dynamic_reconfigure
#include <camera_spinnaker/SpinnakerConfig.h>
#include "camera_spinnaker/camera.h"
#include "camera_spinnaker/cm3.h"
#include "camera_spinnaker/gh3.h"
#include "camera_spinnaker/set_property.h"

// Base types
#include <base/samples/Frame.hpp>

// Spinnaker SDK
#include <Spinnaker.h>
#include <SpinGenApi/SpinnakerGenApi.h>

namespace camera_spinnaker
{
class SpinnakerCamera
{
public:
  SpinnakerCamera();
  ~SpinnakerCamera();

  /*!
  * \brief Function that allows reconfiguration of the camera.
  *
  * This function handles a reference of a camera_library::CameraConfig object and
  * configures the camera as close to the given values as possible.  As a function for
  * dynamic_reconfigure, values that are not valid are changed by the driver and can
  * be inspected after this function ends.
  * This function will stop and restart the camera when called on a SensorLevels::RECONFIGURE_STOP level.
  * \param config  camera_library::CameraConfig object passed by reference.  Values will be changed to those the driver
  * is currently using.
  * \param level  Reconfiguration level. See constants below for details.
  *
  * \return Returns true when the configuration could be applied without modification.
  */
  void setNewConfiguration(const camera_spinnaker::SpinnakerConfig& config, const uint32_t& level);

  /** Parameters that need a sensor to be stopped completely when changed. */
  static const uint8_t LEVEL_RECONFIGURE_CLOSE = 3;

  /** Parameters that need a sensor to stop streaming when changed. */
  static const uint8_t LEVEL_RECONFIGURE_STOP = 1;

  /** Parameters that can be changed while a sensor is streaming. */
  static const uint8_t LEVEL_RECONFIGURE_RUNNING = 0;

  /*!
  * \brief Function that connects to a specified camera.
  *
  * Will connect to the camera specified in the setDesiredCamera(std::string id) call.  If setDesiredCamera is not
  * called first
  * this will connect to the first camera.  Connecting to the first camera is not recommended for multi-camera or
  * production systems.
  * This function must be called before setNewConfiguration() or start()!
  */
  void connect();

  /*!
  * \brief Disconnects from the camera.
  *
  * Disconnects the camera and frees it.
  */
  void disconnect();

  /*!
  * \brief Starts the camera loading data into its buffer.
  *
  * This function will start the camera capturing images and loading them into the buffer.  To retrieve images,
  * grabImage must be called.
  */
  void start();

  /*!
  * \brief Stops the camera loading data into its buffer.
  *
  * This function will stop the camera capturing images and loading them into the buffer.
  */
  void stop();

  /*!
  * \brief Loads the raw data from the cameras buffer.
  *
  * This function will load the raw data from the buffer and place it into a Frame Image.
  * \param image base::samples::frame::Frame that will be filled with the image currently in the buffer.
  * \param frame_id The name of the optical frame of the camera.
  */
  void grabImage(base::samples::frame::Frame* image, const std::string& frame_id);

  /*!
  * \brief Will set grabImage timeout for the camera.
  *
  * This function will set the time required for grabCamera to throw a timeout exception.  Must be called after
  * connect().
  * \param timeout The desired timeout value (in seconds)
  *
  */
  // TODO(mhosmar): Implement later
  void setTimeout(const double& timeout);

  /*!
  * \brief Used to set the serial number for the camera you wish to connect to.
  *
  * Sets the desired serial number.  If this value is not set, the driver will try to connect to the first camera on the
  * bus.
  * This function should be called before connect().
  * \param id serial number for the camera.  Should be something like 10491081.
  */
  void setDesiredCamera(const uint32_t& id);

  void setGain(const float& gain);
  int getHeightMax();
  int getWidthMax();
  Spinnaker::GenApi::CNodePtr readProperty(const Spinnaker::GenICam::gcstring property_name);

  uint32_t getSerial()
  {
    return serial_;
  }

private:
  uint32_t serial_;  ///< A variable to hold the serial number of the desired camera.

  Spinnaker::SystemPtr system_;
  Spinnaker::CameraList camList_;
  Spinnaker::CameraPtr pCam_;

  // TODO(mhosmar) use std::shared_ptr
  Spinnaker::GenApi::INodeMap* node_map_;
  std::shared_ptr<Camera> camera_;

  Spinnaker::ChunkData image_metadata_;

  std::mutex mutex_;  ///< A mutex to make sure that we don't try to grabImages while reconfiguring or vice versa.
  volatile bool captureRunning_;  ///< A status boolean that checks if the camera has been started and is loading images
                                  ///  into its buffer.

  /// If true, camera is currently running in color mode, otherwise camera is running in mono mode
  bool isColor_;

  // For GigE cameras:
  /// If true, GigE packet size is automatically determined, otherwise packet_size_ is used:
  bool auto_packet_size_;
  /// GigE packet size:
  unsigned int packet_size_;
  /// GigE packet delay:
  unsigned int packet_delay_;

  uint64_t timeout_;

  // This function configures the camera to add chunk data to each image. It does
  // this by enabling each type of chunk data before enabling chunk data mode.
  // When chunk data is turned on, the data is made available in both the nodemap
  // and each image.
  void ConfigureChunkData(const Spinnaker::GenApi::INodeMap& nodeMap);
};
}  // namespace camera_spinnaker
#endif  // CAMERA_SPINNAKER_SPINNAKERCAMERA_H
