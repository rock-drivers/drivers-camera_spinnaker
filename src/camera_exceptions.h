
#ifndef CAMERA_SPINNAKER_CAMERA_EXCEPTIONS_H
#define CAMERA_SPINNAKER_CAMERA_EXCEPTIONS_H

#include <stdexcept>
#include <string>

class CameraTimeoutException : public std::runtime_error
{
public:
  CameraTimeoutException() : runtime_error("Image not found within timeout.")
  {
  }
  explicit CameraTimeoutException(const std::string& msg) : runtime_error(msg.c_str())
  {
  }
};

class CameraNotRunningException : public std::runtime_error
{
public:
  CameraNotRunningException() : runtime_error("Camera is currently not running.  Please start the capture.")
  {
  }
  explicit CameraNotRunningException(const std::string& msg) : runtime_error(msg.c_str())
  {
  }
};

class CameraImageNotReadyException : public std::runtime_error
{
public:
  CameraImageNotReadyException() : runtime_error("Image is currently not ready.")
  {
  }
  explicit CameraImageNotReadyException(const std::string& msg) : runtime_error(msg.c_str())
  {
  }
};

#endif  // CAMERA_SPINNAKER_CAMERA_EXCEPTIONS_H
