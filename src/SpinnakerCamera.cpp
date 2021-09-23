#include <camera_spinnaker/SpinnakerCamera.h>

#include <iostream>
#include <sstream>
#include <typeinfo>
#include <string>
#include <cmath>

#include <base-logging/Logging.hpp>

namespace camera_spinnaker
{
SpinnakerCamera::SpinnakerCamera()
  : serial_(0)
  , system_(Spinnaker::System::GetInstance())
  , camList_(system_->GetCameras())
  , pCam_(static_cast<int>(NULL))  // Hack to suppress compiler warning. Spinnaker has only one contructor which takes
                                   // an int
  , camera_(static_cast<int>(NULL))
  , captureRunning_(false)
{
  unsigned int num_cameras = camList_.GetSize();
  LOG_INFO_S<<"[SpinnakerCamera]: Number of cameras detected: " << num_cameras;
}

SpinnakerCamera::~SpinnakerCamera()
{
  // @note ebretl Destructors of camList_ and system_ handle teardown
}

void SpinnakerCamera::setNewConfiguration(const camera_spinnaker::SpinnakerConfig& config, const uint32_t& level)
{
  // Check if camera is connected
  if (!pCam_)
  {
    SpinnakerCamera::connect();
  }

  // Activate mutex to prevent us from grabbing images during this time
  std::lock_guard<std::mutex> scopedLock(mutex_);

  if (level >= LEVEL_RECONFIGURE_STOP)
  {
    LOG_DEBUG_S<<"SpinnakerCamera::setNewConfiguration: Reconfigure Stop.";
    bool capture_was_running = captureRunning_;
    start();  // For some reason some params only work after aquisition has be started once.
    stop();
    camera_->setNewConfiguration(config, level);
    if (capture_was_running)
      start();
  }
  else
  {
    camera_->setNewConfiguration(config, level);
  }
}  // end setNewConfiguration

void SpinnakerCamera::setGain(const float& gain)
{
  if (camera_)
    camera_->setGain(gain);
}

int SpinnakerCamera::getHeightMax()
{
  if (camera_)
    return camera_->getHeightMax();
  else
    return 0;
}

int SpinnakerCamera::getWidthMax()
{
  if (camera_)
    return camera_->getWidthMax();
  else
    return 0;
}

Spinnaker::GenApi::CNodePtr SpinnakerCamera::readProperty(const Spinnaker::GenICam::gcstring property_name)
{
  if (camera_)
  {
    return camera_->readProperty(property_name);
  }
  else
  {
    return 0;
  }
}

void SpinnakerCamera::connect()
{
  if (!pCam_)
  {
    // If we have a specific camera to connect to (specified by a serial number)
    if (serial_ != 0)
    {
      const auto serial_string = std::to_string(serial_);

      try
      {
        pCam_ = camList_.GetBySerial(serial_string);
      }
      catch (const Spinnaker::Exception& e)
      {
        throw std::runtime_error("[SpinnakerCamera::connect] Could not find camera with serial number " +
                                 serial_string + ". Is that camera plugged in? Error: " + std::string(e.what()));
      }
    }
    else
    {
      // Connect to any camera (the first)
      try
      {
        pCam_ = camList_.GetByIndex(0);
      }
      catch (const Spinnaker::Exception& e)
      {
        throw std::runtime_error("[SpinnakerCamera::connect] Failed to get first connected camera. Error: " +
                                 std::string(e.what()));
      }
    }
    if (!pCam_ || !pCam_->IsValid())
    {
      throw std::runtime_error("[SpinnakerCamera::connect] Failed to obtain camera reference.");
    }

    try
    {
      // Check Device type and save serial for reconnecting
      Spinnaker::GenApi::INodeMap& genTLNodeMap = pCam_->GetTLDeviceNodeMap();

      if (serial_ == 0)
      {
        Spinnaker::GenApi::CStringPtr serial_ptr =
            static_cast<Spinnaker::GenApi::CStringPtr>(genTLNodeMap.GetNode("DeviceID"));
        if (IsAvailable(serial_ptr) && IsReadable(serial_ptr))
        {
          serial_ = atoi(serial_ptr->GetValue().c_str());
          LOG_INFO_S<<"[SpinnakerCamera::connect]: Using Serial: "<< serial_;
        }
        else
        {
          throw std::runtime_error("[SpinnakerCamera::connect]: Unable to determine serial number.");
        }
      }

      Spinnaker::GenApi::CEnumerationPtr device_type_ptr =
          static_cast<Spinnaker::GenApi::CEnumerationPtr>(genTLNodeMap.GetNode("DeviceType"));

      if (IsAvailable(device_type_ptr) && IsReadable(device_type_ptr))
      {
        LOG_INFO_S<<"[SpinnakerCamera::connect]: Detected device type: " << device_type_ptr->ToString();

        if (device_type_ptr->GetCurrentEntry() == device_type_ptr->GetEntryByName("U3V"))
        {
          Spinnaker::GenApi::CEnumerationPtr device_speed_ptr =
              static_cast<Spinnaker::GenApi::CEnumerationPtr>(genTLNodeMap.GetNode("DeviceCurrentSpeed"));
          if (IsAvailable(device_speed_ptr) && IsReadable(device_speed_ptr))
          {
            if (device_speed_ptr->GetCurrentEntry() != device_speed_ptr->GetEntryByName("SuperSpeed"))
            {
                LOG_ERROR_S<<"[SpinnakerCamera::connect]: U3V Device not running at Super-Speed. Check Cables! ";
            }
          }
        }
        // TODO(mhosmar): - check if interface is GigE and connect to GigE cam
      }
    }
    catch (const Spinnaker::Exception& e)
    {
      throw std::runtime_error("[SpinnakerCamera::connect] Failed to determine device info with error: " +
                               std::string(e.what()));
    }

    try
    {
      // Initialize Camera
      pCam_->Init();

      // Retrieve GenICam nodemap
      node_map_ = &pCam_->GetNodeMap();

      // detect model and set camera_ accordingly;
      Spinnaker::GenApi::CStringPtr model_name = node_map_->GetNode("DeviceModelName");
      std::string model_name_str(model_name->ToString());

      LOG_INFO_S<<"[SpinnakerCamera::connect]: Camera model name: %s", model_name_str.c_str();
      if (model_name_str.find("Blackfly S") != std::string::npos)
        camera_.reset(new Camera(node_map_));
      else if (model_name_str.find("Chameleon3") != std::string::npos)
        camera_.reset(new Cm3(node_map_));
      else if (model_name_str.find("Grasshopper3") != std::string::npos)
        camera_.reset(new Gh3(node_map_));
      else
      {
        camera_.reset(new Camera(node_map_));
        LOG_WARN_S<<"SpinnakerCamera::connect: Could not detect camera model name.";
      }

      // Configure chunk data - Enable Metadata
      // SpinnakerCamera::ConfigureChunkData(*node_map_);
    }
    catch (const Spinnaker::Exception& e)
    {
      throw std::runtime_error("[SpinnakerCamera::connect] Failed to connect to camera. Error: " +
                               std::string(e.what()));
    }
    catch (const std::runtime_error& e)
    {
      throw std::runtime_error("[SpinnakerCamera::connect] Failed to configure chunk data. Error: " +
                               std::string(e.what()));
    }
  }

  // TODO(mhosmar): Get camera info to check if camera is running in color or mono mode
  /*
  CameraInfo cInfo;
  error = cam_.GetCameraInfo(&cInfo);
  SpinnakerCamera::handleError("SpinnakerCamera::connect  Failed to get camera info.", error);
  isColor_ = cInfo.isColorCamera;
  */
}

void SpinnakerCamera::disconnect()
{
  std::lock_guard<std::mutex> scopedLock(mutex_);
  captureRunning_ = false;
  try
  {
    // Check if camera is connected
    if (pCam_)
    {
      pCam_->DeInit();
      pCam_ = static_cast<int>(NULL);
      camList_.RemoveBySerial(std::to_string(serial_));
    }
    Spinnaker::CameraList temp_list = system_->GetCameras();
    camList_.Append(temp_list);
  }
  catch (const Spinnaker::Exception& e)
  {
    throw std::runtime_error("[SpinnakerCamera::disconnect] Failed to disconnect camera with error: " +
                             std::string(e.what()));
  }
}

void SpinnakerCamera::start()
{
  try
  {
    // Check if camera is connected
    if (pCam_ && !captureRunning_)
    {
      // Start capturing images
      pCam_->BeginAcquisition();
      captureRunning_ = true;
    }
  }
  catch (const Spinnaker::Exception& e)
  {
    throw std::runtime_error("[SpinnakerCamera::start] Failed to start capture with error: " + std::string(e.what()));
  }
}

void SpinnakerCamera::stop()
{
  if (pCam_ && captureRunning_)
  {
    // Stop capturing images
    try
    {
      captureRunning_ = false;
      pCam_->EndAcquisition();
    }
    catch (const Spinnaker::Exception& e)
    {
      throw std::runtime_error("[SpinnakerCamera::stop] Failed to stop capture with error: " + std::string(e.what()));
    }
  }
}

void SpinnakerCamera::grabImage(base::samples::frame::Frame &frame, const std::string& frame_id)
{
  std::lock_guard<std::mutex> scopedLock(mutex_);

  // Check if Camera is connected and Running
  if (pCam_ && captureRunning_)
  {
    // Handle "Image Retrieval" Exception
    try
    {
        Spinnaker::ImagePtr image_ptr = pCam_->GetNextImage(timeout_);
        //std::string format(image_ptr->GetPixelFormatName());
        //std::printf("\033[100mImage format: %s \033[0m\n", format.c_str());

      if (image_ptr->IsIncomplete())
      {
        throw std::runtime_error("[SpinnakerCamera::grabImage] Image received from camera " + std::to_string(serial_) +
                                 " is incomplete.");
      }
      else
      {
        // Check the bits per pixel.
        size_t bits_per_pixel = image_ptr->GetBitsPerPixel();

        Spinnaker::GenApi::CEnumerationPtr color_filter_ptr =
            static_cast<Spinnaker::GenApi::CEnumerationPtr>(node_map_->GetNode("PixelColorFilter"));

        Spinnaker::GenICam::gcstring color_filter_str = color_filter_ptr->ToString();
        Spinnaker::GenICam::gcstring bayer_rg_str = "BayerRG";
        Spinnaker::GenICam::gcstring bayer_gr_str = "BayerGR";
        Spinnaker::GenICam::gcstring bayer_gb_str = "BayerGB";
        Spinnaker::GenICam::gcstring bayer_bg_str = "BayerBG";

        int stride = image_ptr->GetStride();
        //std::cout<<"bits_per_pixel:  "<<bits_per_pixel<<std::endl;
        //std::cout<<"stride:  "<<stride<<std::endl;
        //std::cout<<"color_filter_str:  "<<color_filter_str<<std::endl;

        /** Image size **/
        int width = image_ptr->GetWidth();
        int height = image_ptr->GetHeight();

        /** Image mode **/
        ::base::samples::frame::frame_mode_t mode = ::base::samples::frame::frame_mode_t::MODE_UNDEFINED;
        int color_depth = bits_per_pixel; //bits per pixel

        // if(isColor_ && bayer_format != NONE)
        if (color_filter_ptr->GetCurrentEntry() != color_filter_ptr->GetEntryByName("None"))
        {
            if (color_filter_str.compare(bayer_rg_str) == 0)
            {
                //BAYER_RGGB;
                mode = ::base::samples::frame::frame_mode_t::MODE_BAYER_RGGB;
            }
            else if (color_filter_str.compare(bayer_gr_str) == 0)
            {
                //BAYER_GRBG;
                mode = ::base::samples::frame::frame_mode_t::MODE_BAYER_GRBG;
            }
            else if (color_filter_str.compare(bayer_gb_str) == 0)
            {
                //BAYER_GBRG;
                mode = ::base::samples::frame::frame_mode_t::MODE_BAYER_GBRG;
            }
            else if (color_filter_str.compare(bayer_bg_str) == 0)
            {
                //BAYER_BGGR;
                mode = ::base::samples::frame::frame_mode_t::MODE_BAYER_BGGR;
            }
            else
            {
              throw std::runtime_error("[SpinnakerCamera::grabImage] Bayer format not recognized for 16-bit format.");
            }
        }
       else  // Mono camera or in pixel binned mode.
        {
            /** Mono grayscale **/
            if (bits_per_pixel == 16 || bits_per_pixel == 8)
            {
                mode = ::base::samples::frame::frame_mode_t::MODE_GRAYSCALE;
                color_depth = bits_per_pixel; //one channel 16 or 8 bits per channel
            }
            else if (bits_per_pixel == 24) // color BGR
            {
                mode = ::base::samples::frame::frame_mode_t::MODE_BGR;
                color_depth = 8; //3-channels, 8-bits per channel
            }
        }

        /** Init the image frame **/
        frame.init(width, height, color_depth, mode);

        /** Set Image Time Stamp **/
        frame.time = base::Time::now();//base::Time::fromMicroseconds(image_ptr->GetTimeStamp() * 1e-3);
        double time = image_ptr->GetTimeStamp() * 1e-9;
        frame.setAttribute<double>("CameraTimeStamp",time);
        frame.setAttribute<uint32_t>("SerialID", serial_);

        /** Fill the image **/
        memcpy(&(frame.image[0]), image_ptr->GetData(), frame.image.size());

      }// end else
    }
    catch (const Spinnaker::Exception& e)
    {
      throw std::runtime_error("[SpinnakerCamera::grabImage] Failed to retrieve buffer with error: " +
                               std::string(e.what()));
    }
  }
  else if (pCam_)
  {
    throw CameraNotRunningException("[SpinnakerCamera::grabImage] Camera is currently not running.  Please start "
                                    "capturing frames first.");
  }
  else
  {
    throw std::runtime_error("[SpinnakerCamera::grabImage] Not connected to the camera.");
  }
}  // end grabImage

void SpinnakerCamera::setTimeout(const double& timeout)
{
  timeout_ = static_cast<uint64_t>(std::round(timeout * 1000));
}
void SpinnakerCamera::setDesiredCamera(const uint32_t& id)
{
  serial_ = id;
}

void SpinnakerCamera::ConfigureChunkData(const Spinnaker::GenApi::INodeMap& nodeMap)
{
  LOG_INFO_S<<"*** CONFIGURING CHUNK DATA ***";
  try
  {
    // Activate chunk mode
    //
    // *** NOTES ***
    // Once enabled, chunk data will be available at the end of the payload
    // of every image captured until it is disabled. Chunk data can also be
    // retrieved from the nodemap.
    //
    Spinnaker::GenApi::CBooleanPtr ptrChunkModeActive = nodeMap.GetNode("ChunkModeActive");
    if (!Spinnaker::GenApi::IsAvailable(ptrChunkModeActive) || !Spinnaker::GenApi::IsWritable(ptrChunkModeActive))
    {
      throw std::runtime_error("Unable to activate chunk mode. Aborting...");
    }
    ptrChunkModeActive->SetValue(true);
    LOG_INFO_S<<"Chunk mode activated...";

    // Enable all types of chunk data
    //
    // *** NOTES ***
    // Enabling chunk data requires working with nodes: "ChunkSelector"
    // is an enumeration selector node and "ChunkEnable" is a boolean. It
    // requires retrieving the selector node (which is of enumeration node
    // type), selecting the entry of the chunk data to be enabled, retrieving
    // the corresponding boolean, and setting it to true.
    //
    // In this example, all chunk data is enabled, so these steps are
    // performed in a loop. Once this is complete, chunk mode still needs to
    // be activated.
    //
    Spinnaker::GenApi::NodeList_t entries;
    // Retrieve the selector node
    Spinnaker::GenApi::CEnumerationPtr ptrChunkSelector = nodeMap.GetNode("ChunkSelector");
    if (!Spinnaker::GenApi::IsAvailable(ptrChunkSelector) || !Spinnaker::GenApi::IsReadable(ptrChunkSelector))
    {
      throw std::runtime_error("Unable to retrieve chunk selector. Aborting...");
    }
    // Retrieve entries
    ptrChunkSelector->GetEntries(entries);

    LOG_INFO_S<<"Enabling entries...";

    for (unsigned int i = 0; i < entries.size(); i++)
    {
      // Select entry to be enabled
      Spinnaker::GenApi::CEnumEntryPtr ptrChunkSelectorEntry = entries.at(i);
      // Go to next node if problem occurs
      if (!Spinnaker::GenApi::IsAvailable(ptrChunkSelectorEntry) ||
          !Spinnaker::GenApi::IsReadable(ptrChunkSelectorEntry))
      {
        continue;
      }
      ptrChunkSelector->SetIntValue(ptrChunkSelectorEntry->GetValue());

      LOG_INFO_S<<"\t" << ptrChunkSelectorEntry->GetSymbolic() << ": ";
      // Retrieve corresponding boolean
      Spinnaker::GenApi::CBooleanPtr ptrChunkEnable = nodeMap.GetNode("ChunkEnable");
      // Enable the boolean, thus enabling the corresponding chunk data
      if (!Spinnaker::GenApi::IsAvailable(ptrChunkEnable))
      {
        LOG_INFO_S<<"Node not available";
      }
      else if (ptrChunkEnable->GetValue())
      {
        LOG_INFO_S<<"Enabled";
      }
      else if (Spinnaker::GenApi::IsWritable(ptrChunkEnable))
      {
        ptrChunkEnable->SetValue(true);
        LOG_INFO_S<<"Enabled";
      }
      else
      {
        LOG_INFO_S<<"Node not writable";
      }
    }
  }
  catch (const Spinnaker::Exception& e)
  {
    throw std::runtime_error(e.what());
  }
}
}  // namespace camera_spinnaker
