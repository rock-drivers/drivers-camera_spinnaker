#include <boost/test/unit_test.hpp>
#include <camera_spinnaker/SpinnakerCamera.h>

/** Frame helper **/
#include <frame_helper/FrameHelper.h>
#include <opencv2/opencv.hpp>

using namespace camera_spinnaker;

// Spinnaker include shoul be in system /opt/spinnaker/include
#include <Spinnaker.h>

BOOST_AUTO_TEST_CASE(test_spinnaker_lib)
{

    Spinnaker::SystemPtr system = Spinnaker::System::GetInstance();

    Spinnaker::InterfaceList interfaceList = system->GetInterfaces();
    unsigned int numInterfaces = interfaceList.GetSize();
    std::printf("\033[93m[Spinnaker] Number of interfaces detected: %d \n", numInterfaces);

    Spinnaker::CameraList camList = system->GetCameras();
    unsigned int numCameras = camList.GetSize();

    std::printf("\033[93m[Spinnaker] # of connected cameras: %d \n", numCameras);

    // Finish if there are no cameras
    if (numCameras == 0)
    {
        std::printf("\033[91mNO Cameras Connected! \n\n");
        // Clear camera list before releasing system
        camList.Clear();
        interfaceList.Clear();
        // Release system
        system->ReleaseInstance();

        return;
    }
    else
    {
        BOOST_CHECK(true); //just checking passing from this line
        for (unsigned int i = 0; i < numCameras; i++)
        {
            Spinnaker::CameraPtr pCam = camList[i];
            Spinnaker::GenApi::INodeMap& nodeMapTLDevice = pCam->GetTLDeviceNodeMap();
            Spinnaker::GenApi::CStringPtr ptrDeviceSerialNumber = nodeMapTLDevice.GetNode("DeviceSerialNumber");
            if (Spinnaker::GenApi::IsAvailable(ptrDeviceSerialNumber) && Spinnaker::GenApi::IsReadable(ptrDeviceSerialNumber))
            {
                std::cout << "\033[92m[" << i << "]\t" << ptrDeviceSerialNumber->ToString() << std::endl;
            }
        }
    }
    camList.Clear();
    interfaceList.Clear();
    system->ReleaseInstance();

}

BOOST_AUTO_TEST_CASE(test_spinnaker_driver)
{
    camera_spinnaker::SpinnakerCamera spinnaker;// Instance of the SpinnakerCamera library, used to interface with the hardware.
    camera_spinnaker::SpinnakerConfig config;// configuration
    config.acquisition_frame_rate = 1.0;

    Spinnaker::SystemPtr system = Spinnaker::System::GetInstance();
    Spinnaker::InterfaceList interfaceList = system->GetInterfaces();
    Spinnaker::CameraList camList = system->GetCameras();
    unsigned int num_cameras = camList.GetSize();

    if (num_cameras > 0)
    {
        BOOST_CHECK(true); //just checking passing from this line
        std::printf("\033[93m[Spinnaker] # of connected cameras: %d \n", num_cameras);
        Spinnaker::CameraPtr pCam = camList[0];
        Spinnaker::GenApi::INodeMap& nodeMapTLDevice = pCam->GetTLDeviceNodeMap();
        Spinnaker::GenApi::CStringPtr ptrDeviceSerialNumber = nodeMapTLDevice.GetNode("DeviceSerialNumber");
        if (Spinnaker::GenApi::IsAvailable(ptrDeviceSerialNumber) && Spinnaker::GenApi::IsReadable(ptrDeviceSerialNumber))
        {
            std::cout << "\033[92m[" << 0 << "]\t" << std::string(ptrDeviceSerialNumber->ToString()) << std::endl;
            spinnaker.setDesiredCamera((uint32_t) std::stoi(std::string(ptrDeviceSerialNumber->ToString())));
            BOOST_TEST_MESSAGE("\033[93mCONNECTING TO THE CAMERA.. ");
            spinnaker.connect();
            BOOST_TEST_MESSAGE("\033[93mCONNECTED TO THE CAMERA.. ");
            spinnaker.setNewConfiguration(config, SpinnakerCamera::LEVEL_RECONFIGURE_STOP);
            try
            {
                double timeout = 1.0; BOOST_TEST_MESSAGE("Setting timeout to: "<< timeout<<"[seconds]");
                spinnaker.setTimeout(timeout);
            }
            catch (const std::runtime_error& e)
            {
                std::cout<<"[ERROR] "<<e.what()<<std::endl;
            }

            BOOST_TEST_MESSAGE("\033[93mSTARTING THE CAMERA.. ");
            spinnaker.start();
            BOOST_TEST_MESSAGE("\033[93mSTARTED THE CAMERA.. ");

            BOOST_TEST_MESSAGE("\033[93mGRAB AN IMAGE FROM CAMERA WITH SERIAL: "<< spinnaker.getSerial());
            base::samples::frame::Frame img;
            for (int i=0; i<10; ++i)
            {
                spinnaker.grabImage(img, "image_test");
                std::cout<<"[time] "<<img.time.toString()<<" img size "<<img.size.width<<" x "<<img.size.height<<" px_size: "<<img.pixel_size <<" data size: "<<img.image.size()
                     <<" img mode "<<img.frame_mode<<std::endl;
                cv::Mat img_mat = frame_helper::FrameHelper::convertToCvMat(img);
                cv::imwrite("/tmp/img_mat_"+std::to_string(i)+".png", img_mat);
            }

            BOOST_TEST_MESSAGE("\033[93mSTOPPING THE CAMERA.. ");
            spinnaker.stop();
            BOOST_TEST_MESSAGE("\033[93mSTOPPED THE CAMERA.. ");

            BOOST_TEST_MESSAGE("\033[93mDISCONNECTING FROM THE CAMERA.. ");
            spinnaker.disconnect();
            BOOST_TEST_MESSAGE("\033[93mDISCONNECTED FROM THE CAMERA..\033[0m");
        }
    }

    camList.Clear();
    interfaceList.Clear();
    system->ReleaseInstance();
}
