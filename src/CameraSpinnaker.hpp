#ifndef __CAMERA_SPINNAKER_HPP__
#define __CAMERA_SPINNAKER_HPP__

#include <camera_spinnaker/SpinnakerConfig.h>

// Spinnaker SDK
#include <Spinnaker.h>
#include <SpinGenApi/SpinnakerGenApi.h>

#include <camera_interface/CamInterface.h>
#include <boost/thread.hpp>

namespace Spinnaker
{
    class CInstantCamera;
    class CGrabResultPtr;
}

namespace camera
{
    class CameraSpinnaker: public CamInterface
    {
        friend class ImageEventHandler;

    private:
        CamInfo cam_info;
        Spinnaker::CameraPtr camera;

        Spinnaker::CInstantCamera *camera;
        std::vector<base::samples::frame::Frame> buffer;
        int buffer_idx;
        int buffer_idx2;
        boost::mutex mutex;

	void (*pcallback_function_)(const void* p);
	void *pass_through_pointer_;

    protected:
        void queueData(const Spinnaker::CGrabResultPtr& ptrGrabResult);

    public:
        CameraSpinnaker();
        ~CameraSpinnaker();

        int listCameras(std::vector<CamInfo>&cam_infos)const;
        bool open(const CamInfo &cam,const AccessMode mode);
        bool open(const std::string &serial_number,const AccessMode mode);
        bool open2(const std::string &unique_name,const AccessMode mode);
        bool openFirst();

        bool isOpen()const;
        bool close();
        const CamInfo *getCameraInfo()const;

        bool grab(const GrabMode mode, const int buffer_len);
        bool retrieveFrame(base::samples::frame::Frame &frame,const int timeout);
        bool setFrameSettings(const base::samples::frame::frame_size_t size,
                              const base::samples::frame::frame_mode_t mode,
                              const  uint8_t color_depth,
                              const bool resize_frames);
        bool isFrameAvailable();
        int skipFrames();

        bool setAttrib(const int_attrib::CamAttrib attrib,const int value);
        bool setAttrib(const double_attrib::CamAttrib attrib,const double value);
        bool setAttrib(const enum_attrib::CamAttrib attrib);
        bool setAttrib(const str_attrib::CamAttrib attrib,const std::string &string);

        bool isAttribAvail(const int_attrib::CamAttrib attrib);
        bool isAttribAvail(const str_attrib::CamAttrib attrib);
        bool isAttribAvail(const double_attrib::CamAttrib attrib);
        bool isAttribAvail(const enum_attrib::CamAttrib attrib);

        int getAttrib(const int_attrib::CamAttrib attrib);
        double getAttrib(const double_attrib::CamAttrib attrib);
        std::string getAttrib(const str_attrib::CamAttrib attrib);
        bool isAttribSet(const enum_attrib::CamAttrib attrib);

        bool getFrameSettings(base::samples::frame::frame_size_t &size,
			      base::samples::frame::frame_mode_t &mode,
			      uint8_t &color_depth);

        bool triggerFrame();
	
	bool setCallbackFcn(void (*pcallback_function)(const void* p),void *p);
	void callUserCallbackFcn()const;
        void saveConfiguration(uint8_t index);
        void loadConfiguration(uint8_t index);
	
	void getRange(const double_attrib::CamAttrib attrib,double &dmin,double &dmax);
	void getRange(const int_attrib::CamAttrib attrib,int &imin,int &imax);

    void setOverlapMode(bool value);
    };
}

#endif	

