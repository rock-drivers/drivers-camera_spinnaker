#include "CameraSpinnaker.hpp"

#include <sstream>
#include <sys/types.h>
#include <limits.h>

#include <unistd.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <arpa/inet.h>

using namespace base::samples::frame;
using namespace camera;

// HELPER MEHTODS //
///////////////////////////////////////////
CamInfo toCamInfo(const Spinnaker::GenApi::INodeMap &node_device)
{
    Spinnaker::GenApi::CStringPtr ptrDeviceSerialNumber = node_device.GetNode("DeviceSerialNumber");
    CamInfo info;
    info.serial_string = std::string(ptrDeviceSerialNumber->ToString());
    info.display_name = std::string(node_devide.GetDeviceName());
    info.unique_id = std::stoi(std::string(ptrDeviceSerialNumber->ToString()));
    return info;
}
///////////////////////////////////////////

// Event Handler
namespace camera
{
    class ImageEventHandler: public CImageEventHandler
    {
        private:
            CameraSpinnaker &spinnaker_cam;

        public:
            ImageEventHandler(CameraSpinnaker &cam)
                :spinnaker_cam(cam){}
            
            void OnImageEvent(ImagePtr image)
            {
                spinnaker_cam.queueData(ptrGrabResult);
            }
    };
}

CameraPylon::CameraPylon():camera(NULL),buffer_idx(0),buffer_idx2(0),
    pcallback_function_(NULL),pass_through_pointer_(NULL)
{
    Pylon::PylonInitialize();
}

CameraPylon::~CameraPylon()
{
    // this is reference counted
    // for each PylonInitialize, PylonTerminate must be called
    close();
    Pylon::PylonTerminate();
}

int CameraPylon::listCameras(std::vector<CamInfo>&cam_infos)const
{
    CTlFactory &factory = CTlFactory::GetInstance();
    Pylon::DeviceInfoList_t list;
    int count  = factory.EnumerateDevices(list);

    cam_infos.reserve(count);
    Pylon::DeviceInfoList_t::const_iterator iter = list.begin();
    for(;iter != list.end();++iter)
        cam_infos.push_back(toCamInfo(*iter));
    return count;
}

bool CameraPylon::setCallbackFcn(void (*pcallback_function)(const void *p),void *p)
{
    pcallback_function_ = pcallback_function;
    pass_through_pointer_ = p;
    return true;
}

//this function is thread safe as long as pcallback_function_ is not set
//to NULL
void CameraPylon::callUserCallbackFcn()const
{
    if(pcallback_function_)
        pcallback_function_(pass_through_pointer_);		
}

const CamInfo *CameraPylon::getCameraInfo()const
{
    if(isOpen())
        return &cam_info;
    return NULL;
}

bool CameraPylon::openFirst()
{
    std::vector<CamInfo> infos;
    listCameras(infos);
    if(infos.empty())
        return false;
    return open(infos.front(),Master);
}

bool CameraPylon::open(const std::string &serial_nr,const AccessMode mode)
{

    CamInfo cam;
    cam.serial_string = serial_nr;
    return open(cam,mode);
}

bool CameraPylon::open2(const std::string &unique_name,const AccessMode mode)
{
    if(isOpen())
        close();
    try
    {
        CTlFactory &factory = CTlFactory::GetInstance();
        Pylon::IPylonDevice *device = factory.CreateDevice(CDeviceInfo().SetFullName(unique_name.c_str()));
        if(!device)
            return false;
        camera = new CInstantCamera(device);

        // open camera 
        // low level API must be used to support other modes than master
        AccessModeSet amode;
        if(mode != Master)
            throw std::runtime_error("CamerPylon::open: only Master mode is supported");

        camera->Open();
        if(!camera->IsOpen())
        {
            close();
            throw std::runtime_error("CamerPylon::open: Camera was found but cannot be opened");
        }

        // cache cam info
        cam_info = toCamInfo(camera->GetDeviceInfo());

        // register event handler
        camera->RegisterImageEventHandler(new ImageEventHandler(*this),RegistrationMode_Append,Cleanup_Delete);
    }
    catch (const GenericException & e)
    {
        std::cerr << "cannot find camera: " << unique_name << std::endl;
        std::cerr << "the following cameras are available: " << std::endl;

        std::vector<CamInfo> cam_infos;
        listCameras(cam_infos);
        std::vector<CamInfo>::const_iterator iter = cam_infos.begin();
        for(;iter != cam_infos.end();++iter)
            std::cerr << " serial nr: " << iter->serial_string << " unique name: " << iter->display_name << std::endl;
        throw std::runtime_error(std::string("Failed to open camera: ") + e.GetDescription());
    }

    return camera != NULL;
}

bool CameraPylon::open(const CamInfo &cam,const AccessMode mode)
{
    std::string full_name;
    if(!cam.display_name.empty())
        full_name = cam.display_name;
    else
    {
        CamInfo cam2;
        if(findCamera(cam,cam2))
            full_name = cam2.display_name;
        else
            full_name = cam.unique_id;
    }
    return open2(full_name,mode);
}

bool CameraPylon::isOpen()const
{
    return camera != NULL;
}

bool CameraPylon::grab(const GrabMode mode,const int buffer_len)
{
    if(!isOpen())
        throw std::runtime_error("Camera is not opene");

    if(buffer_len == 0)
        throw std::runtime_error("Buffer len of zero is not supported");

    // prepare buffer
    if(!camera->IsGrabbing())
    {
        base::samples::frame::frame_size_t size;
        base::samples::frame::frame_mode_t fmode;
        uint8_t color_depth;
        buffer.resize(buffer_len);
        getFrameSettings(size,fmode,color_depth);
        std::vector<base::samples::frame::Frame>::iterator iter = buffer.begin();
        for(;iter != buffer.end();++iter)
            iter->init(size.width,size.height,color_depth*8,fmode,-1);
        buffer_idx = 0;
        buffer_idx2 = 0;
    }
    else if(mode != Stop)
        throw std::runtime_error("Camera is already grabbing");

    switch(mode)
    {
    case Stop:
        if(camera->IsGrabbing())
            camera->StopGrabbing();
        break;
    case SingleFrame:
        camera->RegisterConfiguration( new CSoftwareTriggerConfiguration, RegistrationMode_ReplaceAll, Cleanup_Delete);
        camera->StartGrabbing(1,GrabStrategy_OneByOne,GrabLoop_ProvidedByInstantCamera);
        break;
    case Continuously:
        // prepare buffer
        camera->RegisterConfiguration(new CAcquireContinuousConfiguration, RegistrationMode_ReplaceAll,Cleanup_Delete);
        camera->StartGrabbing(GrabStrategy_OneByOne,GrabLoop_ProvidedByInstantCamera);
        break;
    case MultiFrame:
        camera->RegisterConfiguration( new CSoftwareTriggerConfiguration, RegistrationMode_ReplaceAll, Cleanup_Delete);
        camera->StartGrabbing(GrabStrategy_OneByOne,GrabLoop_ProvidedByInstantCamera);
    default:
        throw std::runtime_error("Grap mode is not supported");
    }
    return true;
}

bool CameraPylon::isFrameAvailable()
{
    return !buffer.empty() && buffer[buffer_idx2].frame_status != base::samples::frame::STATUS_EMPTY;
}

int CameraPylon::skipFrames()
{
    boost::lock_guard<boost::mutex> guard(mutex);
    buffer_idx = 0;
    buffer_idx2 = 0;
    int count = 0;

    std::vector<base::samples::frame::Frame>::iterator iter = buffer.begin();
    for(;iter != buffer.end();++iter)
    {
        if(iter->frame_status != base::samples::frame::STATUS_EMPTY)
        {
            iter->frame_status = base::samples::frame::STATUS_EMPTY;
            ++count;
        }
    }
    return count;
}

bool CameraPylon::retrieveFrame(Frame &frame,const int timeout)
{
    base::Time time = base::Time::now();
    while(!isFrameAvailable())
    {
        if((base::Time::now()-time).toMilliseconds() >= timeout)
            return false;
        usleep(100);
    }

    const base::samples::frame::Frame &tframe = buffer[0];
    frame.init(tframe.size.width,tframe.size.height,tframe.data_depth,tframe.frame_mode,-1);
    frame.frame_status = base::samples::frame::STATUS_EMPTY;

    boost::lock_guard<boost::mutex> guard(mutex);
    buffer[buffer_idx2].swap(frame);

    ++buffer_idx2;
    if(buffer_idx2 >= int(buffer.size()))
        buffer_idx2 = 0;
    return true;
}

bool CameraPylon::close()
{
    if(camera)
    {
        if(camera-isOpen())
            camera->Close();
        delete camera;
        camera = NULL;
        return true;
    }
    return false;
}

bool CameraPylon::setFrameSettings(const frame_size_t size,
        const frame_mode_t mode,
        const uint8_t color_depth,
        const bool resize_frames)
{
    if(!isOpen())
        throw std::runtime_error("Camera is not opene");

    try
    {
        GenApi::INodeMap &control= camera->GetNodeMap();
        GenApi::CIntegerPtr width = control.GetNode("Width");
        GenApi::CIntegerPtr height = control.GetNode("Height");
        if(!GenApi::IsWritable(width) || !GenApi::IsWritable(height))
            throw std::runtime_error("cannot write image width or height");

        if(width->GetMax() < size.width)
        {
            std::stringstream stream;
            stream << "Cannot set image width. Width exceeded physical image size of " << width->GetMax();
            throw std::runtime_error(stream.str());
        }
        if(height->GetMax() < size.height)
        {
            std::stringstream stream;
            stream << "Cannot set image height. Height exceeded physical image size of " << height->GetMax();
            throw std::runtime_error(stream.str());
        }
        width->SetValue(size.width);
        height->SetValue(size.height);

        std::string format = convertPixelFormatToStr(mode, color_depth);
        GenApi::CEnumerationPtr(control.GetNode("PixelFormat"))->FromString(format.c_str());
    }
    catch(const GenericException & e)
    {
        throw std::runtime_error(std::string("Failed to set frame settings: ") + e.GetDescription());
    }
    return true;
}

bool CameraPylon::setAttrib(const int_attrib::CamAttrib attrib,
        const int value)
{
    if(!isOpen())
        throw std::runtime_error("Camera is not open");
    try
    {
        GenApi::INodeMap &control= camera->GetNodeMap();

        std::string str;
        attribToStr(attrib,str);
        GenApi::CIntegerPtr val = control.GetNode(str.c_str());
        if(!GenApi::IsAvailable(val))
            throw std::runtime_error("attrib " + str + " is not available");
        if(!GenApi::IsWritable(val))
            throw std::runtime_error("attrib " + str + " is not writable");

        val->SetValue(value);
    }
    catch(const GenericException & e)
    {
        throw std::runtime_error(std::string("Failed to set Attribute: ") + e.GetDescription());
    }

    return true;
}

void CameraPylon::setOverlapMode(bool value)
{
    try
    {
        GenApi::INodeMap &control= camera->GetNodeMap();
        GenApi::CEnumerationPtr val = control.GetNode("OverlapMode");
        if(!GenApi::IsAvailable(val))
            throw std::runtime_error("attrib OverlapMode is not available");
        if(!GenApi::IsWritable(val))
            throw std::runtime_error("attrib OverlapMode is not writable");
        if(value)
            val->FromString("On");
        else
            val->FromString("Off");
    }
    catch(const GenericException & e)
    {
        throw std::runtime_error(std::string("Failed to set Attribute: ") + e.GetDescription());
    }
}

bool CameraPylon::setAttrib(const double_attrib::CamAttrib attrib,
        const double value)
{
    if(!isOpen())
        throw std::runtime_error("Camera is not open");

    try
    {
        GenApi::INodeMap &control= camera->GetNodeMap();

        std::string str;
        attribToStr(attrib,str);
        if(attrib == double_attrib::FrameRate)
        {
            GenApi::CIntegerPtr val = control.GetNode(str.c_str());
            if(!GenApi::IsAvailable(val))
                throw std::runtime_error("attrib " + str + " is not available");
            if(!GenApi::IsWritable(val))
                throw std::runtime_error("attrib " + str + " is not writable");
            val->SetValue(1e6/value);
            if(value < 20)
                setOverlapMode(false);
            else
                setOverlapMode(true);
        }
        else
        {
            GenApi::CFloatPtr val = control.GetNode(str.c_str());
            if(!GenApi::IsAvailable(val))
                throw std::runtime_error("attrib " + str + " is not available");
            if(!GenApi::IsWritable(val))
                throw std::runtime_error("attrib " + str + " is not writable");
            val->SetValue(value);
        }
    }
    catch(const GenericException & e)
    {
        throw std::runtime_error(std::string("Failed to set Attribute: ") + e.GetDescription());
    }

    return true;
}

bool CameraPylon::setAttrib(const enum_attrib::CamAttrib attrib)
{
    if(!isOpen())
        throw std::runtime_error("Camera is not open");

    try
    {
        GenApi::INodeMap &control= camera->GetNodeMap();

        std::string str,str2;
        attribToStr(attrib,str,str2);
        GenApi::CEnumerationPtr val = control.GetNode(str.c_str());
        if(!GenApi::IsAvailable(val))
            throw std::runtime_error("attrib " + str + " is not available");
        if(!GenApi::IsWritable(val))
            throw std::runtime_error("attrib " + str + " is not writable");

        val->FromString(str2.c_str());

        // some more adjustments
        switch(attrib)
        {
        case enum_attrib::FrameStartTriggerModeToFreerun:
            setAttrib(double_attrib::FrameRate,10000);
            break;
        case enum_attrib::FrameStartTriggerModeToSyncIn1:
        case enum_attrib::FrameStartTriggerModeToSyncIn2:
        case enum_attrib::FrameStartTriggerModeToSoftware:
            {
                GenApi::CEnumerationPtr val2 = control.GetNode("TriggerSource");
                if(!GenApi::IsAvailable(val2))
                    throw std::runtime_error("attrib TriggerSource is not available");
                if(!GenApi::IsWritable(val2))
                    throw std::runtime_error("attrib TriggerSource is not writable");
                switch(attrib)
                {
                case enum_attrib::FrameStartTriggerModeToSyncIn1:
                    val2->FromString("Line1");
                    break;
                case enum_attrib::FrameStartTriggerModeToSyncIn2:
                    val2->FromString("Line2");
                    break;
                case enum_attrib::FrameStartTriggerModeToSoftware:
                    val2->FromString("Software");
                    break;
                default:
                    break;
                }
            }
            break;
        default:
            break;
        }
    }
    catch(const GenericException & e)
    {
        throw std::runtime_error(std::string("Failed to set Attribute: ") + e.GetDescription());
    }

    return true;
}

bool CameraPylon::setAttrib(const str_attrib::CamAttrib attrib,
        const std::string &string)
{
    if(!isOpen())
        throw std::runtime_error("Camera is not open");

    try
    {
        GenApi::INodeMap &control= camera->GetNodeMap();

        std::string str;
        attribToStr(attrib,str);
        GenApi::CStringPtr val = control.GetNode(str.c_str());
        if(!GenApi::IsAvailable(val))
            throw std::runtime_error("attrib " + str + " is not available");
        if(!GenApi::IsWritable(val))
            throw std::runtime_error("attrib " + str + " is not writable");

        *val = string.c_str();
    }
    catch(const GenericException & e)
    {
        throw std::runtime_error(std::string("Failed to set Attribute: ") + e.GetDescription());
    }
    return true;
}

bool CameraPylon::isAttribAvail(const int_attrib::CamAttrib attrib)
{
    if(!isOpen())
        throw std::runtime_error("Camera is not open");

    bool bval =false;
    try
    {
        GenApi::INodeMap &control= camera->GetNodeMap();

        std::string str;
        attribToStr(attrib,str);
        GenApi::CIntegerPtr val = control.GetNode(str.c_str());
        bval = GenApi::IsAvailable(val);
    }
    catch(const GenericException & e)
    {}
    catch(const std::runtime_error& e)
    {}
    return bval;
}

bool CameraPylon::isAttribAvail(const double_attrib::CamAttrib attrib)
{
    if(!isOpen())
        throw std::runtime_error("Camera is not open");

    bool bval=false;
    try
    {
        GenApi::INodeMap &control= camera->GetNodeMap();

        std::string str;
        attribToStr(attrib,str);
        if(attrib == double_attrib::FrameRate)
        {
            GenApi::CIntegerPtr val = control.GetNode(str.c_str());
            bval = GenApi::IsAvailable(val);
        }
        else
        {
            GenApi::CFloatPtr val = control.GetNode(str.c_str());
            bval = GenApi::IsAvailable(val);
        }
    }
    catch(const GenericException & e)
    { }
    catch(const std::runtime_error& e)
    {}
    return bval;
}

bool CameraPylon::isAttribAvail(const str_attrib::CamAttrib attrib)
{
    if(!isOpen())
        throw std::runtime_error("Camera is not open");

    bool bval =false;
    try
    {
        GenApi::INodeMap &control= camera->GetNodeMap();

        std::string str;
        attribToStr(attrib,str);
        GenApi::CStringPtr val = control.GetNode(str.c_str());
        bval = GenApi::IsAvailable(val);
    }
    catch(const GenericException & e)
    { }
    catch(const std::runtime_error& e)
    {}
    return bval;
}

bool CameraPylon::isAttribAvail(const enum_attrib::CamAttrib attrib)
{
    if(!isOpen())
        throw std::runtime_error("Camera is not open");

    bool bval=false;
    try
    {
        GenApi::INodeMap &control= camera->GetNodeMap();

        std::string str,str2;
        attribToStr(attrib,str,str2);
        GenApi::CEnumerationPtr val = control.GetNode(str.c_str());
        if(!GenApi::IsAvailable(val))
            return false;

        GenApi::IEnumEntry *entry = val->GetEntryByName(str2.c_str());
        if(!entry)
            return false;
        bval = GenApi::IsAvailable(entry);
    }
    catch(const GenericException & e)
    { }
    catch(const std::runtime_error& e)
    {}
    return bval;
}

int CameraPylon::getAttrib(const int_attrib::CamAttrib attrib)
{
    if(!isOpen())
        throw std::runtime_error("Camera is not open");
    int ival;
    try
    {
        GenApi::INodeMap &control= camera->GetNodeMap();

        std::string str;
        attribToStr(attrib,str);
        GenApi::CIntegerPtr val = control.GetNode(str.c_str());
        if(!GenApi::IsAvailable(val))
            throw std::runtime_error("attrib " + str + " is not available");
        ival = val->GetValue();
    }
    catch(const GenericException & e)
    {
        throw std::runtime_error(std::string("Failed to set Attribute: ") + e.GetDescription());
    }
    return ival;
}

double CameraPylon::getAttrib(const double_attrib::CamAttrib attrib)
{
    if(!isOpen())
        throw std::runtime_error("Camera is not open");

    double dval;
    try
    {
        GenApi::INodeMap &control= camera->GetNodeMap();
        std::string str;
        attribToStr(attrib,str);
        if(attrib == double_attrib::FrameRate)
        {
            GenApi::CIntegerPtr val = control.GetNode(str.c_str());
            if(!GenApi::IsAvailable(val))
                throw std::runtime_error("attrib " + str + " is not available");
            dval = 1e6/double(val->GetValue());
        }
        else
        {
            GenApi::CFloatPtr val = control.GetNode(str.c_str());
            if(!GenApi::IsAvailable(val))
                throw std::runtime_error("attrib " + str + " is not available");
            dval = val->GetValue();
        }

    }
    catch(const GenericException & e)
    {
        throw std::runtime_error(std::string("Failed to set Attribute: ") + e.GetDescription());
    }
    return dval;
}

std::string CameraPylon::getAttrib(const str_attrib::CamAttrib attrib)
{
    if(!isOpen())
        throw std::runtime_error("Camera is not open");

    std::string str;
    try
    {
        GenApi::INodeMap &control= camera->GetNodeMap();

        std::string str;
        attribToStr(attrib,str);
        GenApi::CStringPtr val = control.GetNode(str.c_str());
        if(!GenApi::IsAvailable(val))
            throw std::runtime_error("attrib " + str + " is not available");

        str = std::string(val->operator*());
    }
    catch(const GenericException & e)
    {
        throw std::runtime_error(std::string("Failed to set Attribute: ") + e.GetDescription());
    }
    return str;
}

bool CameraPylon::isAttribSet(const enum_attrib::CamAttrib attrib)
{
    if(!isOpen())
        throw std::runtime_error("Camera is not open");

    bool bval;
    try
    {
        GenApi::INodeMap &control= camera->GetNodeMap();

        std::string str,str2;
        attribToStr(attrib,str,str2);
        GenApi::CEnumerationPtr val = control.GetNode(str.c_str());
        if(!GenApi::IsAvailable(val))
            return false;

        GenApi::IEnumEntry *entry = val->GetEntryByName(str2.c_str());
        if(!entry)
            return false;
        if(!GenApi::IsAvailable(entry))
            return false;
        bval = entry == val->GetCurrentEntry();
    }
    catch(const GenericException & e)
    {
        throw std::runtime_error(std::string("Failed to set Attribute: ") + e.GetDescription());
    }
    return bval;
}

void CameraPylon::getRange(const double_attrib::CamAttrib attrib,double &dmin,double &dmax)
{
    if(!isOpen())
        throw std::runtime_error("Camera is not open");
    try
    {
        GenApi::INodeMap &control= camera->GetNodeMap();
        std::string str;
        attribToStr(attrib,str);

        GenApi::CIntegerPtr val = control.GetNode(str.c_str());
        dmin = val->GetMin();
        dmax= val->GetMax();
    }
    catch(const GenericException & e)
    {
        throw std::runtime_error(std::string("Failed to set Attribute: ") + e.GetDescription());
    }
}

void CameraPylon::getRange(const int_attrib::CamAttrib attrib,int &imin,int &imax)
{
    if(!isOpen())
        throw std::runtime_error("Camera is not open");
    try
    {
        GenApi::INodeMap &control= camera->GetNodeMap();
        std::string str;
        attribToStr(attrib,str);

        GenApi::CIntegerPtr val = control.GetNode(str.c_str());
        imin = val->GetMin();
        imax= val->GetMax();
    }
    catch(const GenericException & e)
    {
        throw std::runtime_error(std::string("Failed to set Attribute: ") + e.GetDescription());
    }
}

bool CameraPylon::triggerFrame()
{
    if(!isOpen())
        throw std::runtime_error("Camera is not open");

    try
    {
        camera->ExecuteSoftwareTrigger();
    }
    catch(const GenericException & e)
    {
        throw std::runtime_error(std::string("Failed to set Attribute: ") + e.GetDescription());
    }
    return true;
}

bool CameraPylon::getFrameSettings(frame_size_t &size,
        frame_mode_t &mode,
        uint8_t &color_depth)
{
    if(!isOpen())
        throw std::runtime_error("Camera is not opene");

    try
    {
        GenApi::INodeMap &control= camera->GetNodeMap();
        GenApi::CIntegerPtr width = control.GetNode("Width");
        GenApi::CIntegerPtr height = control.GetNode("Height");

        size.width = width->GetValue();
        size.height= height->GetValue();

        std::string format(GenApi::CEnumerationPtr(control.GetNode("PixelFormat"))->GetCurrentEntry()->GetSymbolic());
        convertStrToPixelFormat(format,mode,color_depth);
    }
    catch(const GenericException & e)
    {
        throw std::runtime_error(std::string("Failed to set Attribute: ") + e.GetDescription());
    }
    return true;
}

void CameraPylon::saveConfiguration(uint8_t index)
{
}

void CameraPylon::loadConfiguration(uint8_t index)
{
}

// this called from a different thread
void CameraPylon::queueData(const CGrabResultPtr& result)
{
    if(!result->GrabSucceeded())
    {
        std::cerr << "Grab does no succeeded: " << result->GetErrorDescription() << std::endl;
        return;
    }

    {
        boost::lock_guard<boost::mutex> guard(mutex);
        base::samples::frame::Frame &frame = buffer[buffer_idx];

        // do some simple checking
        if(frame.image.size() != result->GetImageSize())
        {
            std::cerr << "Grab does no succeeded: size mismatch, got  " << result->GetImageSize() << " expected " << frame.image.size()  << std::endl;
            return;
        }
        if(frame.frame_status != base::samples::frame::STATUS_EMPTY)
        {
            std::cerr << "buffer underflow: skipping image"  << std::endl;
            if(buffer_idx == buffer_idx2)
            {
                ++buffer_idx2;
                if(buffer_idx2 >= int(buffer.size()))
                    buffer_idx2 = 0;
            }
        }

        // copy data
        memcpy(&frame.image.front(),result->GetBuffer(),frame.image.size());

        // set data
        frame.frame_status = base::samples::frame::STATUS_VALID;
        uint64_t time = result->GetTimeStamp();
        frame.time = base::Time::now();
        frame.setAttribute<uint64_t>("CameraTimeStamp",time);
        frame.setAttribute<uint64_t>("ID",result->GetID());
        frame.setAttribute<uint64_t>("Skipped",result->GetNumberOfSkippedImages());
        ++buffer_idx;
        if(buffer_idx >= int(buffer.size()))
            buffer_idx = 0;
    }
    callUserCallbackFcn();
}
