/**
 * @file CameraUnit_ATIK.cpp
 * @author Sunip K. Mukherjee (sunipkmukherjee@gmail.com)
 * @brief Implementation of CameraUnit interfaces for ATIK Cameras
 * @version 0.1
 * @date 2022-01-03
 *
 * @copyright Copyright (c) 2022
 *
 */
#include "CameraUnit_ATIK.hpp"
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <mutex>
#include <chrono>

#if !defined(OS_Windows)
#include <unistd.h>
static inline void Sleep(int dwMilliseconds)
{
    usleep(dwMilliseconds * 1000);
}
#endif

#ifndef eprintf
#define eprintf(str, ...)                                                   \
    {                                                                       \
        fprintf(stderr, "%s, %d: " str, __func__, __LINE__, ##__VA_ARGS__); \
        fflush(stderr);                                                     \
    }
#endif
#ifndef eprintlf
#define eprintlf(str, ...) eprintf(str "\n", ##__VA_ARGS__)
#endif

static inline uint64_t getTime()
{
    return ((std::chrono::duration_cast<std::chrono::milliseconds>((std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now())).time_since_epoch())).count());
}

bool CCameraUnit_ATIK::HasError(int error, unsigned int line) const
{
    switch (error)
    {
    default:
        fprintf(stderr, "%s, %d: ATIK Error %d\n", __FILE__, line, error);
        fflush(stderr);
        return true;
    case ARTEMIS_OK:
        return false;

#define ARTEMIS_ERROR(x)                                                    \
    case x:                                                                 \
        fprintf(stderr, "%s, %d: ARTEMIS error: " #x "\n", __FILE__, line); \
        fflush(stderr);                                                     \
        return true;

        ARTEMIS_ERROR(ARTEMIS_INVALID_PARAMETER)
        ARTEMIS_ERROR(ARTEMIS_NOT_CONNECTED)
        ARTEMIS_ERROR(ARTEMIS_NOT_IMPLEMENTED)
        ARTEMIS_ERROR(ARTEMIS_NO_RESPONSE)
        ARTEMIS_ERROR(ARTEMIS_NOT_INITIALIZED)
        ARTEMIS_ERROR(ARTEMIS_INVALID_FUNCTION)
        ARTEMIS_ERROR(ARTEMIS_OPERATION_FAILED)
#undef ARTEMIS_ERROR
    }
}

int CCameraUnit_ATIK::ArtemisGetCameraState(ArtemisHandle h)
{
    int state = ArtemisCameraState(h);
    switch (state)
    {
#define ARTEMIS_STATE(x)           \
    case x:                        \
        status_ = std::string(#x); \
        break;

        ARTEMIS_STATE(CAMERA_ERROR)
        ARTEMIS_STATE(CAMERA_IDLE)
        ARTEMIS_STATE(CAMERA_WAITING)
        ARTEMIS_STATE(CAMERA_EXPOSING)
        ARTEMIS_STATE(CAMERA_READING)
        ARTEMIS_STATE(CAMERA_DOWNLOADING)
        ARTEMIS_STATE(CAMERA_FLUSHING)
#undef ARTEMIS_STATE
    }
    return state;
}

CCameraUnit_ATIK::CCameraUnit_ATIK()
    : hCam(NULL),
      m_initializationOK(false),
      cancelCapture_(true),
      hasshutter(false),
      numtempsensors(0),
      binningX_(1),
      binningY_(1),
      imageLeft_(0),
      imageRight_(0),
      imageTop_(0),
      imageBottom_(0),
      roiLeft(0),
      roiRight(0),
      roiTop(0),
      roiBottom(0),
      roi_updated_(false),
      CCDWidth_(0),
      CCDHeight_(0)
{
    // do initialization stuff
    short numcameras = 0;
    // initialize camera

#ifdef _WIN32
    // First: Try to load the DLL:
    if (!ArtemisLoadDLL("AtikCameras.dll"))
    {

        return;
    }
#endif

    // Now Check API / DLL versions
    int apiVersion = ArtemisAPIVersion();
    int dllVersion = ArtemisDLLVersion();
    (void)hArtemisDLL;
    if (apiVersion != dllVersion)
    {
        eprintlf("Version do not match! API: %d DLL: %d", apiVersion, dllVersion);
        return;
    }

    // get number of cameras and names
    numcameras = ArtemisDeviceCount();
    if (numcameras == 0)
    {
        return;
    }
    if (ArtemisDeviceInUse(0))
    {
        eprintlf("Device 0 already in use");
        return;
    }
    if (!ArtemisDeviceIsCamera(0))
    {
        eprintf("Device 0 is not a camera");
        return;
    }
    if (!ArtemisDeviceName(0, cam_name))
    {
        eprintlf("Could not get camera name");
        return;
    }
    // Open the camera
    hCam = ArtemisConnect(0);
    if (!ArtemisIsConnected(hCam))
    {
        eprintlf("Could not connect to ATIK handle %p, returning", hCam);
        return;
    }
    // Get camera properties
    if (HasError(ArtemisProperties(hCam, &props), __LINE__))
    {
        eprintlf("Could not get camera properties");
        goto close;
    }

    CCDHeight_ = int(props.nPixelsY);
    CCDWidth_ = int(props.nPixelsX);

    imageLeft_ = 0;
    imageRight_ = CCDWidth_;
    imageTop_ = 0;
    imageBottom_ = CCDHeight_;
    roiLeft = imageLeft_;
    roiRight = imageRight_;
    roiTop = imageTop_;
    roiBottom = imageBottom_;

    // Set preview mode to false
    HasError(ArtemisSetPreview(hCam, false), __LINE__);

    // Set binning to 1x1
    HasError(ArtemisBin(hCam, 1, 1), __LINE__);

    // Get number of temperature sensors
    HasError(ArtemisTemperatureSensorInfo(hCam, 0, &numtempsensors), __LINE__);

    // Get shutter caps
    HasError(ArtemisCanControlShutter(hCam, &hasshutter), __LINE__);

    // Set subsample
    HasError(ArtemisSetSubSample(hCam, false), __LINE__);

    // Initialization done
    m_initializationOK = true;

    return;
close:
    ArtemisDisconnect(hCam);
    m_initializationOK = false;
}

CCameraUnit_ATIK::~CCameraUnit_ATIK()
{
    // CriticalSection::Lock lock(criticalSection_);
    std::lock_guard<std::mutex> lock(cs_);
    ArtemisDisconnect(hCam);
    m_initializationOK = false;
#ifdef _WIN32
    ArtemisUnLoadDLL();
#endif
}

void CCameraUnit_ATIK::CancelCapture()
{
    std::lock_guard<std::mutex> lock(cs_);
    cancelCapture_ = true;
    // abort acquisition
    ArtemisAbortExposure(hCam);
}

// ----------------------------------------------------------

CImageData CCameraUnit_ATIK::CaptureImage(long int &retryCount)
{
    std::unique_lock<std::mutex> lock(cs_);
    CImageData retVal;
    cancelCapture_ = false;
    BOOL image_ready;

    void *pImgBuf;

    int x, y, w, h, binx, biny;

    int sleep_time_ms = 0;

    int cameraState = 0;

    int exposure_ms = exposure_ * 1000;

    float exposure_now = exposure_;
    if (exposure_ms < 1)
        exposure_ms = 1;
    
    uint64_t exposure_start; 

    if (!m_initializationOK)
    {
        goto exit_err;
    }

    exposure_start = getTime();
    if (HasError(ArtemisStartExposureMS(hCam, exposure_ms), __LINE__))
    {
        goto exit_err;
    }
    sleep_time_ms = 1000 * ArtemisExposureTimeRemaining(hCam); // sleep time in ms
    if (sleep_time_ms < 0)
        sleep_time_ms = 0;
    lock.unlock();

    Sleep(sleep_time_ms);

    lock.lock();
    while (!(image_ready = ArtemisImageReady(hCam)))
    {
        cameraState = ArtemisGetCameraState(hCam);
        if (cameraState == CAMERA_DOWNLOADING)
        {
            status_ += std::string(" Download: ");
            status_ += std::to_string(ArtemisDownloadPercent(hCam));
            status_ += " %";
        }
        Sleep(10);
    }
    if (HasError(ArtemisGetImageData(hCam, &x, &y, &w, &h, &binx, &biny), __LINE__))
    {
        eprintlf("Error getting image data");
        goto exit_err;
    }
    pImgBuf = ArtemisImageBuffer(hCam);

    binningX_ = binx;
    binningY_ = biny;

    retVal = CImageData(w, h);
    if (pImgBuf == NULL)
    {
        eprintlf("Image buffer is NULL");
        goto exit_err;
    }
    memcpy(retVal.GetImageData(), pImgBuf, w * h * 2);
    retVal.SetImageMetadata(exposure_now, binx, biny, GetTemperature(), exposure_start, CameraName());
exit_err:
    // printf("Exiting capture\n");
    return retVal;
}

void CCameraUnit_ATIK::SetTemperature(double temperatureInCelcius)
{
    if (!m_initializationOK)
    {
        return;
    }

    int16_t settemp;
    settemp = temperatureInCelcius * 100;

    HasError(ArtemisSetCooling(hCam, settemp), __LINE__);
}

// get temperature is done
double CCameraUnit_ATIK::GetTemperature() const
{
    if (!m_initializationOK)
    {
        return INVALID_TEMPERATURE;
    }

    int temperature = 0;
    for (int i = 0; i < numtempsensors; i++)
        HasError(ArtemisTemperatureSensorInfo(hCam, i + 1, &temperature), __LINE__);

    double retVal;

    retVal = double(temperature) / 100;
    return retVal;
}

void CCameraUnit_ATIK::SetBinningAndROI(int binX, int binY, int x_min, int x_max, int y_min, int y_max)
{
    if (!m_initializationOK)
    {
        return;
    }

    std::lock_guard<std::mutex> lock(cs_);
    if (!m_initializationOK)
    {
        return;
    }

    if (binX < 1)
        binX = 1;
    if (binX > 16)
        binX = 16;

    bool change_bin = false;
    if (binningX_ != binX)
    {
        change_bin = true;
    }

    if (binY < 1)
        binY = 1;
    if (binY > 16)
        binY = 16;

    if (binningY_ != binY)
    {
        change_bin = true;
    }

    if (change_bin)
    {
        if (HasError(ArtemisBin(hCam, binX, binY), __LINE__))
            return;
        binningY_ = binY;
        binningX_ = binX;
    }

    int imageLeft, imageRight, imageTop, imageBottom;

    imageLeft = x_min;
    imageRight = (x_max - x_min) + imageLeft;
    imageTop = y_min;
    imageBottom = (y_max - y_min) + imageTop;

    if (imageRight > GetCCDWidth())
        imageRight = GetCCDWidth();
    if (imageLeft < 0)
        imageLeft = 0;
    if (imageRight <= imageLeft)
        imageRight = GetCCDWidth();

    if (imageBottom > GetCCDWidth())
        imageBottom = GetCCDHeight();
    if (imageTop < 0)
        imageTop = 0;
    if (imageBottom <= imageTop)
        imageBottom = GetCCDHeight();

    if (!HasError(ArtemisSubframe(hCam, imageLeft, imageTop, imageRight - imageLeft, imageBottom - imageTop), __LINE__))
    {
        imageLeft_ = imageLeft;
        imageRight_ = imageRight;
        imageTop_ = imageTop_;
        imageBottom_ = imageBottom_;
        roiLeft = imageLeft;
        roiRight = imageRight;
        roiBottom = imageBottom;
        roiTop = imageTop;
    }
    else
    {
        imageLeft_ = 0;
        imageRight_ = GetCCDWidth();
        imageTop_ = 0;
        imageBottom_ = GetCCDHeight();
        roiLeft = imageLeft_;
        roiRight = imageRight_;
        roiBottom = imageBottom_;
        roiTop = imageTop_;
    }

    // printf("%d %d, %d %d | %d %d\n", binningX_, binningY_, imageLeft_, imageRight_, imageBottom_, imageTop_);
}

const ROI *CCameraUnit_ATIK::GetROI() const
{
    static ROI roi;
    roi.x_min = roiLeft;
    roi.x_max = roiRight;
    roi.y_min = roiBottom;
    roi.y_max = roiTop;
    return &roi;
}

void CCameraUnit_ATIK::SetShutter(bool open)
{
    if (!m_initializationOK)
    {
        return;
    }
    if (hasshutter)
    {
        if (open)
        {
            HasError(ArtemisOpenShutter(hCam), __LINE__);
            requestShutterOpen_ = true;
        }
        else
        {
            HasError(ArtemisCloseShutter(hCam), __LINE__);
            requestShutterOpen_ = false;
        }
    }
}

void CCameraUnit_ATIK::SetShutterIsOpen(bool open)
{
    SetShutter(open);
}

// exposure time done
void CCameraUnit_ATIK::SetExposure(float exposureInSeconds)
{
    if (!m_initializationOK)
    {
        return;
    }

    if (exposureInSeconds <= 0)
    {
        exposureInSeconds = 0.0;
    }

    long int maxexposurems = exposureInSeconds * 1000;

    if (maxexposurems > 10 * 60 * 1000) // max exposure 10 minutes
        maxexposurems = 10 * 60 * 1000;
    std::lock_guard<std::mutex> lock(cs_);
    exposure_ = maxexposurems * 0.001; // 1 ms increments only
}

void CCameraUnit_ATIK::SetReadout(int ReadSpeed)
{
    if (!m_initializationOK)
    {
        return;
    }

    return;
}