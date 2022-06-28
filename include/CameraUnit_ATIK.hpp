/**
 * @file CameraUnit_ATIK.hpp
 * @author Sunip K. Mukherjee (sunipkmukherjee@gmail.com)
 * @brief Interface for ATIK CameraUnit
 * @version 0.1
 * @date 2022-01-03
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef __CAMERAUNIT_ATIK_HPP__
#define __CAMERAUNIT_ATIK_HPP__

#include "CameraUnit.hpp"
#include "AtikCameras.h"
#include <mutex>

class CCameraUnit_ATIK : public CCameraUnit
{
    ArtemisHandle hCam;
    struct ARTEMISPROPERTIES props;

    bool m_initializationOK;
    std::mutex cs_;
    bool cancelCapture_;
    std::string status_;

    bool hasshutter;

    int numtempsensors;

    float exposure_;
    bool exposure_updated_;

    bool requestShutterOpen_;
    bool shutter_updated_;

    int binningX_;
    int binningY_;

    int imageLeft_;
    int imageRight_;
    int imageTop_;
    int imageBottom_;

    int roiLeft;
    int roiRight;
    int roiTop;
    int roiBottom;

    bool roi_updated_;

    int CCDWidth_;
    int CCDHeight_;

    char cam_name[100];

public:
    /**
     * @brief Construct a new Atik Camera Object. This will connect to the first available Atik camera.
     * 
     */
    CCameraUnit_ATIK();
    /**
     * @brief Close connection to the connected Atik camera.
     * 
     */
    ~CCameraUnit_ATIK();

    CImageData CaptureImage(long int &retryCount);
    void CancelCapture();

    inline bool CameraReady() const { return m_initializationOK; }
    inline const char *CameraName() const { return cam_name; }
    void SetExposure(float exposureInSeconds);
    inline float GetExposure() const { return exposure_; }
    void SetShutterIsOpen(bool open);
    void SetReadout(int ReadSpeed);
    void SetTemperature(double temperatureInCelcius);
    double GetTemperature() const;
    void SetBinningAndROI(int x, int y, int x_min = 0, int x_max = 0, int y_min = 0, int y_max = 0);
    inline int GetBinningX() const { return binningX_; }
    inline int GetBinningY() const { return binningY_; }
    const ROI *GetROI() const;
    inline std::string GetStatus() const { return status_; }
    inline int GetCCDWidth() const { return CCDWidth_; }
    inline int GetCCDHeight() const { return CCDHeight_; }

private:
    bool StatusIsIdle();
    void SetShutter(bool open);
    bool HasError(int error, unsigned int line) const;
    int ArtemisGetCameraState(ArtemisHandle h);
};

#endif // __CAMERAUNIT_ATIK_HPP__
