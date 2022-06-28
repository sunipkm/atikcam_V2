/**
 * @file CameraUnit.hpp
 * @author Sunip K. Mukherjee (sunipkmukherjee@gmail.com)
 * @brief Virtual CameraUnit interface for different camera backends
 * @version 0.1
 * @date 2022-01-03
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef __CAMERAUNIT_HPP__
#define __CAMERAUNIT_HPP__

#include "ImageData.hpp"
#include <string>

typedef struct
{
    int x_min;
    int x_max;
    int y_min;
    int y_max;
} ROI;

const double INVALID_TEMPERATURE = -273.0;
class CCameraUnit
{
public:
    virtual ~CCameraUnit(){};

    /**
     * @brief Capture image from the connected camera
     * 
     * @param retryCount Unused
     * @return CImageData Container with raw image data
     */
    virtual CImageData CaptureImage(long int &retryCount) = 0;
    /**
     * @brief Cancel an ongoing capture
     * 
     */
    virtual void CancelCapture() = 0;

    /**
     * @brief Check if camera was initialized properly
     * 
     * @return true 
     * @return false 
     */
    virtual bool CameraReady() const = 0;
    /**
     * @brief Get the name of the connected camera
     * 
     * @return const char* 
     */
    virtual const char *CameraName() const = 0;
    /**
     * @brief Set the exposure time in seconds
     * 
     * @param exposureInSeconds 
     */
    virtual void SetExposure(float exposureInSeconds) = 0;
    /**
     * @brief Get the currently set exposure
     * 
     * @return float 
     */
    virtual float GetExposure() const = 0;
    /**
     * @brief Open or close the shutter
     * 
     * @param open 
     */
    virtual void SetShutterIsOpen(bool open) = 0;
    /**
     * @brief Set the readout speed (unused)
     * 
     * @param ReadSpeed 
     */
    virtual void SetReadout(int ReadSpeed) = 0;
    /**
     * @brief Set the cooler target temperature
     * 
     * @param temperatureInCelcius 
     */
    virtual void SetTemperature(double temperatureInCelcius) = 0;
    /**
     * @brief Get the current detector temperature
     * 
     * @return double 
     */
    virtual double GetTemperature() const = 0;
    /**
     * @brief Set the Binning And ROI information
     * 
     * @param x X axis binning
     * @param y Y axis binning
     * @param x_min Leftmost pixel index (unbinned)
     * @param x_max Rightmost pixel index (unbinned)
     * @param y_min Topmost pixel index (unbinned)
     * @param y_max Bottommost pixel index (unbinned)
     */
    virtual void SetBinningAndROI(int x, int y, int x_min = 0, int x_max = 0, int y_min = 0, int y_max = 0) = 0;
    /**
     * @brief Get the X binning set on the detector
     * 
     * @return int 
     */
    virtual int GetBinningX() const = 0;
    /**
     * @brief Get the Y binning set on the detector
     * 
     * @return int 
     */
    virtual int GetBinningY() const = 0;
    /**
     * @brief Get the currently set region of interest
     * 
     * @return const ROI* 
     */
    virtual const ROI *GetROI() const = 0;
    /**
     * @brief Get the current status string
     * 
     * @return std::string 
     */
    virtual std::string GetStatus() const = 0; // should return empty string when idle
    /**
     * @brief Get the detector width in pixels
     * 
     * @return int 
     */
    virtual int GetCCDWidth() const = 0;
    /**
     * @brief Get the detector height in pixels
     * 
     * @return int 
     */
    virtual int GetCCDHeight() const = 0;
};

#endif // __CAMERAUNIT_HPP__
