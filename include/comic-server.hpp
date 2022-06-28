/**
 * @file comic-server.hpp
 * @author Sunip K. Mukherjee (sunipkmukherjee@gmail.com)
 * @brief
 * @version 0.1
 * @date 2021-12-07
 *
 * @copyright Copyright (c) 2021
 *
 */

#ifndef _COMIC_SERVER_HPP_
#define _COMIC_SERVER_HPP_

#include "CameraUnit_ATIK.hpp"
#include "network_common.hpp"
#include "network_server.hpp"
#include "meb_print.h"
#include "ProtQueue.hpp"

/**
 * @brief Network Image Metadata
 *
 * @return typedef struct
 */
typedef struct __attribute__((packed))
{
    uint16_t x;
    uint16_t y;
    uint16_t width;
    uint16_t height;
    int32_t temperature;
    uint32_t exposure_ms;
    uint64_t tstamp;
    int32_t type;
    int32_t size;
} netimg_meta;

typedef enum
{
    JPEGMONO = 15,
    JPEGRGBA = 20,
    JPEGRGB = 25,
    RAW8 = 30,
    RAW16 = 35
} netimg_type;

typedef struct __attribute__((packed))
{
    uint8_t binX;
    uint8_t binY;
    uint16_t x;
    uint16_t y;
    uint16_t h;
    uint16_t w;
} netcmd_binroi;

typedef enum
{
    CoolingState = 0x00a0,     // command: 0xa, state: 0x0 or 0x1
    CoolingTarget = 0x00b4,    // command: 0xb, size: 0x4 (sizeof(int32_t))
    SaveImageCadence = 0x00c8, // command: 0xc, size: 0x8
    ExposureCadence = 0x00d8,
    SaveImage = 0x00e0,
    TotalSave = 0x00f4,
    SetBinROI = 0x0100 | sizeof(netcmd_binroi)
} netcmd_type;

typedef struct __attribute__((packed))
{
    int32_t ccd_temp;
    int32_t ccdtemp_target;
    int32_t cooling_active;
    uint64_t exp_cadence_ms;
    uint64_t save_cadence_ms;
    int32_t saving_image;
    int32_t current_save;
    int32_t total_save;
    char SaveFileStatus[30];
} netdata_telem;

typedef enum
{
    SAVE_ERR_DIR_PATH_NOT_FOUND
} netcmd_err;

typedef enum
{
    CMD = 40, // command from controller
    DATA,     // data sent to controller
    TELEM,    // telemetry sent to controller
    INFO,     // info sent to controller
    ERR       // Error info sent to controller
} comic_netdata;

typedef struct
{
    std::mutex cs;

    uint64_t tstamp;
    uint32_t exposure_ms;
    uint16_t binX;
    uint16_t binY;
    uint16_t x_min;
    uint16_t x_max;
    uint16_t y_min;
    uint16_t y_max;
    float ccdtemp;
    uint16_t *data;
    int size;
    int width;
    int height;
    bool data_avail;
    bool new_data;

    void Init()
    {
        std::lock_guard<std::mutex> lock(cs);
        tstamp = 0;
        exposure_ms = 0;
        binX = 0;
        binY = 0;
        x_min = 0;
        x_max = 0;
        y_min = 0;
        y_max = 0;
        ccdtemp = 0;
        data = 0;
        size = 0;
        width = 0;
        height = 0;
        data_avail = 0;
        new_data = 0;
    };

    void Reset()
    {
        std::lock_guard<std::mutex> lock(cs);
        if (data)
            delete[] data;
        Init();
    };
} raw_image;

static std::mutex capturelock;               // This mutex is locked when image capture is ongoing
static CCameraUnit *cam = nullptr;           // Camera object
static CImageData image;                     // Image object
static int32_t CCDTemperature;               // CCD Temperature, in 100th of degree
static int32_t CCDTemperatureTarget = -3000; // CCD Temperature Target, in 100th of degree
static uint64_t ExposureCadenceMs = 1000;    // Time between exposures in ms
static uint64_t SaveCadenceMs = 5000;        // Time between saves
static int32_t SaveImageNum = 0;             // 0 for continuous exposure save, > 0 for saving number of exposures, < 0 for no save
static int32_t CurrentSaveImageNum = 0;      // Currently saving index
static bool SaveImageCommand = false;        // Save image is false by default
static char SaveImagePrefix[20] = "comic";   // Default file name prefix
static char SaveImageDir[256] = "./fits/";   // Default save directory
static char DirPathErrorName[256];           // Error finding directory name
static ProtQueue<NetFrame *> tx_queue;       // Transmit queue
static NetVertex CtrlVertex;                 // Controller netvertex
static raw_image MainImage[1];               // Image data

/**
 * @brief This thread operates the camera to collect exposures.
 * Also sends collected exposures to the controller over the network,
 * and saves the collected exposures as instructed.
 * 
 * @param _inout Unused.
 * @return void* 
 */
void *CameraThread(void *_inout);
/**
 * @brief This thread receives data from the controller and parses
 * the data in form of commands.
 * 
 * @param _inout 
 * @return void* 0
 */
void *ServerRxThread(void *_inout);
/**
 * @brief This thread transmits data from the camera system to the
 * controller.
 * 
 * @param _inout 
 * @return void* 0
 */
void *ServerTxThread(void *_inout);
/**
 * @brief This thread controls the thermoelectric cooler external
 * to the camera.
 * 
 * @param _inout Unused.
 * @return void* 0
 */
void *CoolerThread(void *_inout);

#ifndef OS_Windows
#define _snprintf snprintf
#endif

char SaveFitsStatus[30];

#include <fitsio.h>
/**
 * @brief Save image data to FITS file
 * 
 * @param filePrefix Prefix to autogenerated file name
 * @param DirPrefix Path to file
 * @param i Image index, out of n saves
 * @param n Total number of saves in this run
 * @param image Image data
 */
void SaveFits(char *filePrefix, char *DirPrefix, int i, int n, raw_image *image);

/**
 * @brief Check if a directory exists
 * 
 * @param dirName Path to directory
 * @return true 
 * @return false 
 */
bool dirExists(const char *dirName);

#ifndef OS_Windows
#define ERROR_ALREADY_EXISTS EEXIST
#define ERROR_PATH_NOT_FOUND 6557
/**
 * @brief Create a directory
 * 
 * @param path Path to directory
 * @param _permissions Pointer to mode_t permissions, NULL for default (0777)
 * @return int 1 on success, 0 on error
 */
int CreateDirectoryA(const char *path, unsigned int *_permissions);
/**
 * @brief Get the error from CreateDirectoryA function.
 * 
 * @return int ERROR_ALREADY_EXISTS or ERROR_PATH_NOT_FOUND or 0 on success
 */
int GetLastError();
#endif

#include <chrono>

static inline uint64_t getTime()
{
    return ((std::chrono::duration_cast<std::chrono::milliseconds>((std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now())).time_since_epoch())).count());
}

#endif // _COMIC_SERVER_HPP_