#include "CameraUnit_ATIK.hpp"
#include "meb_print.h"
#include "gpiodev/gpiodev.h"
#include <signal.h>
#include <unistd.h>
#include <string.h>
#include <stdint.h>
#include <inttypes.h>
#include <stdlib.h>
#include <fcntl.h>
#include <math.h>
#include <limits.h>
#include <clkgen.h>

#include <dirent.h>
#include <errno.h>

#include <thread>
#include "mcp9808/mcp9808.h"

#define _Catchable

const char *bootcount_fname = "./bootcount.dat";

static int GetBootCount();
void _Catchable checknmakedir(const char *path);

#include <time.h>
static inline double get_tstamp()
{
    struct timespec tm;
    clock_gettime(CLOCK_REALTIME, &tm);
    uint64_t msec = tm.tv_sec * 1000LLU;
    msec += (tm.tv_nsec / 1000000LLU);
    return msec * 0.001;
}

volatile sig_atomic_t done = 0;
void sighandler(int sig)
{
    done = 1;
}

static int bootCount = 0;
static char dirname[256] = {
    0,
};

void frame_grabber(CCameraUnit *cam, double cadence = 10) // cadence in seconds
{
    static float maxExposure = 120;
    static float pixelPercentile = 90;
    static int pixelTarget = 40000;
    static int pixelUncertainty = 5000;
    static int maxBin = 4;
    static int imgXMin = 100, imgYMin = 335, imgXMax = -1, imgYMax = -1;

    static float exposure = 0.2; // 200 ms
    static int bin = 1;          // start with bin 1

    long retrycount = 10;

    if (cam == nullptr)
    {
        dbprintlf(FATAL "Camera is NULL");
        return;
    }

    cam->SetBinningAndROI(bin, bin, imgXMin, imgXMax, imgYMin, imgYMax); // set binning and ROI
    cam->SetExposure(exposure);                                          // set exposure

    while (!done)
    {
        double start = get_tstamp();
        CImageData img = cam->CaptureImage(retrycount); // capture frame
        if (!img.SaveFits(NULL, dirname))               // save frame
        {
            bprintlf(FATAL "[%.3f] Could not save FITS", start);
        }
        else
        {
            bprintlf(GREEN_FG "[%.3f] Saved: Exposure %.3f s, Bin %d", start, exposure, bin);
        }
        sync();
        // run auto exposure
        img.FindOptimumExposure(exposure, bin, pixelPercentile, pixelTarget, maxExposure, maxBin, 100, pixelUncertainty);
        cam->SetBinningAndROI(bin, bin, imgXMin, imgXMax, imgYMin, imgYMax); // set binning and ROI
        cam->SetExposure(exposure);
        start = get_tstamp() - start;
        if (start < cadence)
        {
            int res = (start - cadence) * 1000000;
            dbprintlf("Res: %d", res);
            usleep(res % 1000000);
            res -= res % 1000000;
            while ((res > 0) && (!done))
            {
                usleep(1000000);
                res -= 1000000;
                dbprintlf("Res: %d", res);
            }
        }
    }
}

int main(int argc, char *argv[])
{
    uint64_t cadence = 30;

    gpioSetMode(11, GPIO_OUT);
    gpioWrite(11, GPIO_HIGH);

    sleep(1);

    signal(SIGINT, sighandler);
    CCameraUnit *cam = nullptr;
    long retryCount = 10;
    do
    {
        cam = new CCameraUnit_ATIK();
        if (cam->CameraReady())
        {
            bprintlf(GREEN_FG "Camera ready");
            break;
        }
        else
        {
            delete cam;
            cam = nullptr;
        }
    } while (retryCount--);

    if (cam == nullptr)
    {
        bprintlf(RED_FG "Error opening camera");
        exit(0);
    }

    bootCount = GetBootCount();

    snprintf(dirname, sizeof(dirname), "data/%d", bootCount);

    try
    {
        checknmakedir(dirname);
    }
    catch (const std::exception &e)
    {
        dbprintlf(FATAL "Error creating directory: %s", e.what());
        delete cam;
        gpioWrite(11, GPIO_LOW);
        exit(0);
    }

    std::thread camera_thread(frame_grabber, cam, cadence);

    mcp9808 board_tsensor;
    bool board_ts_active = board_tsensor.begin(0x18);
    if (!board_ts_active)
    {
        dbprintlf(RED_FG "Board temperature sensor not found.");
    }
    std::string tfile_name = dirname;
    tfile_name += "/templog.txt";
    FILE *fp = fopen(tfile_name.c_str(), "a");
    if (fp == NULL)
    {
        dbprintlf(FATAL "Could not open temperature log file");
    }
    while (!done)
    {
        double tnow = get_tstamp();
        short board_temp = -9999;
        if (board_ts_active)
            board_temp = board_tsensor.readTemp();
        short ccd_temp = cam->GetTemperature() * 100;
        std::string tlog_str = std::to_string(tnow) + "," + std::to_string(ccd_temp) + "," + std::to_string(board_temp) + "\n";
        bprintf("%s", tlog_str.c_str());
        if (fp != NULL)
        {
            fprintf(fp, "%s", tlog_str.c_str());
        }
        usleep(1000000); // every second
    }
    if (fp != NULL)
    {
        fclose(fp);
    }
    camera_thread.join();
    if (board_ts_active)
        board_tsensor.shutdown();
    sync();

    delete cam;
    gpioWrite(11, GPIO_LOW);
    return 0;
}

static int create_or_open(const char *path, int create_flags, int open_flags,
                          int *how)
{
    int fd;
    create_flags |= (O_CREAT | O_EXCL);
    open_flags &= ~(O_CREAT | O_EXCL);
    for (;;)
    {
        *how = 1;
        fd = open(path, create_flags, 0666);
        if (fd >= 0)
            break;
        if (errno != EEXIST)
            break;
        *how = 0;
        fd = open(path, open_flags);
        if (fd >= 0)
            break;
        if (errno != ENOENT)
            break;
    }
    return fd;
}

static int GetBootCount()
{
    // 1. Check if file exists, if not ask for step counter and clear all calibration
    char cwd[PATH_MAX];
    off_t sz;
    memset(cwd, 0x0, sizeof(cwd));
    if (getcwd(cwd, sizeof(cwd)) == NULL)
    {
        dbprintlf("Error getting current directory.");
    }
    int newfile = 0;
    int fd = create_or_open(bootcount_fname, O_WRONLY | O_SYNC, O_RDWR | O_SYNC, &newfile);
    int current_pos = 0;
    if (fd < 0)
    {
        dbprintlf(FATAL "Error creating/opening file. Error: %s, Errno: %d.", strerror(errno), errno);
    }
    if (!newfile)
    {
        // 2. File exists
        sz = lseek(fd, 0, SEEK_END);
        if (sz < (int)(sizeof(int)))
        {
            dbprintlf("Size of position file %u, invalid.", (unsigned int)sz);
            goto rewrite;
        }
        lseek(fd, 0, SEEK_SET);
        if (read(fd, &current_pos, sizeof(current_pos)) < 0)
        {
            dbprintlf(RED_FG "Unexpected EOF on %s/%s.", cwd, bootcount_fname);
            current_pos = 0;
        }
    }
rewrite:
    lseek(fd, 0, SEEK_SET);
    int _current_pos = current_pos + 1;
    if (write(fd, &_current_pos, sizeof(_current_pos)) != sizeof(_current_pos))
    {
        dbprintlf(RED_FG "Could not update current position.");
    }
    close(fd);
    return current_pos;
}

void _Catchable checknmakedir(const char *path)
{
    DIR *dir = opendir(path);
    if (dir)
    {
        /* Directory exists. */
        closedir(dir);
    }
    else if (ENOENT == errno)
    {
        char buf[512];
        snprintf(buf, sizeof(buf), "mkdir -p %s", path);
        int retval = system(buf);
        if (retval)
        {
            dbprintlf(FATAL "could not create directory %s, retval %d", path, retval);
            throw std::runtime_error("Could not create savedir, retval " + std::to_string(retval));
        }
    }
    else
    {
        dbprintlf(FATAL "could not create directory %s", path);
        throw std::runtime_error("Could not create savedir");
    }
}