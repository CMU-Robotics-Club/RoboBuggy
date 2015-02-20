/* This sample demonstrates working on one piece of data using two GPUs.
   It splits input into two parts and processes them separately on different
   GPUs. */

// Disable some warnings which are caused with CUDA headers
#if defined(_MSC_VER)
#pragma warning(disable: 4201 4408 4100)
#endif

#include <iostream>
#include "cvconfig.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/gpu/gpu.hpp"

#if !defined(HAVE_CUDA) || !defined(HAVE_TBB) || defined(__arm__)

int main()
{
#if !defined(HAVE_CUDA)
    std::cout << "CUDA support is required (CMake key 'WITH_CUDA' must be true).\n";
#endif

#if !defined(HAVE_TBB)
    std::cout << "TBB support is required (CMake key 'WITH_TBB' must be true).\n";
#endif

#if defined(__arm__)
    std::cout << "Unsupported for ARM CUDA library." << std::endl;
#endif

    return 0;
}

#else

#include <cuda.h>
#include <cuda_runtime.h>
#include "opencv2/core/internal.hpp" // For TBB wrappers

using namespace std;
using namespace cv;
using namespace cv::gpu;

struct Worker { void operator()(int device_id) const; };
void destroyContexts();

#define safeCall(expr) safeCall_(expr, #expr, __FILE__, __LINE__)
inline void safeCall_(int code, const char* expr, const char* file, int line)
{
    if (code != CUDA_SUCCESS)
    {
        std::cout << "CUDA driver API error: code " << code << ", expr " << expr
            << ", file " << file << ", line " << line << endl;
        destroyContexts();
        exit(-1);
    }
}

// Each GPU is associated with its own context
CUcontext contexts[2];

void inline contextOn(int id)
{
    safeCall(cuCtxPushCurrent(contexts[id]));
}

void inline contextOff()
{
    CUcontext prev_context;
    safeCall(cuCtxPopCurrent(&prev_context));
}

// GPUs data
GpuMat d_left[2];
GpuMat d_right[2];
StereoBM_GPU* bm[2];
GpuMat d_result[2];

static void printHelp()
{
    std::cout << "Usage: driver_api_stereo_multi_gpu --left <left_image> --right <right_image>\n";
}

int main(int argc, char** argv)
{
    if (argc < 5)
    {
        printHelp();
        return -1;
    }

    int num_devices = getCudaEnabledDeviceCount();
    if (num_devices < 2)
    {
        std::cout << "Two or more GPUs are required\n";
        return -1;
    }

    for (int i = 0; i < num_devices; ++i)
    {
        cv::gpu::printShortCudaDeviceInfo(i);

        DeviceInfo dev_info(i);
        if (!dev_info.isCompatible())
        {
            std::cout << "GPU module isn't built for GPU #" << i << " ("
                 << dev_info.name() << ", CC " << dev_info.majorVersion()
                 << dev_info.minorVersion() << "\n";
            return -1;
        }
    }

    // Load input data
    Mat left, right;
    for (int i = 1; i < argc; ++i)
    {
        if (string(argv[i]) == "--left")
        {
            left = imread(argv[++i], CV_LOAD_IMAGE_GRAYSCALE);
            CV_Assert(!left.empty());
        }
        else if (string(argv[i]) == "--right")
        {
            right = imread(argv[++i], CV_LOAD_IMAGE_GRAYSCALE);
            CV_Assert(!right.empty());
        }
        else if (string(argv[i]) == "--help")
        {
            printHelp();
            return -1;
        }
    }


    // Init CUDA Driver API
    safeCall(cuInit(0));

    // Create context for GPU #0
    CUdevice device;
    safeCall(cuDeviceGet(&device, 0));
    safeCall(cuCtxCreate(&contexts[0], 0, device));
    contextOff();

    // Create context for GPU #1
    safeCall(cuDeviceGet(&device, 1));
    safeCall(cuCtxCreate(&contexts[1], 0, device));
    contextOff();

    // Split source images for processing on GPU #0
    contextOn(0);
    d_left[0].upload(left.rowRange(0, left.rows / 2));
    d_right[0].upload(right.rowRange(0, right.rows / 2));
    bm[0] = new StereoBM_GPU();
    contextOff();

    // Split source images for processing on the GPU #1
    contextOn(1);
    d_left[1].upload(left.rowRange(left.rows / 2, left.rows));
    d_right[1].upload(right.rowRange(right.rows / 2, right.rows));
    bm[1] = new StereoBM_GPU();
    contextOff();

    // Execute calculation in two threads using two GPUs
    int devices[] = {0, 1};
    parallel_do(devices, devices + 2, Worker());

    // Release the first GPU resources
    contextOn(0);
    imshow("GPU #0 result", Mat(d_result[0]));
    d_left[0].release();
    d_right[0].release();
    d_result[0].release();
    delete bm[0];
    contextOff();

    // Release the second GPU resources
    contextOn(1);
    imshow("GPU #1 result", Mat(d_result[1]));
    d_left[1].release();
    d_right[1].release();
    d_result[1].release();
    delete bm[1];
    contextOff();

    waitKey();
    destroyContexts();
    return 0;
}


void Worker::operator()(int device_id) const
{
    contextOn(device_id);

    bm[device_id]->operator()(d_left[device_id], d_right[device_id],
                              d_result[device_id]);

    std::cout << "GPU #" << device_id << " (" << DeviceInfo().name()
        << "): finished\n";

    contextOff();
}


void destroyContexts()
{
    safeCall(cuCtxDestroy(contexts[0]));
    safeCall(cuCtxDestroy(contexts[1]));
}

#endif
