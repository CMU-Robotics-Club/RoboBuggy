#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/ocl/ocl.hpp"
#include <iostream>
#include <stdio.h>

#if defined(_MSC_VER) && (_MSC_VER >= 1700)
    # include <thread>
#endif

using namespace std;
using namespace cv;

#define LOOP_NUM 10
#define MAX_THREADS 10


///////////////////////////single-threading faces detecting///////////////////////////////

const static Scalar colors[] =  { CV_RGB(0,0,255),
                                  CV_RGB(0,128,255),
                                  CV_RGB(0,255,255),
                                  CV_RGB(0,255,0),
                                  CV_RGB(255,128,0),
                                  CV_RGB(255,255,0),
                                  CV_RGB(255,0,0),
                                  CV_RGB(255,0,255)
                                } ;


int64 work_begin[MAX_THREADS] = {0};
int64 work_total[MAX_THREADS] = {0};
string inputName, outputName, cascadeName;

static void workBegin(int i = 0)
{
    work_begin[i] = getTickCount();
}

static void workEnd(int i = 0)
{
    work_total[i] += (getTickCount() - work_begin[i]);
}

static double getTotalTime(int i = 0)
{
    return work_total[i] /getTickFrequency() * 1000.;
}


static void detect( Mat& img, vector<Rect>& faces,
             ocl::OclCascadeClassifier& cascade,
             double scale, bool calTime);


static void detectCPU( Mat& img, vector<Rect>& faces,
                CascadeClassifier& cascade,
                double scale, bool calTime);


static void Draw(Mat& img, vector<Rect>& faces, double scale);


// This function test if gpu_rst matches cpu_rst.
// If the two vectors are not equal, it will return the difference in vector size
// Else if will return (total diff of each cpu and gpu rects covered pixels)/(total cpu rects covered pixels)
double checkRectSimilarity(Size sz, vector<Rect>& cpu_rst, vector<Rect>& gpu_rst);

static int facedetect_one_thread(bool useCPU, double scale )
{
    CvCapture* capture = 0;
    Mat frame, frameCopy, image;

    ocl::OclCascadeClassifier cascade;
    CascadeClassifier  cpu_cascade;

    if( !cascade.load( cascadeName ) || !cpu_cascade.load(cascadeName) )
    {
        cout << "ERROR: Could not load classifier cascade: " << cascadeName << endl;
        return EXIT_FAILURE;
    }

    if( inputName.empty() )
    {
        capture = cvCaptureFromCAM(0);
        if(!capture)
            cout << "Capture from CAM 0 didn't work" << endl;
    }
    else
    {
        image = imread( inputName, CV_LOAD_IMAGE_COLOR );
        if( image.empty() )
        {
            capture = cvCaptureFromAVI( inputName.c_str() );
            if(!capture){
                cout << "Capture from AVI didn't work" << endl;
                return EXIT_FAILURE;
            }
        }
    }

    if( capture )
    {
        cout << "In capture ..." << endl;
        for(;;)
        {
            IplImage* iplImg = cvQueryFrame( capture );
            frame = iplImg;
            vector<Rect> faces;
            if( frame.empty() )
                break;
            if( iplImg->origin == IPL_ORIGIN_TL )
                frame.copyTo( frameCopy );
            else
                flip( frame, frameCopy, 0 );

            if(useCPU)
                detectCPU(frameCopy, faces, cpu_cascade, scale, false);
            else
                detect(frameCopy, faces, cascade, scale, false);

            Draw(frameCopy, faces, scale);
            if( waitKey( 10 ) >= 0 )
                break;
        }
        cvReleaseCapture( &capture );
    }
    else
    {
        cout << "In image read " << image.size() << endl;
        vector<Rect> faces;
        vector<Rect> ref_rst;
        double accuracy = 0.;
        cout << "loops: ";
        for(int i = 0; i <= LOOP_NUM; i++)
        {
            cout << i << ", ";
            if(useCPU)
                detectCPU(image, faces, cpu_cascade, scale, i!=0);
            else
            {
                detect(image, faces, cascade, scale, i!=0);
                if(i == 0)
                {
                    detectCPU(image, ref_rst, cpu_cascade, scale, false);
                    accuracy = checkRectSimilarity(image.size(), ref_rst, faces);
                }
            }
        }
        cout << "done!" << endl;
        if (useCPU)
            cout << "average CPU time (noCamera) : ";
        else
            cout << "average GPU time (noCamera) : ";
        cout << getTotalTime() / LOOP_NUM << " ms" << endl;
        cout << "accuracy value: " << accuracy <<endl;

        Draw(image, faces, scale);
        waitKey(0);
    }

    cvDestroyWindow("result");
    std::cout<< "single-threaded sample has finished" <<std::endl;
    return 0;
}

///////////////////////////////////////detectfaces with multithreading////////////////////////////////////////////
#if defined(_MSC_VER) && (_MSC_VER >= 1700)

static void detectFaces(std::string fileName, int threadNum)
{
    ocl::OclCascadeClassifier cascade;
    if(!cascade.load(cascadeName))
    {
        std::cout << "ERROR: Could not load classifier cascade: " << cascadeName << std::endl;
        return;
    }

    Mat img = imread(fileName, CV_LOAD_IMAGE_COLOR);
    if (img.empty())
    {
        std::cout << '[' << threadNum << "] " << "can't open file " + fileName <<std::endl;
        return;
    }

    ocl::oclMat d_img;
    d_img.upload(img);

    std::vector<Rect> oclfaces;
    std::thread::id tid = std::this_thread::get_id();
    std::cout << '[' << threadNum << "] "
        << "ThreadID = " << tid
        << ", CommandQueue = " << *(void**)ocl::getClCommandQueuePtr()
        << endl;
    for(int i = 0; i <= LOOP_NUM; i++)
    {
        if(i>0) workBegin(threadNum);
        cascade.detectMultiScale(d_img, oclfaces,  1.1, 3, 0|CV_HAAR_SCALE_IMAGE, Size(30, 30), Size(0, 0));
        if(i>0) workEnd(threadNum);
    }
    std::cout << '[' << threadNum << "] " << "Average time = " << getTotalTime(threadNum) / LOOP_NUM << " ms" << endl;

    for(unsigned int i = 0; i<oclfaces.size(); i++)
        rectangle(img, Point(oclfaces[i].x, oclfaces[i].y), Point(oclfaces[i].x + oclfaces[i].width, oclfaces[i].y + oclfaces[i].height), colors[i%8], 3);

    std::string::size_type pos = outputName.rfind('.');
    std::string strTid = std::to_string(_threadid);
    if( !outputName.empty() )
    {
        if(pos == std::string::npos)
        {
            std::cout << "Invalid output file name: " << outputName << std::endl;
        }
        else
        {
            std::string outputNameTid = outputName.substr(0, pos) + "_" + strTid + outputName.substr(pos);
            imwrite(outputNameTid, img);
        }
    }
    imshow(strTid, img);
    waitKey(0);
}

static void facedetect_multithreading(int nthreads)
{
    int thread_number = MAX_THREADS < nthreads ? MAX_THREADS : nthreads;
    std::vector<std::thread> threads;
    for(int i = 0; i<thread_number; i++)
        threads.push_back(std::thread(detectFaces, inputName, i));
    for(int i = 0; i<thread_number; i++)
        threads[i].join();
}
#endif

int main( int argc, const char** argv )
{

    const char* keys =
        "{ h | help       | false       | print help message }"
        "{ i | input      |             | specify input image }"
        "{ t | template   | haarcascade_frontalface_alt.xml |"
        " specify template file path }"
        "{ c | scale      |   1.0       | scale image }"
        "{ s | use_cpu    | false       | use cpu or gpu to process the image }"
        "{ o | output     | | specify output image save path(only works when input is images) }"
        "{ n | thread_num |      1      | set number of threads >= 1 }";

    CommandLineParser cmd(argc, argv, keys);
    if (cmd.get<bool>("help"))
    {
        cout << "Usage : facedetect [options]" << endl;
        cout << "Available options:" << endl;
        cmd.printParams();
        return EXIT_SUCCESS;
    }
    bool useCPU = cmd.get<bool>("s");
    inputName = cmd.get<string>("i");
    outputName = cmd.get<string>("o");
    cascadeName = cmd.get<string>("t");
    double scale = cmd.get<double>("c");
    int n = cmd.get<int>("n");

    if(n > 1)
    {
#if defined(_MSC_VER) && (_MSC_VER >= 1700)
            std::cout<<"multi-threaded sample is running" <<std::endl;
            facedetect_multithreading(n);
            std::cout<<"multi-threaded sample has finished" <<std::endl;
            return 0;
#else
            std::cout << "std::thread is not supported, running a single-threaded version" << std::endl;
#endif
    }
    if (n<0)
        std::cout<<"incorrect number of threads:" << n << ", running a single-threaded version" <<std::endl;
    else
        std::cout<<"single-threaded sample is running" <<std::endl;
    return facedetect_one_thread(useCPU, scale);

}

void detect( Mat& img, vector<Rect>& faces,
             ocl::OclCascadeClassifier& cascade,
             double scale, bool calTime)
{
    ocl::oclMat image(img);
    ocl::oclMat gray, smallImg( cvRound (img.rows/scale), cvRound(img.cols/scale), CV_8UC1 );
    if(calTime) workBegin();
    ocl::cvtColor( image, gray, CV_BGR2GRAY );
    ocl::resize( gray, smallImg, smallImg.size(), 0, 0, INTER_LINEAR );
    ocl::equalizeHist( smallImg, smallImg );

    cascade.detectMultiScale( smallImg, faces, 1.1,
                              3, 0
                              |CV_HAAR_SCALE_IMAGE
                              , Size(30,30), Size(0, 0) );
    if(calTime) workEnd();
}


void detectCPU( Mat& img, vector<Rect>& faces,
                CascadeClassifier& cascade,
                double scale, bool calTime)
{
    if(calTime) workBegin();
    Mat cpu_gray, cpu_smallImg( cvRound (img.rows/scale), cvRound(img.cols/scale), CV_8UC1 );
    cvtColor(img, cpu_gray, CV_BGR2GRAY);
    resize(cpu_gray, cpu_smallImg, cpu_smallImg.size(), 0, 0, INTER_LINEAR);
    equalizeHist(cpu_smallImg, cpu_smallImg);
    cascade.detectMultiScale(cpu_smallImg, faces, 1.1,
                             3, 0 | CV_HAAR_SCALE_IMAGE,
                             Size(30, 30), Size(0, 0));
    if(calTime) workEnd();
}


void Draw(Mat& img, vector<Rect>& faces, double scale)
{
    int i = 0;
    for( vector<Rect>::const_iterator r = faces.begin(); r != faces.end(); r++, i++ )
    {
        Point center;
        Scalar color = colors[i%8];
        int radius;
        center.x = cvRound((r->x + r->width*0.5)*scale);
        center.y = cvRound((r->y + r->height*0.5)*scale);
        radius = cvRound((r->width + r->height)*0.25*scale);
        circle( img, center, radius, color, 3, 8, 0 );
    }
    if( !outputName.empty() ) imwrite( outputName, img );
    if( abs(scale-1.0)>.001 )
    {
        resize(img, img, Size((int)(img.cols/scale), (int)(img.rows/scale)));
    }
    imshow( "result", img );

}


double checkRectSimilarity(Size sz, vector<Rect>& ob1, vector<Rect>& ob2)
{
    double final_test_result = 0.0;
    size_t sz1 = ob1.size();
    size_t sz2 = ob2.size();

    if(sz1 != sz2)
    {
        return sz1 > sz2 ? (double)(sz1 - sz2) : (double)(sz2 - sz1);
    }
    else
    {
        if(sz1==0 && sz2==0)
            return 0;
        Mat cpu_result(sz, CV_8UC1);
        cpu_result.setTo(0);

        for(vector<Rect>::const_iterator r = ob1.begin(); r != ob1.end(); r++)
        {
            Mat cpu_result_roi(cpu_result, *r);
            cpu_result_roi.setTo(1);
            cpu_result.copyTo(cpu_result);
        }
        int cpu_area = countNonZero(cpu_result > 0);


        Mat gpu_result(sz, CV_8UC1);
        gpu_result.setTo(0);
        for(vector<Rect>::const_iterator r2 = ob2.begin(); r2 != ob2.end(); r2++)
        {
            cv::Mat gpu_result_roi(gpu_result, *r2);
            gpu_result_roi.setTo(1);
            gpu_result.copyTo(gpu_result);
        }

        Mat result_;
        multiply(cpu_result, gpu_result, result_);
        int result = countNonZero(result_ > 0);
        if(cpu_area!=0 && result!=0)
            final_test_result = 1.0 - (double)result/(double)cpu_area;
        else if(cpu_area==0 && result!=0)
            final_test_result = -1;
    }
    return final_test_result;
}
