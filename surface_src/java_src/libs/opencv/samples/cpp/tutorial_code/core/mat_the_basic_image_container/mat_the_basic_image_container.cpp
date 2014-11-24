/*  For description look into the help() function. */

#include "opencv2/core/core.hpp"
#include <iostream>

using namespace std;
using namespace cv;

static void help()
{
    cout
    << "\n--------------------------------------------------------------------------" << endl
    << "This program shows how to create matrices(cv::Mat) in OpenCV and its serial"
    << " out capabilities"                                                            << endl
    << "That is, cv::Mat M(...); M.create and cout << M. "                            << endl
    << "Shows how output can be formated to OpenCV, python, numpy, csv and C styles." << endl
    << "Usage:"                                                                       << endl
    << "./cvout_sample"                                                               << endl
    << "--------------------------------------------------------------------------"   << endl
    << endl;
}

int main(int,char**)
{
    help();
    // create by using the constructor
    Mat M(2,2, CV_8UC3, Scalar(0,0,255));
    cout << "M = " << endl << " " << M << endl << endl;

    // create by using the create function()
    M.create(4,4, CV_8UC(2));
    cout << "M = "<< endl << " "  << M << endl << endl;

    // create multidimensional matrices
    int sz[3] = {2,2,2};
    Mat L(3,sz, CV_8UC(1), Scalar::all(0));
    // Cannot print via operator <<

    // Create using MATLAB style eye, ones or zero matrix
    Mat E = Mat::eye(4, 4, CV_64F);
    cout << "E = " << endl << " " << E << endl << endl;

    Mat O = Mat::ones(2, 2, CV_32F);
    cout << "O = " << endl << " " << O << endl << endl;

    Mat Z = Mat::zeros(3,3, CV_8UC1);
    cout << "Z = " << endl << " " << Z << endl << endl;

    // create a 3x3 double-precision identity matrix
    Mat C = (Mat_<double>(3,3) << 0, -1, 0, -1, 5, -1, 0, -1, 0);
    cout << "C = " << endl << " " << C << endl << endl;

    Mat RowClone = C.row(1).clone();
    cout << "RowClone = " << endl << " " << RowClone << endl << endl;

    // Fill a matrix with random values
    Mat R = Mat(3, 2, CV_8UC3);
    randu(R, Scalar::all(0), Scalar::all(255));

    // Demonstrate the output formating options
    cout << "R (default) = " << endl <<        R           << endl << endl;
    cout << "R (python)  = " << endl << format(R,"python") << endl << endl;
    cout << "R (numpy)   = " << endl << format(R,"numpy" ) << endl << endl;
    cout << "R (csv)     = " << endl << format(R,"csv"   ) << endl << endl;
    cout << "R (c)       = " << endl << format(R,"C"     ) << endl << endl;

    Point2f P(5, 1);
    cout << "Point (2D) = " << P << endl << endl;

    Point3f P3f(2, 6, 7);
    cout << "Point (3D) = " << P3f << endl << endl;


    vector<float> v;
    v.push_back( (float)CV_PI);   v.push_back(2);    v.push_back(3.01f);

    cout << "Vector of floats via Mat = " << Mat(v) << endl << endl;

    vector<Point2f> vPoints(20);
    for (size_t i = 0; i < vPoints.size(); ++i)
        vPoints[i] = Point2f((float)(i * 5), (float)(i % 7));

    cout << "A vector of 2D Points = " << vPoints << endl << endl;
    return 0;
}
