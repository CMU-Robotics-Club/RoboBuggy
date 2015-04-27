#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <iostream>

using namespace cv;
using namespace std;

static void help()
{
    cout << "This program demonstrates finding the minimum enclosing box or circle of a set\n"
            "of points using functions: minAreaRect() minEnclosingCircle().\n"
            "Random points are generated and then enclosed.\n"
            "Call:\n"
            "./minarea\n"
            "Using OpenCV version %s\n" << CV_VERSION << "\n" << endl;
}

int main( int /*argc*/, char** /*argv*/ )
{
    help();

    Mat img(500, 500, CV_8UC3);
    RNG& rng = theRNG();

    for(;;)
    {
        int i, count = rng.uniform(1, 101);
        vector<Point> points;
        for( i = 0; i < count; i++ )
        {
            Point pt;
            pt.x = rng.uniform(img.cols/4, img.cols*3/4);
            pt.y = rng.uniform(img.rows/4, img.rows*3/4);

            points.push_back(pt);
        }

        RotatedRect box = minAreaRect(Mat(points));

        Point2f center, vtx[4];
        float radius = 0;
        minEnclosingCircle(Mat(points), center, radius);
        box.points(vtx);

        img = Scalar::all(0);
        for( i = 0; i < count; i++ )
            circle( img, points[i], 3, Scalar(0, 0, 255), CV_FILLED, CV_AA );

        for( i = 0; i < 4; i++ )
            line(img, vtx[i], vtx[(i+1)%4], Scalar(0, 255, 0), 1, CV_AA);

        circle(img, center, cvRound(radius), Scalar(0, 255, 255), 1, CV_AA);

        imshow( "rect & circle", img );

        char key = (char)waitKey();
        if( key == 27 || key == 'q' || key == 'Q' ) // 'ESC'
            break;
    }

    return 0;
}
