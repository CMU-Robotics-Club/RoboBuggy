#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cstdlib>

using namespace std;
using namespace cv;

int main(int argc, char* argv[]) {
	// Parse command line arguments
	bool display = true;
	int cameras[] = {2, 3, 0, 0, 0};
	char* names[] = {"cam1", "cam2", "cam3", "cam4", "cam5"};
	string files[] = {"C:\\Users\\robot\\buggy-log\\out1.avi",
			"C:\\Users\\abc\\buggy-log\\out2.avi",
			"C:\\Users\\abc\\buggy-log\\out3.avi",
			"C:\\Users\\abc\\buggy-log\\out4.avi",
			"C:\\Users\\abc\\buggy-log\\out5.avi" };
	int num_cameras = 0;
	VideoCapture captures[5];
	VideoWriter videos[5];

	for (int i = 0; i < argc; i++) {
		if (argv[i] == "-c") {
			if (i+1 != argc) {
				cout << "Added camera" << endl;
				if (num_cameras < 5) cameras[num_cameras++] = atoi(argv[i++]);
			}
		} else if (argv[i] == "-s") {
			display = false;
		}
	}

	if (num_cameras <= 0) {
		num_cameras = 2;
	}

	if (display) {
		for (int i = 0; i < num_cameras; i++) {
			cvNamedWindow(names[i], 1);
		}
	}

	Mat src, dst;
	vector<Mat> hsv_src;
	Size img_size;

	for (int i = 0; i < num_cameras; i++) {
		VideoCapture camera(cameras[i]);
		if (!camera.isOpened()) {
			cout << "Unable to open camera capture!" << endl;
			return -1;
		}

		captures[i] = camera;

		if (i == 0) {
			captures[0].read(src);
			img_size = src.size();
			img_size.height /= 2;
			img_size.width /= 2;
		}

		VideoWriter vidcapt;
		vidcapt.open(files[i],CV_FOURCC('P','I','M','1'),20,img_size,true);

		if ( !vidcapt.isOpened() ) {
		  cout << "ERROR: Failed to write the video" << endl;
		  return -1;
		}

		camera.read(src);
		resize(src, dst, img_size, 0, 0, INTER_CUBIC);
		vidcapt.write(dst);
		videos[i] = vidcapt;
	}

	bool running = true;

	while (running) {
		for (int i = 0; i < num_cameras; i++) {
			if (!captures[i].read(src)) {
				cout << "Unable to read from camera" << endl;
				break;
			}
			resize(src, dst, img_size, 0, 0, INTER_CUBIC);

			videos[i].write(dst);

			if (display) {
				imshow(names[i], dst);
				if (cvWaitKey(10) == 27) {
					running = false;
					break;
				}
			}
		}
	}

	// Release image arrays and window
	for (int i = 0; i < num_cameras; i++) {
		cvDestroyWindow(names[i]);
		captures[i].release();
		videos[i].release();

	}
	src.release();
	dst.release();

	return 0;
}
