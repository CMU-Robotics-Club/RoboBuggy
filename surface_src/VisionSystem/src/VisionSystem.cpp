#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cstdlib>

using namespace std;
using namespace cv;

int main(int argc, char* argv[]) {
	int cameras[] = {0,0,0,0,0};
	char* names[] = {"camera1", "camera2", "camera3", "camera4", "camera5"};
	int num_cameras = 0;
	Mat src,dst;
	VideoCapture feeds[5];
	Size img_size;
	bool running = true;

	// Parse Command Line Arguments
	for (int i = 0; i < argc; i++) {
		if (argv[i][0] == '-' && argv[i][1] == 'c') {
			if (num_cameras < 5 && i + 1 < argc) {
				int tmp = (int)argv[i+1][0];
				if (tmp >= 48 && tmp <= 57) {
					cout << "Worked" << endl;
					cameras[num_cameras++] = tmp - 48;
				}
			}
		}
	}

	if (num_cameras == 0) return 0;

	for (int i = 0; i < num_cameras; i++) {
		VideoCapture camera(cameras[i]);
		if (!camera.isOpened()) {
			cout << "Unable to open camera capture!" << endl;
			return -1;
		}

		if (i == 0) {
			camera.read(src);
			img_size = src.size();
			img_size.height /= 2;
			img_size.width /= 2;
		}
		feeds[i] = camera;

		cvNamedWindow(names[i], cameras[i]);
	}

	while (running) {
		for (int i = 0; i < num_cameras; i++) {
			if (!feeds[i].read(src)) {
				cout << "Unable to read from camera" << endl;
				break;
			}
			resize(src, dst, img_size, 0, 0, INTER_CUBIC);

			imshow(names[i], dst);
			if (cvWaitKey(10) == 27) {
				running = false;
			}
		}
	}

	for(int i = 0; i < num_cameras; i++) {
		feeds[i].release();
	}
	src.release();
	dst.release();

	return 0;
}
