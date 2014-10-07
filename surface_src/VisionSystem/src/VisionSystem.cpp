#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cstdlib>
#include <string>
#include <fstream>
#include <csignal>

using namespace std;
using namespace cv;

string getFilename(int id) {
	int i  = 0;
	string path = "C:\\Users\\abc\\buggy-log\\run";

	for (;;) {
		string filename;
		filename.append(path);
		stringstream strs;
		strs << i;
		filename.append(strs.str());
		filename.append("-cam");
		stringstream ids;
		ids << id;
		filename.append(ids.str());
		filename.append(".avi");

		ifstream file(filename.c_str());
		if (file.good()) {
			file.close();
		} else {
			file.close();
			return filename;
		}

		i++;
	}

	return NULL;

}

int num_cameras;
Mat src,dst;
VideoCapture feeds[5];

void signalHandler(int signum)
{
std::cout<<"Interrupt signal ("<<signum <<") received.\n";
for(int i = 0; i < num_cameras; i++) {
	feeds[i].release();
}

src.release();
dst.release();
}




int main(int argc, char* argv[]) {

	//setsup signal handler so that the program can exit gracefully
	signal(SIGINT,signalHandler);

	int cameras[5];
	char* names[] = {"camera1", "camera2", "camera3", "camera4", "camera5"};
	num_cameras = 0;
	VideoWriter files[5];
	Size img_size;
	bool running = true;

	cout << "Argc : " << argc << endl;

	// Parse Command Line Arguments
	for (int i = 0; i < argc; i++) {
		if (argv[i][0] == '-' && argv[i][1] == 'c') {
			if (i + 1 < argc) {
				int tmp = (int)argv[i + 1][0];
				if (tmp >= 48 && tmp <= 57) {
					cameras[num_cameras++] = tmp - 48;
				}
			}
		}
	}

	cout << "Opening " << num_cameras << " cameras" << endl;
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
			//img_size.height /= 2;
			//img_size.width /= 2;
		}
		feeds[i] = camera;

		cvNamedWindow(names[i], cameras[i]);

		string filename = getFilename(cameras[i]);
		cout << "Creating file: " << filename << " for camera: " << cameras[i] << endl;

		VideoWriter file(filename.c_str(), CV_FOURCC('M','J','P','G'), 20, img_size, true);
		if (!file.isOpened()) {
			cout << "Failed to open file" << endl;
			return -1;
		}
		files[i] = file;
	}

	while (running) {
		for (int i = 0; i < num_cameras; i++) {
			if (!feeds[i].read(src)) {
				cout << "Failed to read frame from camera: " << cameras[i] << endl;
				return -1;
			}
			resize(src, dst, img_size, 0, 0, INTER_CUBIC);

			imshow(names[i], dst);
			files[i].write(dst);
			if (cvWaitKey(10) > 0) {
				running = false;
			}
		}
	}

	for(int i = 0; i < num_cameras; i++) {
		feeds[i].release();
	}

	src.release();
	dst.release();
}
