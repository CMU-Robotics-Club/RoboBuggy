#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cstdlib>
#include <string>
#include <vector>
#include <sstream>

using namespace std;
using namespace cv;

struct camera_t {
	const int id;
	const string name;
	VideoCapture feed;

	camera_t(int id_,string name_,VideoCapture feed_) :
		id(id_),name(name_),feed(feed_) {}
	~camera_t() {
		feed.release();
	}
};

int main(int argc, char* argv[]) {
	vector<camera_t> cameras;
	int num_cameras = 0;
	Mat src,dst;
	Size img_size;
	bool running = true;

	cout << argc << " arguments" << endl;
//
//	// Parse Command Line Arguments
//	for (int i = 0; i < argc; i++) {
//		cout << "arg[" << i << "] = " << argv[i] << endl;
//		if (argv[i][0] == '-' && argv[i][1] == 'c') {
//			if (num_cameras < 5 && i + 1 < argc) {
//				int tmp = (int)argv[++i][0];
//				if (tmp >= 48 && tmp <= 57) {
//					cout << "Worked" << endl;
//					cameras[num_cameras++] = tmp - 48;
//				}
//			}
//		}
//	}

//	if (num_cameras == 0) return 0;
	cout << "Opening " << num_cameras << " cameras" << endl;

	bool anyOpen = false;
	for (int i = 0; i < 10; i++) {
		VideoCapture camera(i);
		if (!camera.isOpened()) {
			cout << "Unable to open camera capture " << i << "!" << endl;
			continue;
		}
		anyOpen = true;

		if (i == 0) {
			camera.read(src);
			img_size = src.size();
			img_size.height /= 2;
			img_size.width /= 2;
		}

		stringstream s;
		s << "Camera " << i;
		camera_t c{ i,s.str(),camera };
		cameras.push_back(c);

		cvNamedWindow(c.name.c_str(), c.id);
	}
	cout << cameras.size() << " cameras" << endl;

	while (running) {
		for (camera_t c: cameras) {
			if (!c.feed.read(src)) {
				cout << "Unable to read from camera" << endl;
				break;
			}
			resize(src, dst, img_size, 0, 0, INTER_CUBIC);

			imshow(c.name, dst);
			if (cvWaitKey(10) == 27) {
				running = false;
			}
		}
	}


	src.release();
	dst.release();

	return 0;
}
