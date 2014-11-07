#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "RoboBuggy_Vision.h"
#include <fstream>
#include <string>

#define ESCAPE 27

using namespace std;
using namespace cv;

int ids[MAX_CAMERAS];
const char* labels[MAX_CAMERAS];
VideoCapture feeds[MAX_CAMERAS];
VideoWriter writers[MAX_CAMERAS];
int num_cameras;
bool recording;
bool showing;
bool connected;
Size img_size;

string getFilename(string directory, int id) {
	int i  = 0;
	string path = directory;
	path.append("run");

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

int setup(int* cameras, string* names, int length) {
	img_size = Size(WIDTH, HEIGHT);
	recording = false;
	showing = false;
	connected = false;
	num_cameras = length;
	
	for (int i = 0; i < num_cameras; i++) {
		ids[i] = cameras[i];
		labels[i] = names[i].c_str();
	
		VideoCapture camera(ids[i]);
		if (!camera.isOpened()) {
			cout << "Unable to open camera " << ids[i] << endl;
			return FAILURE;
		}
		
		cout << "Opened feed for camera " << ids[i] << endl;
		
		feeds[i] = camera;

		cvNamedWindow(labels[i], ids[i]);
	}
	
	connected = true;
	showing = true;
	
	int data[3] = {1,2};
	callback(data, 2);

	return SUCCESS;
}

int run() {
	Mat src,dst;
	
	while(connected) {
	
		for (int i = 0; i < num_cameras; i++) {
			if (!feeds[i].read(src)) {
				cout << "Failed to read frame from camera: " << ids[i] << endl;
				connected = false;
				return FAILURE;
			}
			
			resize(src, dst, img_size, 0, 0, INTER_CUBIC);
			
			if (showing) imshow(labels[i], dst);
			if (recording) writers[i].write(dst);
			
			if (cvWaitKey(10) == ESCAPE) connected = false;
		}
	}
	
	src.release();
	dst.release();
	
	for(int i = 0; i < num_cameras; i++) {
		feeds[i].release();
		if (recording) writers[i].release();
	}
	
	cout << "Disconnected" << endl;
	return SUCCESS;
}

int disconnect() {
	connected = false;
	
	return SUCCESS;
}

int startRecording(const char* dir) {
	if (!connected || recording || NULL == dir) return FAILURE;
	
	string directory(dir);

	for (int i = 0; i < num_cameras; i++) {
		cout << "Searching for filename" << endl; 
		string filename = getFilename(directory, ids[i]);
		cout << "Found filename " << filename << endl;
		
		VideoWriter writer(filename.c_str(), CV_FOURCC('M','J','P','G'), 20, img_size, true);
		if (!writer.isOpened()) {
			cout << "Failed to open file" << filename << endl;
			return -1;
		}
		writers[i] = writer;
	}
	
	recording = true;
	return SUCCESS;
}

int stopRecording() {
	if (connected && recording) {
		for(int i = 0; i < num_cameras; i++) {
			writers[i].release();
			cout << "Closed file for camera " << ids[i] << endl;
		}
	}
	
	recording = false;
	return SUCCESS;
}

int initVision(int cameras[], const char* labels[], int length) {
	cout << "Starting JNI C++ Vision Thread" << endl;
	
	string names[length];
	for (int i = 0; i < length; i++) {
		string tmp(labels[i]);
		names[i] = tmp;
	}
	
	int retVal = setup(cameras, names, length);
	if (retVal < 0) return retVal;
	
	return run();
}

int initVisionDefault() {
	cout << "Starting JNI C++ Vision Thread" << endl;
	
	int cameras[2] = {0};
	string labels[2] = {"Front"};
	int length = 1;
	
	int retVal = setup(cameras, labels, length);
	if (retVal < 0) return retVal;
	
	return run(); 
}