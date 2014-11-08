#ifndef _ROBOBUGGY_VISION_H
#define _ROBOBUGGY_VISION_H

#include <string>
#define FAILURE -1
#define SUCCESS 0
#define MAX_CAMERAS 2
#define WIDTH 640
#define HEIGHT 480

#ifdef __cplusplus
	extern "C" {
#endif

	int initVisionDefault();
	int initVision(int* cameras, const char** labels, int length);
	std::string getFilename(std::string directory, int id);
	int setup(int* cameras, char** names, int length);
	int run();
	int disconnect();
	int startRecording(const char* directory);
	int stopRecording();
	void callback(int* data, int length);
	
#ifdef __cplusplus
	}
	
#endif

#endif