#include <jni.h>
#include "RoboBuggy_Vision.h"
#include "VisionSystem.h"
#include <string.h>

JNIEnv *g_env;
jobject g_obj;
jmethodID g_mid;

JNIEXPORT jint JNICALL Java_CameraJNI_initVision (JNIEnv *env, jobject thisObj, 
	jintArray cameras, jobjectArray labels, jint length) {
	/*jint* camerasC = GetIntArrayElements(env, cameras, NULL);
	
	int count = GetArrayLength(env, labels);
	const char* labelsC[count];
	
	for (int i = 0; i < count; i++) {
		jstring tmp = (jstring)GetObjectArrayElement(env, labels, i);
		const char *tmp_char = GetStringUTFChars(env, tmp, 0);
		labelsC[i] = tmp_char;
		ReleaseStringUTFChars(env, tmp, tmp_char);
	}
	
	if (camerasC != NULL && labelsC != NULL) {
		return initVision(camerasC, labelsC, length);
	} else {
		return FAILURE;
	}*/
	
	return FAILURE;
}

JNIEXPORT jint JNICALL Java_CameraJNI_initVisionDefault (JNIEnv *env, jobject thisObj) {
	g_env = env;
	g_obj = env->NewGlobalRef(thisObj);
	
	// Get a class reference for this object
	jclass thisClass = env->GetObjectClass(thisObj);
	
	// Initialize Callback method
	g_mid = env->GetMethodID(thisClass, "callback", "()V");
	if (NULL == g_mid) return FAILURE;
	
	// Call back the method (which returns void), baed on the Method ID
	env->CallVoidMethod(thisObj, g_mid);

	int cameras[2] = {0};
	const char* labels[2] = {"Front"};
	int length = 1;
	
	return (jint)initVision(cameras, labels, length);
}

JNIEXPORT jint JNICALL Java_CameraJNI_disconnect(JNIEnv *, jobject) {
	return (jint)disconnect();
}

JNIEXPORT jint JNICALL Java_CameraJNI_startRecording(JNIEnv *env, jobject thisObj, jstring jdirectory) {
	const char *rawString = env->GetStringUTFChars(jdirectory, 0);
	if (NULL == rawString) {
		env->ReleaseStringUTFChars(jdirectory, rawString);
		return -1;
	}
	
	const char* directory = rawString;
	
	env->ReleaseStringUTFChars(jdirectory, rawString);
	
	return (jint)startRecording(rawString);
}

JNIEXPORT jint JNICALL Java_CameraJNI_stopRecording(JNIEnv *, jobject) {
	return (jint)stopRecording();
}

void callback(int* data, int length) {
	jintArray jdata = g_env->NewIntArray(length);
	g_env->SetIntArrayRegion(jdata, 0, length, (jint*)&data[0]);
	
	//g_env->CallVoidMethod(g_obj, g_mid, jdata);
}

/*void callback(int[] data) {
	JNIEnv * g_env;
	// double check it's all ok
	int getEnvStat = g_vm->GetEnv((void **)&g_env, JNI_VERSION_1_6);
	if (getEnvStat == JNI_EDETACHED) {
		std::cout << "GetEnv: not attached" << std::endl;
		if (g_vm->AttachCurrentThread((void **) &g_env, NULL) != 0) {
			std::cout << "Failed to attach" << std::endl;
		}
	} else if (getEnvStat == JNI_OK) {
		//
	} else if (getEnvStat == JNI_EVERSION) {
		std::cout << "GetEnv: version not supported" << std::endl;
	}

	g_env->CallVoidMethod(g_obj, g_mid, val);

	if (g_env->ExceptionCheck()) {
		g_env->ExceptionDescribe();
	}

	g_vm->DetachCurrentThread();
}*/
