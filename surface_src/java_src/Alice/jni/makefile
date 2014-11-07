# Define a variable for classpath
CLASS_PATH = ../bin

# Define a virtual path for .class in the bin directory
vpath %.class $(CLASS_PATH)

all : robobuggy_vision.dll

# $@ matches the target, $< matches the first dependancy
robobuggy_vision.dll : RoboBuggy_Vision.c RoboBuggy_Vision_Impl.cpp
	g++ -m64 -Wl,--add-stdcall-alias -I"%JAVA_HOME%\include" -I"%JAVA_HOME%\include\win32" -shared -o $@ $<

# $* matches the target filename without the extension
VisionSystem.h : VisionSystem.class
	javah -classpath $(CLASS_PATH) $*

clean :
	rm VisionSystem.h RoboBuggy_Vision.o robobuggy_vision.dll

# $* matches the target filename without the extension
VisionSystem.h : VisionSystem.class
	javah -classpath $(CLASS_PATH) $*