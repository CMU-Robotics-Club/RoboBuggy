REM an example of using haar cascade recognition for face and eye detection.
facedetect --cascade="../../data/haarcascades/haarcascade_frontalface_alt.xml" --nested-cascade="../../data/haarcascades/haarcascade_eye.xml" --scale=1.3 %1
