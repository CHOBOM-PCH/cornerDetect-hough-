#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/opencv.hpp"
#include <vector>
#include <math.h>
#include <iostream>
#include "RANSAC_LineFittingAlgorithm.h"

#ifdef _DEBUG
        #pragma comment(lib,"opencv_core2413d.lib")
        #pragma comment(lib,"opencv_highgui2413d.lib")
        #pragma comment(lib,"opencv_imgproc2413d.lib")
#else
        #pragma comment(lib,"opencv_core2413.lib")
        #pragma comment(lib,"opencv_highgui2413.lib")
        #pragma comment(lib,"opencv_imgproc2413.lib")
#endif

#define PI 3.1415926
using namespace cv;

int houghCornerDetect(const char* route, int* pointCX, int* pointCY, int* pointX, int* pointY, float* conAngle, 
	float* distance, OutputArray _dst, int conerDirection);//conerDirection 중심점 기준으로 코너의 위치 : 좌상=1, 우상=2 우하=3 좌하=4
int detectMeetPoint (int asx, int asy, int aex, int aey, int bsx, int bsy, int bex, int bey, float* mx, float* my);
//, int* distanceX, int* distanceY, bool viewLine)