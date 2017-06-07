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
int detectMeetPoint(Point aStart,//line a의 시작점
					Point aEnd,//line a의 끝점
					Point bStart,//line b의 시작점
					Point bEnd,//line b의 끝점
					float* mx, float* my);//a와 b의 교차점
					
void erodeTest(Mat &src, //들어간 영상
			Mat &dst,//침식된영상
			Mat &kernel);//영상주변탐색 필터
void getBressenHamLine(Point st,//탐색할 선의 시작점
					Point ed,//탐색할 선의 끝점
					vector<Point> &kPoint);//탐색한 선의 모든포인트

