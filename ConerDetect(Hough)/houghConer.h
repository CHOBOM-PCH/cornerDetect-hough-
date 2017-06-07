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
	float* distance, OutputArray _dst, int conerDirection);//conerDirection �߽��� �������� �ڳ��� ��ġ : �»�=1, ���=2 ����=3 ����=4
int detectMeetPoint(Point aStart,//line a�� ������
					Point aEnd,//line a�� ����
					Point bStart,//line b�� ������
					Point bEnd,//line b�� ����
					float* mx, float* my);//a�� b�� ������
					
void erodeTest(Mat &src, //�� ����
			Mat &dst,//ħ�ĵȿ���
			Mat &kernel);//�����ֺ�Ž�� ����
void getBressenHamLine(Point st,//Ž���� ���� ������
					Point ed,//Ž���� ���� ����
					vector<Point> &kPoint);//Ž���� ���� �������Ʈ

