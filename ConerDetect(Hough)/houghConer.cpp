#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
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

void erodeTest(Mat &src, Mat &dst, Mat &kernel)
{
	int iMin, iVal;
	for (int i = 0; i < src.rows - 2; i++)	{
		
		for (int j = 0; j < src.cols - 2; j++) {
			
			iMin = 0xfff;
			
			for (int ii = 0; ii < kernel.rows; ii++) {
				
				for (int jj = 0; jj < kernel.cols; jj++) {
					
					if (kernel.at<uchar>(ii, jj)) { 
						
						iVal = src.at<uchar>(i + ii, j + jj);
						
						if (iMin > iVal)
							iMin = iVal;
					}
				}
			}

			dst.at<uchar>(i + 1, j + 1) = iMin;

		}
	}
}


int main()
{
	Mat input_img = imread("image/필름1.PNG");
	Mat gray_img;
	Mat blur_img;
	Mat threshOutput_img;
	Mat edge_img;
	Mat kernel = Mat::ones(3, 3, CV_8U);
	Mat output_img = input_img.clone();
	cvtColor(input_img, gray_img, CV_BGR2GRAY);
	
	Mat erode1_img = gray_img.clone();
	erodeTest(gray_img, erode1_img, kernel);
	Mat erode2_img = gray_img.clone();
	erodeTest(erode1_img, erode2_img, kernel);
	GaussianBlur(erode2_img, blur_img, cv::Size(7,7), 3);
	
	adaptiveThreshold(blur_img, threshOutput_img, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY_INV, 15, 2);
	//threshold(erode2_img, threshOutput_img, 100, 255, THRESH_OTSU);

	Canny(threshOutput_img, edge_img, 250, 300);

	vector<Vec4i> lines;
	vector<Vec4i> lineA, lineB;

	HoughLinesP(threshOutput_img, lines, 1, (PI / 180), 50, 100, 20);
	
	Vec4d params;//, avgparams;
	
	int x1, y1, x2, y2;
	double angle = 0;
	double angleA = 0;
	double angleB = 365;
	for (int k = 0; k < lines.size(); k++){
		params = lines[k];
		x1 = params[0];
		y1 = params[1];
		x2 = params[2];
		y2 = params[3];
		angle = abs(atan2f(((float)(y1 - y2)), abs((float)(x2 - x1))) * 180 / PI);
		Point pt1(x1, y1),pt2(x2, y2);
		//line(output_img, pt1, pt2, Scalar(255, 0, 255), 1);
		if (k == 0) {
			angleA = angle;
			lineA.push_back(lines[k]);
		} else {
			int aa = angleA - angle;
			if (aa > 1) {
				if (angleB == 365) {
					angleB = angle;
					lineB.push_back(lines[k]);
				} else {
					int bb = angleB - angle;
					if (bb > 1);
					else { 
						lineB.push_back(lines[k]);
					}
				}
			} else {
				lineA.push_back(lines[k]);
			}
		}
					
				
		printf("시작점 %d, %d  끝점 %d %d  각도 %lf \n", x1, y1, x2, y2, angle);
	}
	sPoint *aDirection = new sPoint[lineA.size() * 2];
	sPoint *bDirection = new sPoint[lineB.size() * 2];
	sLine aLine, bLine;
	for (int k = 0; k < lineA.size(); k++) {
		params = lineA[k];
		int i = (k + 1) * 2;
		aDirection[i - 2].x = params[0];
		aDirection[i - 2].y = params[1];
		aDirection[i - 1].x = params[2];
		aDirection[i - 1].y = params[3];
	}
	for (int k = 0; k < lineB.size(); k++) {
		params = lineB[k];
		int i = (k + 1) * 2;
		bDirection[i - 2].x = params[0];
		bDirection[i - 2].y = params[1];
		bDirection[i - 1].x = params[2];
		bDirection[i - 1].y = params[3];
	}
	double costA = ransac_line_fitting (aDirection, lineA.size() * 2, aLine, 30);
	double costB = ransac_line_fitting (bDirection, lineB.size() * 2, bLine, 30);

	line(output_img, Point(aLine.sx - 500 * aLine.mx, aLine.sy - 500 * aLine.my), 
		Point(aLine.sx + 500 * aLine.mx, aLine.sy + 500 * aLine.my), Scalar(0, 0, 255), 2);
	line(output_img, Point(bLine.sx - 500 * bLine.mx, bLine.sy - 500 * bLine.my),
		Point(bLine.sx + 500 * bLine.mx, bLine.sy + 500 * bLine.my), Scalar(255, 255, 0), 2);

	delete aDirection;
	delete bDirection;


	imshow("edge", threshOutput_img);
	imshow("image",output_img);
	waitKey(0);

}