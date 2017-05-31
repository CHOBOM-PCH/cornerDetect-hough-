#include "houghConer.h"

int Xall, Yall;

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
int detectMeetPoint (int asx, int asy, int aex, int aey, int bsx, int bsy, int bex, int bey, float* mx, float* my)
{
	float parallel = ((asx - aex) * (bsy - bey) - (asy - aey) * (bsx - bex));
	
	if (parallel == 0)
		return -1;
	else {
	*mx = ((asx * aey - asy * aex) * (bsx - bex) - (asx - aex) * (bsx * bey - bsy * bex)) 
		/ ((asx - aex) * (bsy - bey) - (asy - aey) * (bsx - bex));
	
	*my = ((asx * aey - asy * aex) * (bsy - bey) - (asy - aey) * (bsx * bey - bsy * bex)) 
		/ ((asx - aex) * (bsy - bey) - (asy - aey) * (bsx - bex));
	return 1;
	}

}
void getBressenHamLine(Point p1, Point p2, vector<Point> &kPoint)
{
	int dx, dy;
	int pValue;
	int inc2dy;
	int inc2dydx;
	int incValue;
	int ndx;
	dx = abs(p2.x - p1.x);
	dy = abs(p2.y - p1.y);

	if(dy <= dx) {
		inc2dy = 2 * dy;
		inc2dydx = 2 * (dy - dx);
		if (p2.x < p1.x) {
			ndx = p1.x;
			p1.x = p2.x;
			p2.x = ndx;

			ndx = p1.y;
			p1.y = p2.y;
			p2.y = ndx;
		}else if(p1.y < p2.y) {
			incValue = 1;
		}else {
			incValue = -1;
		}
		kPoint.push_back(p1);

		pValue = 2 * dy - dx;
		for (ndx = p1.x; ndx < p2.x; ndx++) {
			if (0 > pValue) {
				pValue += inc2dy;
			}else {
				pValue += inc2dydx;
				p1.y += incValue;
			}
			kPoint.push_back(Point(ndx, p1.y));
		}
	}else {
		inc2dy = 2 * dx;
		inc2dydx = 2 * (dx - dy);

		if (p2.y < p1.y) {
			ndx = p1.y;
			p1.y = p2.y;
			p2.y = ndx;

			ndx = p1.x;
			p1.x = p2.x;
			p2.x = ndx;
		}else if(p1.x < p2.x) {
			incValue = 1;
		}else {
			incValue = -1;
		}
		kPoint.push_back(p1);
		
		pValue = 2 * dx - dy;

		for (ndx = p1.y; ndx < p2.y; ndx++) {
			if (0 > pValue) {
				pValue += inc2dy;
			} else {
				pValue += inc2dydx;
				p1.x += incValue;
			}
			kPoint.push_back(Point(p1.x, ndx));
		}
	}
}



//void onMouse(int event, int x, int y, int flags, void* param)
//{
//	Mat *pMat = (Mat *)param;
//	Mat image = Mat(*pMat);
//	int distX, distY;
//	float dist;
//	if( param == NULL )
//	return;
//	switch (event)
//	{
//	case EVENT_LBUTTONDOWN:
//		if (flags & EVENT_FLAG_SHIFTKEY)
//			rectangle(image, Point(x - 5, y - 5), Point(x + 5, y + 5), Scalar(255, 0, 0));
//		else {
//			circle(image, Point(x, y), 5, Scalar(0, 0, 255), 5);
//			std::cout<<"x좌표"<<x << ", "<<"y좌표점 "<<y<<std::endl;
//			distX = Xall - x;
//			distY = Yall - y;
//			dist = sqrt((float)(pow((float)distX,2) + pow((float)distY,2)));
//			std::cout<<"거리 x"<<distX<< ", 거리 y "<<distY<<", 최종거리 "<<dist<<std::endl;
//		}
//		break;
//	/*case EVENT_RBUTTONDOWN:
//		circle(image, Point(x, y), 5, Scalar(255, 0, 0), 5);
//		break;
//	case EVENT_LBUTTONDBLCLK:
//		image = Scalar(255, 255, 255);
//		break;*/
//	}
//	imshow("dstImage", image);
//}


int houghCornerDetect(const char* route, int* pointCX, int* pointCY, int* pointX, int* pointY, float* conAngle, float* distance, OutputArray _dst, int conerDirection)
{
	Mat input_img = imread(route);
	Mat gray_img;
	Mat blur_img;
	Mat threshOutput_img;
	Mat threshOutput2_img;
	Mat edge_img;
	Mat kernel = Mat::ones(3, 3, CV_8U);
	Mat output_img = input_img.clone();
	_dst.getMatRef() = output_img;
	cvtColor(input_img, gray_img, CV_BGR2GRAY);
	
	Mat erode1_img = gray_img.clone();
	erodeTest(gray_img, erode1_img, kernel);
	Mat erode2_img = gray_img.clone();
	erodeTest(erode1_img, erode2_img, kernel);
	GaussianBlur(erode2_img, blur_img, cv::Size(7,7), 3);
	
	adaptiveThreshold(blur_img, threshOutput_img, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY_INV, 15, 2);
	threshold(erode2_img, threshOutput2_img, 100, 255, THRESH_OTSU);

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
			if (aa > 10) {
				if (angleB == 365) {
					angleB = angle;
					lineB.push_back(lines[k]);
				} else {
					int bb = angleB - angle;
					if (bb > 1);//수정할 수 있음
					else { 
						lineB.push_back(lines[k]);
					}
				}
			} else {
				lineA.push_back(lines[k]);
			}
		}
					
				
		//printf("시작점 %d, %d  끝점 %d %d  각도 %lf \n", x1, y1, x2, y2, angle);
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

	int xsa = aLine.sx - 500 * aLine.mx;
	int ysa = aLine.sy - 500 * aLine.my;
	int xea = aLine.sx + 500 * aLine.mx;
	int yea = aLine.sy + 500 * aLine.my;
	int xsb = bLine.sx - 500 * bLine.mx;
	int ysb = bLine.sy - 500 * bLine.my;
	int xeb = bLine.sx + 500 * bLine.mx;
	int yeb = bLine.sy + 500 * bLine.my;
	
	float meetX, meetY, meetfX, meetfY;
	detectMeetPoint(xsa, ysa, xea, yea, xsb, ysb, xeb, yeb, &meetX, &meetY);
	float angleMeet = fabs(angleA - angleB);

	int axs, axe, ays, aye;
	switch (conerDirection) {
	case 1:
		axs = (int)meetX ;//- 500 * (aLine.mx + bLine.mx);
		axe = (int)meetX + 100 * (aLine.mx + bLine.mx);
		ays = (int)meetY ;//- 500 * (aLine.my + bLine.my);
		aye = (int)meetY + 100 * (aLine.my + bLine.my);
		break;
	case 2:
		axs = (int)meetX ;//- 500 * (aLine.mx + bLine.mx);
		axe = (int)meetX + 100 * (aLine.mx - bLine.mx);
		ays = (int)meetY ;//- 500 * (aLine.my + bLine.my);
		aye = (int)meetY + 100 * (aLine.my - bLine.my);
		break;
	case 3:
		axs = (int)meetX - 100 * (aLine.mx + bLine.mx);
		axe = (int)meetX;// + 100 * (aLine.mx + bLine.mx);
		ays = (int)meetY - 100 * (aLine.my + bLine.my);
		aye = (int)meetY;// + 100 * (aLine.my + bLine.my);
		break;
	case 4:
		axs = (int)meetX - 100 * (aLine.mx - bLine.mx);
		axe = (int)meetX;// + 100 * (aLine.mx + bLine.mx);
		ays = (int)meetY - 100 * (aLine.my - bLine.my);
		aye = (int)meetY;// + 100 * (aLine.my + bLine.my);
		break;
	}

	
	
	line(output_img, Point(xsa, ysa), Point(xea, yea), Scalar(0, 0, 255), 2);
	line(output_img, Point(xsb, ysb), Point(xeb, yeb), Scalar(255, 255, 0), 2);
	line(output_img, Point(axs, ays), Point(axe, aye), Scalar(0, 150, 100), 2);
	
	vector<Point> linePoint;
	vector<Point> fitPoint;
	getBressenHamLine(Point(axs, ays), Point(axe, aye), linePoint);
	int kk = 0, kkk = 0, dd = 0;
	for (int ii = 0; ii < linePoint.size(); ii++) {
		int fitColor = 111;
		if (linePoint.at(ii).x <= output_img.cols && linePoint.at(ii).y <= output_img.rows && linePoint.at(ii).x >= 0 && linePoint.at(ii).y >= 0)
			fitColor = threshOutput2_img.at<uchar>(linePoint.at(ii));
		else
			break;
		//std::cout << "색상값" <<linePoint.at(ii).x<<", "<< linePoint.at(ii).y <<" : "<<fitColor<<std::endl;
		if (fitColor == 255 && dd == 0){
			kk++;
		} else { 
			if (kk > 5){
				if (fitColor == 0 && kkk == 0){
					fitPoint.push_back(Point(linePoint.at(ii).x,linePoint.at(ii).y));
					dd++;
				}else {
					if (dd > 5) {
						if(kkk > 2)
							break;
						else if(fitColor == 255)
							kkk++;
						else {
							kk = 0;
							dd = 0;
							kkk = 0;
							fitPoint.clear();
						}
					} else {
						kk = 0;
						dd = 0;
						kkk = 0;
						fitPoint.clear();
					}

				}
			}
		}
	}
	*pointCX = (int)meetX;
	*pointCY = (int)meetY;
	*conAngle = angleMeet;
	Xall = meetX;
	Yall = meetY;
	circle(output_img, Point(meetX, meetY), 5, Scalar(0, 255, 0), 2);
	dd = fitPoint.size() - 1;
	//line(output_img, Point(fitPoint.at(0).x,fitPoint.at(0).y) , Point(fitPoint.at(dd).x, fitPoint.at(dd).y) , Scalar(0, 0, 255), 2);
	if (fitPoint.size() > 6) {
		int fitX, fitY;
		*pointX = fitX = (fitPoint.at(0).x + fitPoint.at(dd).x)/2;
		*pointY = fitY = (fitPoint.at(0).y + fitPoint.at(dd).y)/2;


		int distX = Xall - fitX;
		int distY = Yall - fitY;
		*distance = sqrt((float)(pow((float)distX,2) + pow((float)distY,2)));
		//std::cout<<"곡선코너 X : "<<fitX<<" , 곡선 코너 Y : "<<fitY <<" , 거리 : "<<*distance<<std::endl;
		line(output_img, Point((int)meetX, (int)meetY), Point(fitX, fitY), Scalar(0, 150, 100), 2);
		circle(output_img, Point(fitX, fitY), 5, Scalar(0, 0, 255), 2);
	}
	delete aDirection;
	delete bDirection;
	imshow("origin", threshOutput2_img);
	//imshow("dstImage", output_img);
	//setMouseCallback("dstImage", onMouse, NULL);// (void *)&output_img);
	//waitKey(0);
	return 1;

}