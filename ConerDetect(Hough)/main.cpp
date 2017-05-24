#include "houghConer.h"

int main()
{
	const char* k = "image/필름2.PNG";
	int cx = 0, cy = 0;
	int x = 0, y = 0;
	float angle = 0, dist = 0;
	cv::Mat image;

	int detect = houghCornerDetect(k, &cx, &cy, &x, &y, &angle, &dist, image, 4);
	if (detect == 1){
		std::cout<<"찾아낸 포인트 x: "<<cx<<" y: "<<cy<<std::endl;
		std::cout<<"찾아낸 포인트의 각도 degree: "<<angle<<std::endl;
		std::cout<<"곡선코너 X : "<<x<<" , 곡선 코너 Y : "<<y <<" , 거리 : "<<dist<<std::endl;
		cv::imshow("ended",image);
		cv::waitKey(0);
	}else if (detect = -1){
		printf("찾을수 없음\n");
		cv::waitKey(0);
		system("pause");
	}else { 
		printf("point 많음\n");
		cv::waitKey(0);
	}

}