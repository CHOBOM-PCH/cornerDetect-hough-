#include "houghConer.h"

int main()
{
	const char* k = "image/�ʸ�2.PNG";
	int cx = 0, cy = 0;
	int x = 0, y = 0;
	float angle = 0, dist = 0;
	cv::Mat image;

	int detect = houghCornerDetect(k, &cx, &cy, &x, &y, &angle, &dist, image, 4);
	if (detect == 1){
		std::cout<<"ã�Ƴ� ����Ʈ x: "<<cx<<" y: "<<cy<<std::endl;
		std::cout<<"ã�Ƴ� ����Ʈ�� ���� degree: "<<angle<<std::endl;
		std::cout<<"��ڳ� X : "<<x<<" , � �ڳ� Y : "<<y <<" , �Ÿ� : "<<dist<<std::endl;
		cv::imshow("ended",image);
		cv::waitKey(0);
	}else if (detect = -1){
		printf("ã���� ����\n");
		cv::waitKey(0);
		system("pause");
	}else { 
		printf("point ����\n");
		cv::waitKey(0);
	}

}