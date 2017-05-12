#include "houghConer.h"

int main()
{
	const char* k = "image/필름2.PNG";
	int distX = 0;
	int distY = 0;
	int x = 0;
	int y = 0;
	float angle = 0;

	int detect = houghCornerDetect(k, &x, &y, &angle);
	if (detect == 1){
		std::cout<<"찾아낸 포인트 x: "<<x<<" y: "<<y/*<<"중심에서 거리: X축으로 "
			<<distX<<"pixel Y축으로 "<<distY<<"pixel"*/<<std::endl;
		std::cout<<"찾아낸 포인트의 각도 degree: "<<angle<<std::endl;
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