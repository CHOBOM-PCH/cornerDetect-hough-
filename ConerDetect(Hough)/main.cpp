#include "houghConer.h"

int main()
{
	const char* k = "image/�ʸ�2.PNG";
	int distX = 0;
	int distY = 0;
	int x = 0;
	int y = 0;
	float angle = 0;

	int detect = houghCornerDetect(k, &x, &y, &angle);
	if (detect == 1){
		std::cout<<"ã�Ƴ� ����Ʈ x: "<<x<<" y: "<<y/*<<"�߽ɿ��� �Ÿ�: X������ "
			<<distX<<"pixel Y������ "<<distY<<"pixel"*/<<std::endl;
		std::cout<<"ã�Ƴ� ����Ʈ�� ���� degree: "<<angle<<std::endl;
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