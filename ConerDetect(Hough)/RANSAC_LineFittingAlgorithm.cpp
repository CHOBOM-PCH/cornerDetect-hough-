#include <math.h>
#include <stdlib.h>
#include "RANSAC_LineFittingAlgorithm.h"


bool find_in_samples (sPoint *samples, int no_samples, sPoint *data)
{
	for (int i=0; i<no_samples; ++i) {
		if (samples[i].x == data->x && samples[i].y == data->y) {
			return true;
		}
	}
	return false;
}

void get_samples (sPoint *samples, int no_samples, sPoint *data, int no_data)
{
	// 데이터에서 중복되지 않게 N개의 무작위 셈플을 채취한다.
	for (int i=0; i<no_samples; ) {
		int j = rand()%no_data;
		
		if (!find_in_samples(samples, i, &data[j])) {
			samples[i] = data[j];
			++i;
		}
	};
}

int compute_model_parameter(sPoint samples[], int no_samples, sLine &model)
{
	// PCA 방식으로 직선 모델의 파라메터를 예측한다.

	double sx  = 0, sy  = 0;
	double sxx = 0, syy = 0;
	double sxy = 0, sw  = 0;

	for (int i = 0; i<no_samples;++i)
	{
		double &x = samples[i].x;
		double &y = samples[i].y;

		sx  += x;	
		sy  += y;
		sxx += x*x; 
		sxy += x*y;
		syy += y*y;
		sw  += 1;
	}

	//variance;
	double vxx = (sxx - sx*sx/sw)/sw;
	double vxy = (sxy - sx*sy/sw)/sw;
	double vyy = (syy - sy*sy/sw)/sw;
	
	//principal axis
	double theta = atan2(2*vxy, vxx - vyy)/2;
	
	model.mx = cos(theta);
	model.my = sin(theta);
	
	//center of mass(xc, yc)
	model.sx = sx/sw;
	model.sy = sy/sw;
	
	//직선의 방정식: sin(theta)*(x - sx) = cos(theta)*(y - sy);
	return 1;
}

double compute_distance(sLine &line, sPoint &x)
{
	// 한 점(x)로부터 직선(line)에 내린 수선의 길이(distance)를 계산한다.

	return fabs((x.x - line.sx)*line.my - (x.y - line.sy)*line.mx)/sqrt(line.mx*line.mx + line.my*line.my);
}

double model_verification (sPoint *inliers, int *no_inliers, sLine &estimated_model, sPoint *data, int no_data, double distance_threshold)
{
	*no_inliers = 0;

	double cost = 0.;

	for (int i=0; i<no_data; i++){
		// 직선에 내린 수선의 길이를 계산한다.
		double distance = compute_distance(estimated_model, data[i]);
	
		// 예측된 모델에서 유효한 데이터인 경우, 유효한 데이터 집합에 더한다.
		if (distance < distance_threshold) {
			cost += 1.;

			inliers[*no_inliers] = data[i];
			++(*no_inliers);
		}
	}

	return cost;
}

double ransac_line_fitting(sPoint *data, int no_data, sLine &model, double distance_threshold)
{
	const int no_samples = 2;

	if (no_data < no_samples) {
		return 0.;
	}

	sPoint *samples = new sPoint[no_samples];

	int no_inliers = 0;
	sPoint *inliers = new sPoint[no_data];

	sLine estimated_model;
	double max_cost = 0.;

	int max_iteration = (int)(1 + log(1. - 0.99)/log(1. - pow(0.5, no_samples)));

	for (int i = 0; i<max_iteration; i++) {
		// 1. hypothesis

		// 원본 데이터에서 임의로 N개의 셈플 데이터를 고른다.
		get_samples (samples, no_samples, data, no_data);

		// 이 데이터를 정상적인 데이터로 보고 모델 파라메터를 예측한다.
		compute_model_parameter (samples, no_samples, estimated_model);

		// 2. Verification

		// 원본 데이터가 예측된 모델에 잘 맞는지 검사한다.
		double cost = model_verification (inliers, &no_inliers, estimated_model, data, no_data, distance_threshold);

		// 만일 예측된 모델이 잘 맞는다면, 이 모델에 대한 유효한 데이터로 새로운 모델을 구한다.
		if (max_cost < cost) {
			max_cost = cost;
	
			compute_model_parameter (inliers, no_inliers, model);
		}
	}
	
	delete [] samples;
	delete [] inliers;

	return max_cost;
}
