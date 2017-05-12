#pragma once

struct sPoint {
	double x, y;
};
struct sPointi{
	int x, y;
};

struct sLine {
	double mx, my;
	double sx, sy;
};

double ransac_line_fitting(sPoint *data, int no_data, sLine &model, double distance_threshold);
