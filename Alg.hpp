#include <opencv2/opencv.hpp>
#include <iostream>
#include <math.h>

ing namespace std;
using namespace cv;


void init();

void loadImg(const string&, Mat &);

void resizeImg(const Mat&, Mat&, float);

void prepWork(Mat&);

void save(Mat &img, const string&);

vector<int> compression_params;

