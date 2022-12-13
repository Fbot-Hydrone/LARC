#pragma once

#include <image_transport/image_transport.h>
#include <opencv4/opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Float64.h>
#include "std_msgs/Float64MultiArray.h"
#include <math.h>
#include <ros/package.h>

// Limiares da cor azul ( Imagem HSV )
#define MINBLUE         95
#define MAXBLUE         135

#define MINSATBLUE      50
#define MAXSATBLUE      255

#define MINVALBLUE      50
#define MAXVALBLUE      255

// Limiares da cor amarela ( Imagem HSV )
#define MINYELLOW       20
#define MAXYELLOW       45

#define MINSATYELLOW    50
#define MAXSATYELLOW    255

#define MINVALYELLOW    50
#define MAXVALYELLOW    255

#define cvCOLOR_RED Scalar(0, 0, 255)
#define cvCOLOR_GREEN Scalar(0, 255, 0)

// Parametros de filtros
#define GAUSSIANFILTER 3
#define KERNELSIZE 9

using namespace cv;
using namespace std;

struct CameraImage{
    int rows, cols;
    int centerX, centerY;
};

class BaseDetector
{
public:
    //// Variaveis ////
    Mat imageMainC3, imageMainHsvC3, imageBlueC1, imageYellowC1, imageFinalC1;
    Mat morph_kernel, mask;

    CameraImage cameraImage;

    int majorEllipseWidth, majorEllipseHeight;
	int minEllipseWidth, minEllipseHeight;

    bool success, fstTime;

	RotatedRect majorEllipse;
	Rect mark;
	Rect closestBase;

    vector<vector<Point>> contours;
	vector<Rect> basesList;

	float x, y;
	
	BaseDetector();

	Mat imageFill(Mat img);
	Mat imageLimiares(Mat hsv, int hsvMin[3], int hsvMax[3]);

	void setCameraParams(Mat img);
	void setImage(Mat img);
	void processImage();
	void drawClosestBase();
	void setClosestBaseDistance();
	void drawSquare();
	void loadMask(string maskPath);
	void applyMask();
	void show();

	bool findSquare();
	bool existBaseAround();
};
