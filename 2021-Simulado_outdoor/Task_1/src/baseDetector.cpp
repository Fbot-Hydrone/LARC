#include "../include/BaseDetector.hpp"

int ARR_MAXBLUE[3] = {MAXBLUE, MAXSATBLUE, MAXVALBLUE};
int ARR_MINBLUE[3] = {MINBLUE, MINSATBLUE, MINVALBLUE};

int ARR_MAXYELLOW[3] = {MAXYELLOW, MAXSATYELLOW, MAXVALYELLOW};
int ARR_MINYELLOW[3] = {MINYELLOW, MINSATYELLOW, MINVALYELLOW};

BaseDetector::BaseDetector()
{	
	this->morph_kernel = Mat::ones(KERNELSIZE, KERNELSIZE, CV_8U);

	this->success = false;
	this->fstTime = true;
}

Mat BaseDetector::imageFill(Mat img)
{
	morphologyEx(img, img, MORPH_CLOSE, morph_kernel, Point(-1, -1), 2);
	findContours(img, this->contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

	vector<vector<Point>> hull( this->contours.size() );

	for( size_t i = 0; i < this->contours.size(); i++ ) {
		convexHull( this->contours[i], hull[i] );
	}

	float biggestArea = 0;
	vector<Point> biggestContour;

	for ( size_t i = 0; i < hull.size(); i++ ) {
		float area = contourArea(hull[i]);

		if (area > biggestArea) {
			biggestArea = area;
			biggestContour = hull[i];
		}
	}

	vector<vector<Point>> bigContours;
	bigContours.push_back(biggestContour);
	if (bigContours.size() > 1){
		drawContours(img, bigContours, 0, 255, -1);
	}

	return img;
}

Mat BaseDetector::imageLimiares(Mat hsv, int hsvMin[3], int hsvMax[3])
{
	Mat hsvtresh;

	inRange(hsv, Scalar(hsvMin[0], hsvMin[1], hsvMin[2]), Scalar(hsvMax[0], hsvMax[1], hsvMax[2]), hsvtresh);

	hsvtresh = imageFill(hsvtresh);

	return hsvtresh;
}

void BaseDetector::setCameraParams(Mat img)
{
	cameraImage.rows = img.rows;
	cameraImage.cols = img.cols;

	cameraImage.centerX = img.size().width/2;
	cameraImage.centerY = img.size().height/2;
}

void BaseDetector::setImage(Mat img)
{
	GaussianBlur(img, img, Size(GAUSSIANFILTER, GAUSSIANFILTER), 0);

	if (fstTime) {
		this->setCameraParams(img);
		fstTime = false;
	}

	this->imageMainC3 = img;
}

void BaseDetector::processImage()
{
	cvtColor(this->imageMainC3, this->imageMainHsvC3, COLOR_BGR2HSV);

	Mat imageHsv, imageOutput, imageBitwiseBaseFinal;

	// Isola a parte azul da base
	this->imageBlueC1 = imageLimiares(this->imageMainHsvC3, ARR_MINBLUE, ARR_MAXBLUE);
	bitwise_and(this->imageMainHsvC3, this->imageMainHsvC3, imageHsv, this->imageBlueC1);

	// Isola a parte amarela da base
	this->imageYellowC1 = imageLimiares(this->imageMainHsvC3, ARR_MINYELLOW, ARR_MAXYELLOW);
	bitwise_and(imageHsv, imageHsv, imageOutput, this->imageYellowC1);

	// Pega apenas a area do mark
	bitwise_and(this->imageBlueC1, this->imageYellowC1, this->imageFinalC1);	
}

void BaseDetector::loadMask(string maskPath)
{
	this->mask = imread(maskPath, IMREAD_GRAYSCALE);
}

void BaseDetector::applyMask()
{
	Mat output;

	bitwise_and(this->imageMainC3, this->imageMainC3, output, this->mask);

	this->imageMainC3 = output.clone();
}

bool BaseDetector::findSquare()
{
	findContours(this->imageFinalC1, this->contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

	bool success = false;
	int minArea = 400;

	for (int i=0; i<this->contours.size(); i++)
	{
		Rect currentRect = boundingRect( this->contours[i] );

		int areaRect = currentRect.width * currentRect.height;

		if (areaRect >= minArea) {
			this->basesList.push_back(currentRect);

			success = true;
		}
	}

	return success;
}

void BaseDetector::drawClosestBase()
{
	Rect closestBase;
	float distance;
	float lowestDistance = -1.0;

	float centerCurrentBaseX;
	float centerCurrentBaseY;
	for (size_t i = 0; i < this->basesList.size(); i++)
	{
		centerCurrentBaseX = (this->basesList[i].x + this->basesList[i].width) - (this->basesList[i].width/2);
		centerCurrentBaseY = (this->basesList[i].y + this->basesList[i].height) - (this->basesList[i].height/2);

		circle(this->imageMainC3, Point(centerCurrentBaseX, centerCurrentBaseY), 1, (0, 255, 255), 2);
		circle(this->imageMainC3, Point(this->cameraImage.centerX, this->cameraImage.centerY), 1, (255, 255, 0), 2);

		float dx = centerCurrentBaseX - (this->cameraImage.centerX + 0.5);
		float dy = (this->cameraImage.centerY + 0.5) - centerCurrentBaseY;

		distance = hypot(dx, dy);

		// lowestDistance == -1 verifica se é a primeira iteração do for
		if (distance < lowestDistance || lowestDistance == -1) {
			lowestDistance = distance;
			closestBase = this->basesList[i];
			this->x = dx;//std::floor(centerCurrentBaseX/(float(this->cameraImage.centerX)*2))*float(this->cameraImage.centerX) - (float(this->cameraImage.centerX));
			this->y = dy;//std::floor((1-centerCurrentBaseY)/(float(this->cameraImage.centerY)*2))*float(this->cameraImage.centerY) - (float(this->cameraImage.centerY));
			
		}

		rectangle(this->imageMainC3, this->basesList[i], cvCOLOR_RED, 2, LINE_8);
		this->success = false;
	}

	this->basesList.clear();
	this->closestBase = closestBase;

	rectangle(this->imageMainC3, closestBase, cvCOLOR_GREEN, 3);
	this->success = true;
	
}

void BaseDetector::setClosestBaseDistance()
{
	// this->dx.data = this->closestBase.x + ( this->closestBase.width/2 ) - this->cameraImage.centerX;
	// this->dy.data = this->closestBase.y + ( this->closestBase.height/2 ) - this->cameraImage.centerY;
}

bool BaseDetector::existBaseAround()
{
	bool success = false;

	for (size_t i = 0; i < this->basesList.size(); i++)
	{
		int coordX = this->basesList[i].x;
		int coordY = this->basesList[i].y;

		// Verifica se não é a base atual
		if ( coordX == this->closestBase.x && coordY == this->closestBase.y ) {
			continue;
		}

		success = true;
		
		this->closestBase = this->basesList[i];

		break;
	}

	return success;
}

void BaseDetector::drawSquare()
{
	rectangle(imageMainC3, mark, cvCOLOR_RED, 2);
}

void BaseDetector::show()
{
	// imshow("yellow", imageYellowC1);
	// imshow("blue", imageBlueC1);
	imshow("Main", imageMainC3);
}
