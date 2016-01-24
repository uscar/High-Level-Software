/**
 * @file Drawing_1.cpp
 * @brief Simple sample code
 */

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <thread>
#include <iostream>
#include <chrono>
#include <atomic>

#define w 400

using namespace cv;
using namespace std;

/// Function headers
void MyEllipse( Mat img, double angle );
void MyFilledCircle( Mat img, Point center );
void MyPolygon( Mat img );
void MyLine( Mat img, Point start, Point end );

atomic_bool shutdown;
int x = 0;
int y = 0;
void animate() {
	cout << "Hello World" << endl;
	Mat ball_image = Mat::zeros(w, w, CV_8UC3);
	MyFilledCircle(ball_image, Point(x+=5, y+=4));
	x%=w;
	y%=w;
	imshow("Bouncing Ball", ball_image);
}

// tries to draw at the given rate.
void draw_thread(float hz) {
	int millis = 1000 / hz;
	shutdown = false;
	while (1) {
		if (shutdown) {
			return;
		}
		auto t0 = chrono::high_resolution_clock::now();
		animate();
		chrono::milliseconds dt = chrono::duration_cast<chrono::milliseconds>(chrono::high_resolution_clock::now() - t0);
		if (millis - dt.count() > 0) {
			this_thread::sleep_for(chrono::milliseconds(millis - dt.count()));
		}
	}
}

int main() {
	namedWindow("Bouncing Ball");
	thread draw(draw_thread, 30);
	waitKey(0);
	shutdown = true;
	draw.join();
	return(0);
}

void MyFilledCircle( Mat img, Point center ) {
  int thickness = -1;
  int lineType = 8;
  circle(img, center, w/32, Scalar( 0, 0, 255 ), thickness, lineType );
}
