#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <thread>
#include <iostream>
#include <chrono>
#include <atomic>

#define w 400
#define ROOMBA_WIDTH 10
using namespace cv;
using namespace std;

/// Function headers
void drawRoomba( Mat img, Point center, Scalar color);

const float pix_per_cm = 1;
const int width = 1000;
const int height = 1000;

atomic_bool shutdown;
int x = 0;
int y = 0;
void animate() {
	Mat ball_image = Mat::zeros(w, w, CV_8UC3);
	drawRoomba(ball_image, Point(x+=5, y+=4), Scalar(0,0,255));
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

void init_background(Mat& background) {
    background = Mat::zeros(width, height, CV_8UC3);

}

int main() {
	namedWindow("Bouncing Ball");
	thread draw(draw_thread, 30);
	waitKey(0);
	shutdown = true;
	draw.join();
	return(0);
}

void drawRoomba( Mat img, Point center, Scalar color) {
    circle(img, center, ROOMBA_WIDTH, color, -1, 8);
}
