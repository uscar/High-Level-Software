#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <thread>
#include <iostream>
#include <chrono>
#include <atomic>

using namespace cv;
using namespace std;

/// Function headers
void drawRoomba(Mat img, Point center, Scalar color);
void init_background(Mat &background);

const int window_size = 1000; // px
const int field_size = 2000; // cm
const int tape_width = 5; // cm
const int roomba_radius = 17; // cm
const float pix_per_cm = (float) window_size / field_size;

const int roomba_count = 3;
Point roombas[roomba_count];

int x = 0;
int y = 0;

atomic_bool shutdown;
Mat background;

void animate() {
	Mat ball_image = background.clone();
    drawRoomba(ball_image, Point(x+=5, y+=4), Scalar(0,0,255));
    for (int i = 0; i < roomba_count; i++) {
        drawRoomba(ball_image, roombas[i], Scalar(0,255,0));
    }
	x %= field_size;
	y %= field_size;
	imshow("Bouncing Ball", ball_image);
    cout << x << " - " << y << endl;
    ball_image.release();
}

// tries to draw at the given rate.
void draw_thread(float hz) {
	int millis = 1000 / hz;
	shutdown = false;
    init_background(background);
    srand(102310);
    for (int i =0 ;i < roomba_count; i++) {
        roombas[i] = Point(rand() % field_size, rand() % field_size);
    }
	while (1) {
		if (shutdown) {
            return;
		}
		auto t0 = chrono::high_resolution_clock::now();
		animate();
		chrono::milliseconds dt = 
            chrono::duration_cast<chrono::milliseconds>
            (chrono::high_resolution_clock::now() - t0);
        int sleep_time = millis - dt.count();
		if (sleep_time > 0) {
			this_thread::sleep_for(chrono::milliseconds(sleep_time));
		}
        cout << sleep_time << endl;
	}
}

void init_background(Mat& background) {
    background = Mat::zeros(window_size, window_size, CV_8UC3);
    float lw = tape_width * pix_per_cm;
    float line_separation = 100 * pix_per_cm; // 1m between gridlines
    for (int lx = 0; lx <= window_size; lx += line_separation) {
        line(background, Point(lx,0), Point(lx,window_size),
                Scalar(255,255,255), lw);
    }
    for (int ly = 0; ly <= window_size; ly += line_separation) {
        line(background, Point(0,ly), Point(window_size,ly),
                Scalar(255,255,255), lw);
    }
    // green top
    line(background, Point(0,0), Point(window_size,0), Scalar(0,255,0), lw);
    // red bottom
    line(background, Point(0,window_size), 
            Point(window_size,window_size), Scalar(0,0,255), lw);
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
    Point pxPt = Point(center.x * pix_per_cm, center.y * pix_per_cm);
    circle(img, pxPt, roomba_radius * pix_per_cm, color, -1, 8);
}
