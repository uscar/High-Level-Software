#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <thread>
#include <iostream>
#include <chrono>
#include <atomic>

#include "visualizer.h"

using namespace cv;
using namespace std;

// game parameters.
const int field_size = 2000; // cm
const int tape_width = 5; // cm
const int roomba_radius = 17; // cm

Visualizer::Visualizer(string name, vector<Point>* points, int size) : window_size(size), window_name(name), pix_per_cm((float)window_size / field_size) {
  namedWindow(window_name);
  this->roombas = points;
  draw = new thread(&Visualizer::draw_thread, this, 30);
}

Visualizer::~Visualizer() {
  shutdown = true;
  draw->join();
  delete draw;
  cvDestroyWindow(window_name.c_str());
  background.release();
}

void Visualizer::animate() {
  Mat image = background.clone();
  for (int i = 0; i < roombas->size(); i++) {
    drawRoomba(image, (*roombas)[i], Scalar(0,255,0));
  }
  imshow(window_name, image);
  waitKey(1);
  image.release();
}

// tries to draw at the given rate.
void Visualizer::draw_thread(float hz) {
  int millis = 1000 / hz;
  shutdown = false;
  init_background(background);
  //srand(102310);
  srand(102311);
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
  }
}

void Visualizer::init_background(Mat& background) {
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

void Visualizer::drawRoomba( Mat img, Point center, Scalar color) {
  Point pxPt = Point(center.x * pix_per_cm, center.y * pix_per_cm);
  circle(img, pxPt, roomba_radius * pix_per_cm, color, -1, 8);
}

