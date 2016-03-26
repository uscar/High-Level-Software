#ifndef VISUALIZER_H
#define VISUALIZER_H

#include <opencv2/core/core.hpp>
#include <thread>
#include <atomic>

class Visualizer {
  std::atomic_bool shutdown;
  const int window_size;
  const std::string window_name;
  const float pix_per_cm;
  std::thread *draw;
  cv::Mat background;
  std::vector<cv::Point> *roombas;

  void drawRoomba(cv::Mat img, cv::Point center, cv::Scalar color);
  void init_background(cv::Mat &background);
  void draw_thread(float hz);
  void animate();
public:
  Visualizer(std::string name, std::vector<cv::Point> *points, int size = 1000);
  ~Visualizer();
};

#endif // VISUALIZER_H
