#include <iostream>
#include <opencv2/core/core.hpp>

#include "visualizer/visualizer.h"

using namespace std;

int main() {
  vector<cv::Point> robots;
  srand(23423);
  for (int i = 0; i < 10; i++) {
    robots.push_back(cv::Point(rand() %2000, rand()%2000));
  }
  Visualizer vis("Roomba window", &robots, 1000);
  char anything = 0;
  while (anything != 'q') {
    cin >> anything;
    for (int i = 0; i < 10; i++) {
      robots[i] = cv::Point(rand() %2000, rand()%2000);
    }
  }
  cout << "closing" << endl;
  return 0; // cleaning up vis closes the window.
}
