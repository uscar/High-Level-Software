#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "LSWMS.h"
#include <iostream>
#include <stdio.h>
#include <vector>

using namespace cv;
using namespace std;

int main(int argc, char* argv[])
{
  //turn on video camera
  VideoCapture video(0);
  if (!video.isOpened()) {
    cout << "Cannot turn on the camera." << endl;
    return -1;
  }

  // create a window called "Camera-LSWMS"
  namedWindow("Camera-LSWMS", CV_WINDOW_AUTOSIZE);

  // video parameters
  int width = (int) video.get(CV_CAP_PROP_FRAME_WIDTH);
  int height = (int) video.get(CV_CAP_PROP_FRAME_HEIGHT);
  int fps = (int) video.get(CV_CAP_PROP_FPS);
  int fourcc = (int) video.get(CV_CAP_PROP_FOURCC);
  printf("Input camera: (%d x %d) at %d fps\n", width, height, fps);

  // Create and init LSWMS = Line Segment detection using Weighted Mean Shift
  int R = 3, numMaxLSegs = 0;
  bool verbose = true;
  cv::Size processSize = cv::Size(width, height);
  LSWMS lswms(processSize, R, numMaxLSegs, verbose);
  printf("LSWMS object created: R=%d\n\n", R);

  // line segments and errors for LWSMS
  std::vector<LSEG> lSegs;
  std::vector<double> errors;

  while(true)
    {
      // read a new frame from camera
      Mat frame, newframe, imgGRAY;
      /* bool bSuccess = video.read(frame);
      if (!bSuccess) {
	cout << "Cannot read frames from the camera." << endl;
	break;
	} */
      video >> frame;
      if (frame.empty()) break;

      // color conversion
      if(frame.channels() == 3) {
	cv::cvtColor(frame, imgGRAY, CV_BGR2GRAY);	
	frame.copyTo(newframe);
      } else {
	frame.copyTo(imgGRAY);
	cv::cvtColor(frame, newframe, CV_GRAY2BGR);
      }

      // Process LSWMS
      lswms.run(frame, lSegs, errors);

      // draw the lines according to errors
      lswms.drawLSegs(newframe, lSegs, errors);

      //show the processed frame in "Camera Input" window
      imshow("Camera-LSWMS", newframe);
      
      // if user presses 'esc' for 30 ms, end program
      if(waitKey(30) == 27) {
	cout << "Esc key is pressed. Terminating." << endl; 
	break; 
      }
    }

  return 0;

}
