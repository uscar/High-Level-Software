/*
 * camera_capture.cpp
 *
 *  Created on: Feb 3, 2015
 *      Author: nam_macbookpro
 */
#include <iostream>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>

using namespace std;
using namespace cv;


vector<int> neighbors(int x, int y, int height, int width) {
	vector<int> ret;
	for(int dx = -1; dx <=1; dx++) {
		for (int dy = -1; dy <= 1; dy++){
			if ((dx != 0 || dy != 0) && y + dy < height && y+dy>=0 && x+dx < width && x+dx >=0)
				ret.push_back((x+dx)*height+y+dy);
		}
	}
	return ret;
}

float errfunc(const Point2f &p1, const Point2f &p2) {
	return (p1.x - p2.x)*(p1.x-p2.x)+(p1.y-p2.y)*(p1.y-p2.y);
}

Point2f avg(const vector<Point2f> &objs) {
	Point2f ret(0,0);
	for (int i = 0 ;i < objs.size() ; i++){
		ret+=objs[i];
	}
	ret.x /= objs.size();
	ret.y /= objs.size();
	return ret;
}

Point2f displacementGuess(vector<Point2f> &vf) {
	return avg(vf);
}

float rotationGuess(vector<Point2f> &vf, int h) {
	for (int i = 0 ;i < 100 ;i ++ ) {

	}
	return 0;
}


vector<Point2f> neiPoints(vector<Point2f> &vf, int height, int width, int x, int y) {
	vector<int> indxs = neighbors(x,y,height,width);
	vector<Point2f> ret;
	for(int i = 0;i <indxs.size();i++) {
		ret.push_back(vf[indxs[i]]);
	}
	return ret;
}

void outlierFilter(vector<Point2f> &vectField, int height) {
	int width = vectField.size() / height;
	float cutoff = 0.0f;
	for(int i = 0;i < vectField.size();i++) {
		int x = i / height;
		int y = i % height;
		vector<Point2f> neighs = neiPoints(vectField, height, width, x, y);
		Point2f average = avg(neighs);
		Point2f v = vectField[i];
		if (errfunc(average, v) > cutoff && !(errfunc(v, Point2f(0,0)) > cutoff)) {
			vectField[i] = average;
		}
	}
}

// try the following. /consider publishing...
/*
 * Compute the fft of the region of interest for the initial and future frame
 * determine the phase difference -> generate the displacement from that phase
 * (ift(fft(A)*fft(B)) = A**B -> min(ift(fft(A)*fft(shift(phi,B))),phi) => best phi).
 */
int main(int argc, char *argv[]) {
    Mat frame, preframe, greyFrame, image;
    Point2f sum(0,0);
	vector<Point2f> features,prefeatures;
	vector<uchar> status;
	vector<float> err;
	vector<Point2f> uniform, uniout;
	int hpts = 10,vpts = 10;
	vector<Point2f> vector_field(hpts*vpts);

	CvCapture* capture = cvCaptureFromCAM( CV_CAP_ANY );
	if(!capture) {
		cout << "No camera detected" << endl;
	}

	bool first = 1;
	cvNamedWindow( "result", CV_WINDOW_AUTOSIZE );

	Scalar marker_color(1.0,255.0,1.0);
	Scalar marker2_color(255,1,1);
	if(capture) {
		cout << "In capture ..." << endl;
		while(1) {
			IplImage* iplImg = cvQueryFrame(capture);
			frame = iplImg;
			cvtColor(frame, greyFrame,CV_BGR2GRAY);
			if (first) {
				for(int x = 0; x < hpts; x++) {
					for (int y = 0; y < vpts; y++) {
						Point2f p(frame.size().width * (x+.5) / hpts, frame.size().height * (y+.5) /vpts);
						uniform.push_back(p);
					}
				}
			}
			if(!first) {
				calcOpticalFlowPyrLK(preframe,greyFrame,uniform,uniout,status,err);
			}
			Point2f displacement(0,0);
			for(int i = 0;i < uniform.size();i++) {
				circle(frame,(Point2f)uniform[i],2,marker_color,2);
				if (i < uniout.size()) {
					//circle(frame,(Point2f)uniout[i],2,marker2_color,2);
					//line(frame,(Point2f)uniform[i],(Point2f)uniout[i],marker2_color);
					Point2f delta= ((Point2f)uniform[i] - (Point2f)uniout[i]);
					displacement += delta;
					vector_field[i] = delta;
					outlierFilter(vector_field,vpts);
					Point2f newout = uniform[i] - vector_field[i];
					circle(frame,newout,2,marker2_color,2);
					line(frame,(Point2f)uniform[i],newout,marker2_color);
				}
			}
			sum += displacementGuess(vector_field);
			cout << sum <<endl;
			imshow("result",frame);
			preframe = greyFrame.clone();
			first = false;
			if(waitKey(10) >= 0)
				break;
		}
	}
	cvReleaseCapture( &capture );
	cvDestroyWindow( "result" );
	return 0;
}

