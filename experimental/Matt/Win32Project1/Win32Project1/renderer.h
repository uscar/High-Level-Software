#pragma once
#include <vector>
#include "base_classes.h"
#include <set>
#include "helper.h"
#include <time.h>
#include "vec2d.h"
#include <chrono>
#include <thread>
#include <string>
#include <iostream>


class renderer { //here see? object oriented and family friendly
public:
	void setLatestTime(double);
	renderer(robot ** stuff);
	virtual void runRenderer();
	virtual void draw(double time);
	virtual ~renderer();
protected:
	//std::set<robot_state *> objects;
	robot ** objects;
	const int max_objects = 20;
	double latest_time; //upper limit on time that can be rendered
};

void renderer::setLatestTime(double t) {
	latest_time = t;
}

renderer::renderer(robot ** stuff) {
	objects = stuff; //incredibly descriptive
}

renderer::~renderer() {

}

/*void renderer::draw(double time,HDC & hin,RECT *& prc_in) {
	HDC hdcBuffer = CreateCompatibleDC(hdc);
	HBITMAP hbmBuffer = CreateCompatibleBitmap(hdc, prc->right, prc->bottom);
	FillRect(hdcBuffer, prc, (HBRUSH)GetStockObject(WHITE_BRUSH));
	int savedDC = SaveDC(hdcBuffer);
	SelectObject(hdcBuffer, hbmBuffer);
	for (robot_state* i : objects) {
		if (i != nullptr) {
			SelectObject(hdcBuffer, GetStockObject(DC_BRUSH));
			SetDCBrushColor(hdcBuffer, RGB(255, 0, 0));
			SelectObject(hdcBuffer, GetStockObject(DC_PEN));
			SetDCPenColor(hdcBuffer, RGB(0, 0, 0));
			//vec2d st = currbots[i]->getposition(currtime-interval);
			SetDCBrushColor(hdcBuffer, RGB(i->c.r*(double)255, i->c.g*(double)255, i->c.b*(double)255));
			//MoveToEx(hdc, (int)40*st.first, (int)40*st.second, NULL);
			//LineTo(hdc, (int)40*ed.first, (int)40*ed.second);
			vec2d ed = i->getPosition(time);
			vec2d top = 40 * (ed - vec2d(roombaDiameter / (2 + 2 * (i->getName() == "Quad")), roombaDiameter / (2 + 2 * (i->getName() == "Quad"))));
			vec2d bottom = 40 * (ed + vec2d(roombaDiameter / (2 + 2 * (i->getName() == "Quad")), roombaDiameter / (2 + 2 * (i->getName() == "Quad"))));
			//OutputDebugStringA(("Time: "+std::to_string(time)+", Position : "+std::to_string(top.first) + ", " + std::to_string(top.first) + ", " + std::to_string(top.first) + ", " + std::to_string(top.first) + "\r\n").c_str());
			Ellipse(hdcBuffer, top.first, top.second, bottom.first, bottom.second);
		}
	}
	//aerial robot render code
	SelectObject(hdcBuffer, GetStockObject(DC_BRUSH));
	SetDCBrushColor(hdcBuffer, RGB(255, 255, 255));
	SelectObject(hdcBuffer, GetStockObject(DC_PEN));
	SetDCPenColor(hdcBuffer, RGB(255, 255, 255));
	//vec2d st = currbots[i]->getposition(currtime-interval);
	SetDCBrushColor(hdcBuffer, RGB(255, 255, 255));
	//MoveToEx(hdc, (int)40*st.first, (int)40*st.second, NULL);
	//LineTo(hdc, (int)40*ed.first, (int)40*ed.second);
	BitBlt(hdc, 0, 0, prc->right, prc->bottom, hdcBuffer, 0, 0, SRCCOPY); //writes
	RestoreDC(hdcBuffer, savedDC);
	DeleteDC(hdcBuffer);
	DeleteObject(hbmBuffer);
}*/