#pragma once
#include <vector>
#include "base_classes.h"
#include <set>
#include "helper.h"
#include <time.h>
#include "vec2d.h"
#include <chrono>
#include <thread>

class renderer { //here see? object oriented and family friendly
public:
	void setLatestTime(double);
	void removeObject(robot_state * obj);
	void addObject(robot_state * obj);
	renderer(std::vector<void *> stuff);
	virtual void runRenderer();
	virtual void draw(double time);
	virtual ~renderer();
protected:
	std::set<robot_state *> objects;
	double latest_time;
	double updateInterval = 1.0 / 30.0 * 1000.0; //interval in ms
	HDC hdc;
	std::thread * renderThread;
	RECT * prc;
};

void renderer::runRenderer() {
	std::chrono::high_resolution_clock::time_point start_t = std::chrono::high_resolution_clock::now();
	std::chrono::high_resolution_clock::time_point end_t = start_t;
	std::chrono::high_resolution_clock::duration interval = std::chrono::duration_cast<std::chrono::high_resolution_clock::duration>(std::chrono::duration<double,std::milli>(updateInterval));
	//just defines start and interval as usable classes
	double elapsed;
	while (true) {
		std::this_thread::sleep_until(end_t);
		end_t = std::chrono::high_resolution_clock::now();
		elapsed = std::chrono::duration_cast<std::chrono::seconds>(end_t-start_t).count();
		end_t += interval;
		draw(elapsed);
	}
}

renderer::~renderer() {
	delete renderThread;
}

void renderer::removeObject(robot_state * obj) {
	if (obj != nullptr) {
		objects.erase(obj);
	}
}

void renderer::addObject(robot_state * obj) {
	if (obj != nullptr) {
		if (objects.count[obj] == 0) {
			objects.insert(obj);
		}
	}
}

void renderer::setLatestTime(double t) {
	latest_time = t;
}

renderer::renderer(std::vector<void *> stuff) { //just pass in whatever you want and initialize from whatever you want ;)
	hdc = (HDC)stuff[0];
	prc = (RECT*)stuff[1];
	renderThread = new std::thread([&]() {runRenderer(); });
}

void renderer::draw(double time) {
	HDC hdcBuffer = CreateCompatibleDC(hdc);
	HBITMAP hbmBuffer = CreateCompatibleBitmap(hdc, prc->right, prc->bottom);
	FillRect(hdcBuffer, prc, (HBRUSH)GetStockObject(WHITE_BRUSH));
	int savedDC = SaveDC(hdcBuffer);
	SelectObject(hdcBuffer, hbmBuffer);
	for (robot_state* i : objects) {
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
		vec2d bottom = 40 * (ed + vec2d(roombaDiameter / (2+2*(i->getName()=="Quad")), roombaDiameter / (2 + 2 * (i->getName() == "Quad"))));
		Ellipse(hdcBuffer, top.first, top.second, bottom.first, bottom.second);
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
}