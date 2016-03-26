#include "roomba_action.h"

struct evaluation_metric {
	virtual double operator()(roomba * a, double time) {
		double total = 0;
		for (int i = 0; i < 10; ++i) {
			total += 10.0 - a->getPosition(time).second;
		}
		return total;
	}
};
