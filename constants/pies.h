#include <cmath>
#include <utility>
#include <vector>
#include "str_cmds.h"

std::pair<double, double> rect_to_polar(double x, double y) {
	double r = sqrt(x * x + y * y);
	double theta = atan(y / x);
	if (x < 0) {
		theta += M_PI;
	}
	if (x > 0 && y < 0) {
		theta += 2 * M_PI;
	}
	return std::make_pair(r, theta);
}

struct Slice {
	double lower_bound;
	double upper_bound;
	double radius;
};

class Pie {
public:
	Pie(std::vector<Slice> p, std::vector<std::pair<std::vector<size_t>, std::string>> c)
	: portions(p), occupied(p.size(), false), checks(c) {
		// do nothing
	}

	void update(double x, double y) {
		std::pair<double, double> polar_coords = rect_to_polar(x,y);
		double& r = polar_coords.first;
		double& theta = polar_coords.second;
		for (size_t i = 0; i < portions.size(); ++i) {
			if (portions[i].lower_bound < theta &&
			    theta < portions[i].upper_bound &&
			    r < portions[i].radius) {
				occupied[i] = true;
			}
		}
	}

	void reset() {
		for (size_t i = 0; i < occupied.size(); ++i) {
			occupied[i] = false;
		}
	}

	std::string evaluate() {
		for (size_t i = 0; i < checks.size(); ++i) {
			std::pair<std::vector<size_t>, std::string>& condition = checks[i];
			bool flag = true;
			for (size_t j = 0; j < condition.first.size(); ++j) {
				if (!occupied[condition.first[j]]) {
					flag = false;
				}
			}
			if (flag) {
				return condition.second;
			}
		}
		return "";
	}

private:
	std::vector<Slice> portions;
	std::vector<bool> occupied;
	std::vector<std::pair<std::vector<size_t>, std::string>> checks;
};
