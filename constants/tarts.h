#include <cmath>
#include <utility>
#include <vector>

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

class Tart {
public:
	Tart(std::vector<Slice> c_portions, std::vector<Slice> l_portions, std::vector<std::pair<std::pair<std::vector<size_t>, std::vector<size_t>>, std::string>> c)
	: circle_portions(c_portions), line_portions(l_portions), circle_occupied(c_portions.size(), false), line_occupied(l_portions.size(), false), checks(c) {
		// do nothing
	}

	void circle_update(double x, double y) {
		std::pair<double, double> polar_coords = rect_to_polar(x,y);
		double& r = polar_coords.first;
		double& theta = polar_coords.second;
		for (size_t i = 0; i < circle_portions.size(); ++i) {
			if (circle_portions[i].lower_bound < theta &&
			    theta < circle_portions[i].upper_bound &&
			    r < circle_portions[i].radius) {
				circle_occupied[i] = true;
			}
		}
	}

	void line_update(double x, double y) {
		std::pair<double, double> polar_coords = rect_to_polar(x,y);
		double& r = polar_coords.first;
		double& theta = polar_coords.second;
		for (size_t i = 0; i < line_portions.size(); ++i) {
			if (line_portions[i].lower_bound < theta &&
			    theta < line_portions[i].upper_bound &&
			    r < line_portions[i].radius) {
				line_occupied[i] = true;
			}
		}
	}

	void reset() {
		for (size_t i = 0; i < circle_occupied.size(); ++i) {
			circle_occupied[i] = false;
		}
		for (size_t i = 0; i < line_occupied.size(); ++i) {
			line_occupied[i] = false;
		}
	}

	void updateChecks(std::vector<std::pair<std::pair<std::vector<size_t>, std::vector<size_t>>, std::string>> c_in) {
		checks = c_in;
	}

	std::string evaluate() {
		for (size_t i = 0; i < checks.size(); ++i) {
			std::pair<std::pair<std::vector<size_t>, std::vector<size_t>>, std::string>& condition = checks[i];
			bool flag = true;
			for (size_t j = 0; j < condition.first.first.size() && flag; ++j) {
				if (!circle_occupied[condition.first.first[j]]) {
					flag = false;
				}
			}
			for (size_t k = 0; k < condition.first.second.size() && flag; ++k) {
				if (!line_occupied[condition.first.second[k]]) {
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
	std::vector<Slice> circle_portions;
	std::vector<Slice> line_portions;
	std::vector<bool> circle_occupied;
	std::vector<bool> line_occupied;
	std::vector<std::pair<std::pair<std::vector<size_t>, std::vector<size_t>>, std::string>> checks;
};
