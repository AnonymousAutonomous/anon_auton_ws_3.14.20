#include "ros/ros.h"
#include "eyes/Generic.h"
#include <string>
#include <vector>

static const std::vector<eyes::Generic> DANCE_C = {
	[] {
		eyes::Generic first;
		first.identifier = 'c';
		first.left_forward = false;
		first.right_forward = false;
		first.left_speed = 255;
		first.right_speed = 255;
		first.timed = false;
		first.duration = 10000;
		return first;
	}(),
	[] {
		eyes::Generic second;
		second.identifier = 'c';
		second.left_forward = false;
		second.right_forward = false;
		second.left_speed = 0;
		second.right_speed = 0;
		second.timed = true;
		second.duration = 2;
		return second;
	}(),
	[] {
		eyes::Generic third;
		third.identifier = 'c';
		third.left_forward = false;
		third.right_forward = false;
		third.left_speed = 255;
		third.right_speed = 255;
		third.timed = false;
		third.duration = 10000;
		return third;
	}(),
	[] {
		eyes::Generic fourth;
		fourth.identifier = 'c';
		fourth.left_forward = false;
		fourth.right_forward = false;
		fourth.left_speed = 0;
		fourth.right_speed = 0;
		fourth.timed = true;
		fourth.duration = 2;
		return fourth;
	}(),
	[] {
		eyes::Generic eoc;
		eoc.identifier = 'e';
		eoc.left_forward = true;
		eoc.right_forward = true;
		eoc.left_speed = 0;
		eoc.right_speed = 0;
		eoc.timed = true;
		eoc.duration = 0;
		return eoc;
	}()
};

static const std::vector<eyes::Generic> RREVERSE_C = {
	[] {
		eyes::Generic first;
		first.identifier = 'c';
		first.left_forward = false;
		first.right_forward = false;
		first.left_speed = 80;
		first.right_speed = 80;
		first.timed = true;
		first.duration = 1;
		return first;
	}(),
	[] {
		eyes::Generic second;
		second.identifier = 'c';
		second.left_forward = true;
		second.right_forward = false;
		second.left_speed = 100;
		second.right_speed = 100;
		second.timed = true;
		second.duration = 5;
		return second;
	}(),
	[] {
		eyes::Generic third;
		third.identifier = 'c';
		third.left_forward = true;
		third.right_forward = false;
		third.left_speed = 100;
		third.right_speed = 100;
		third.timed = true;
		third.duration = 5;
		return third;
	}(),
	[] {
		eyes::Generic eoc;
		eoc.identifier = 'e';
		eoc.left_forward = true;
		eoc.right_forward = true;
		eoc.left_speed = 0;
		eoc.right_speed = 0;
		eoc.timed = true;
		eoc.duration = 0;
		return eoc;
	}()
};

static const std::vector<eyes::Generic> LREVERSE_C = {
	[] {
		eyes::Generic first;
		first.identifier = 'c';
		first.left_forward = false;
		first.right_forward = false;
		first.left_speed = 80;
		first.right_speed = 80;
		first.timed = true;
		first.duration = 1;
		return first;
	}(),
	[] {
		eyes::Generic second;
		second.identifier = 'c';
		second.left_forward = false;
		second.right_forward = true;
		second.left_speed = 100;
		second.right_speed = 100;
		second.timed = true;
		second.duration = 5;
		return second;
	}(),
	[] {
		eyes::Generic third;
		third.identifier = 'c';
		third.left_forward = false;
		third.right_forward = true;
		third.left_speed = 100;
		third.right_speed = 100;
		third.timed = true;
		third.duration = 5;
		return third;
	}(),
	[] {
		eyes::Generic eoc;
		eoc.identifier = 'e';
		eoc.left_forward = true;
		eoc.right_forward = true;
		eoc.left_speed = 0;
		eoc.right_speed = 0;
		eoc.timed = true;
		eoc.duration = 0;
		return eoc;
	}()
};

static const std::vector<eyes::Generic> LCP_C = {
	[] {
		eyes::Generic first;
		first.identifier = 'c';
		first.left_forward = false;
		first.right_forward = true;
		first.left_speed = 100;
		first.right_speed = 100;
		first.timed = true;
		first.duration = 5;
		return first;
	}(),
	[] {
		eyes::Generic eoc;
		eoc.identifier = 'e';
		eoc.left_forward = true;
		eoc.right_forward = true;
		eoc.left_speed = 0;
		eoc.right_speed = 0;
		eoc.timed = true;
		eoc.duration = 0;
		return eoc;
	}()
};

static const std::vector<eyes::Generic> RCP_C = {
	[] {
		eyes::Generic first;
		first.identifier = 'c';
		first.left_forward = true;
		first.right_forward = false;
		first.left_speed = 100;
		first.right_speed = 100;
		first.timed = true;
		first.duration = 5;
		return first;
	}(),
	[] {
		eyes::Generic eoc;
		eoc.identifier = 'e';
		eoc.left_forward = true;
		eoc.right_forward = true;
		eoc.left_speed = 0;
		eoc.right_speed = 0;
		eoc.timed = true;
		eoc.duration = 0;
		return eoc;
	}()
};

static const std::vector<eyes::Generic> SPIN_C = {
	[] {
		eyes::Generic first;
		first.identifier = 'c';
		first.left_forward = true;
		first.right_forward = false;
		first.left_speed = 100;
		first.right_speed = 100;
		first.timed = true;
		first.duration = 5;
		return first;
	}(),
	[] {
		eyes::Generic second;
		second.identifier = 'c';
		second.left_forward = true;
		second.right_forward = false;
		second.left_speed = 100;
		second.right_speed = 100;
		second.timed = true;
		second.duration = 5;
		return second;
	}(),
	[] {
		eyes::Generic eoc;
		eoc.identifier = 'e';
		eoc.left_forward = true;
		eoc.right_forward = true;
		eoc.left_speed = 0;
		eoc.right_speed = 0;
		eoc.timed = true;
		eoc.duration = 0;
		return eoc;
	}()
};
