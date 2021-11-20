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
		first.left_speed = 3.0;
		first.right_speed = 3.0;
		first.timed = false;
		first.duration = 10000;
		return first;
	}(),
	[] {
		eyes::Generic second;
		second.identifier = 'c';
		second.left_forward = false;
		second.right_forward = false;
		second.left_speed = 0.0;
		second.right_speed = 0.0;
		second.timed = true;
		second.duration = 2;
		return second;
	}(),
	[] {
		eyes::Generic third;
		third.identifier = 'c';
		third.left_forward = false;
		third.right_forward = false;
		third.left_speed = 3.0;
		third.right_speed = 3.0;
		third.timed = false;
		third.duration = 10000;
		return third;
	}(),
	[] {
		eyes::Generic fourth;
		fourth.identifier = 'c';
		fourth.left_forward = false;
		fourth.right_forward = false;
		fourth.left_speed = 0.0;
		fourth.right_speed = 0.0;
		fourth.timed = true;
		fourth.duration = 2;
		return fourth;
	}(),
	[] {
		eyes::Generic eoc;
		eoc.identifier = 'e';
		eoc.left_forward = true;
		eoc.right_forward = true;
		eoc.left_speed = 0.0;
		eoc.right_speed = 0.0;
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
		first.left_speed = 2.0;
		first.right_speed = 2.0;
		first.timed = true;
		first.duration = 1;
		return first;
	}(),
	[] {
		eyes::Generic second;
		second.identifier = 'c';
		second.left_forward = true;
		second.right_forward = false;
		second.left_speed = 2.5;
		second.right_speed = 2.5;
		second.timed = true;
		second.duration = 5;
		return second;
	}(),
	[] {
		eyes::Generic third;
		third.identifier = 'c';
		third.left_forward = true;
		third.right_forward = false;
		third.left_speed = 2.5;
		third.right_speed = 2.5;
		third.timed = true;
		third.duration = 5;
		return third;
	}(),
	[] {
		eyes::Generic eoc;
		eoc.identifier = 'e';
		eoc.left_forward = true;
		eoc.right_forward = true;
		eoc.left_speed = 0.0;
		eoc.right_speed = 0.0;
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
		first.left_speed = 2.0;
		first.right_speed = 2.0;
		first.timed = true;
		first.duration = 1;
		return first;
	}(),
	[] {
		eyes::Generic second;
		second.identifier = 'c';
		second.left_forward = false;
		second.right_forward = true;
		second.left_speed = 2.5;
		second.right_speed = 2.5;
		second.timed = true;
		second.duration = 5;
		return second;
	}(),
	[] {
		eyes::Generic third;
		third.identifier = 'c';
		third.left_forward = false;
		third.right_forward = true;
		third.left_speed = 2.5;
		third.right_speed = 2.5;
		third.timed = true;
		third.duration = 5;
		return third;
	}(),
	[] {
		eyes::Generic eoc;
		eoc.identifier = 'e';
		eoc.left_forward = true;
		eoc.right_forward = true;
		eoc.left_speed = 0.0;
		eoc.right_speed = 0.0;
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
		first.left_speed = 2.5;
		first.right_speed = 2.5;
		first.timed = true;
		first.duration = 5;
		return first;
	}(),
	[] {
		eyes::Generic eoc;
		eoc.identifier = 'e';
		eoc.left_forward = true;
		eoc.right_forward = true;
		eoc.left_speed = 0.0;
		eoc.right_speed = 0.0;
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
		first.left_speed = 2.5;
		first.right_speed = 2.5;
		first.timed = true;
		first.duration = 5;
		return first;
	}(),
	[] {
		eyes::Generic eoc;
		eoc.identifier = 'e';
		eoc.left_forward = true;
		eoc.right_forward = true;
		eoc.left_speed = 0.0;
		eoc.right_speed = 0.0;
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
		first.left_speed = 2.5;
		first.right_speed = 2.5;
		first.timed = true;
		first.duration = 5;
		return first;
	}(),
	[] {
		eyes::Generic second;
		second.identifier = 'c';
		second.left_forward = true;
		second.right_forward = false;
		second.left_speed = 2.5;
		second.right_speed = 2.5;
		second.timed = true;
		second.duration = 5;
		return second;
	}(),
	[] {
		eyes::Generic eoc;
		eoc.identifier = 'e';
		eoc.left_forward = true;
		eoc.right_forward = true;
		eoc.left_speed = 0.0;
		eoc.right_speed = 0.0;
		eoc.timed = true;
		eoc.duration = 0;
		return eoc;
	}()
};

static const std::vector<eyes::Generic> AUDIO_C = {
	[] {
		eyes::Generic first;
		first.identifier = 'p';
		first.left_forward = true;
		first.right_forward = true;
		first.left_speed = 0.0;
		first.right_speed = 0.0;
		first.timed = true;
		first.duration = 0;
		return first;
	}(),
	[] {
		eyes::Generic second;
		second.identifier = 'c';
		second.left_forward = true;
		second.right_forward = true;
		second.left_speed = 0.0;
		second.right_speed = 0.0;
		second.timed = true;
		second.duration = 5;
		return second;
	}(),
	[] {
		eyes::Generic third;
		third.identifier = 'k';
		third.left_forward = true;
		third.right_forward = true;
		third.left_speed = 0.0;
		third.right_speed = 0.0;
		third.timed = true;
		third.duration = 0;
		return third;
	}(),
	[] {
		eyes::Generic fourth;
		fourth.identifier = 'c';
		fourth.left_forward = true;
		fourth.right_forward = true;
		fourth.left_speed = 2.0;
		fourth.right_speed = 2.0;
		fourth.timed = false;
		fourth.duration = 40000;
		return fourth;
	}(),
	[] {
		eyes::Generic fifth;
		fifth.identifier = 't';
		fifth.left_forward = true;
		fifth.right_forward = true;
		fifth.left_speed = 0.0;
		fifth.right_speed = 0.0;
		fifth.timed = true;
		fifth.duration = 0;
		return fifth;
	}(),
	[] {
		eyes::Generic sixth;
		sixth.identifier = 'c';
		sixth.left_forward = true;
		sixth.right_forward = true;
		sixth.left_speed = 0.0;
		sixth.right_speed = 0.0;
		sixth.timed = true;
		sixth.duration = 5;
		return sixth;
	}(),
	[] {
		eyes::Generic seventh;
		seventh.identifier = 'c';
		seventh.left_forward = false;
		seventh.right_forward = false;
		seventh.left_speed = 2.0;
		seventh.right_speed = 2.0;
		seventh.timed = false;
		seventh.duration = 40000;
		return seventh;
	}(),
	[] {
		eyes::Generic eoc;
		eoc.identifier = 'e';
		eoc.left_forward = true;
		eoc.right_forward = true;
		eoc.left_speed = 0.0;
		eoc.right_speed = 0.0;
		eoc.timed = true;
		eoc.duration = 0;
		return eoc;
	}()
};
