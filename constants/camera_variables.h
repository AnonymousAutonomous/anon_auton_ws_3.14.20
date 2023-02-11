#include <string>
#include <unordered_map>

enum CameraVariable
{
	TOP_BAND_WIDTH,
	SIDE_BAND_WIDTH,
	IMAGE_WIDTH,
	IMAGE_HEIGHT,
	BRIGHTNESS_THRESHOLD,
	SIDE_PERCENT_THRESHOLD,
	TOP_PERCENT_THRESHOLD,
	NUM_MIDDLE_PIXELS,
};

static std::unordered_map<std::string, CameraVariable> CAMERA_VARIABLE_STRING_TO_ENUM = {
	{"topBandWidth", TOP_BAND_WIDTH},
	{"sideBandWidth", SIDE_BAND_WIDTH},
	{"imageWidth", IMAGE_WIDTH},
	{"imageHeight", IMAGE_HEIGHT},
	{"brightnessThreshold", BRIGHTNESS_THRESHOLD},
	{"sidePercentThreshold", SIDE_PERCENT_THRESHOLD},
	{"topPercentThreshold", TOP_PERCENT_THRESHOLD},
	{"numMiddlePixels", NUM_MIDDLE_PIXELS},
};
