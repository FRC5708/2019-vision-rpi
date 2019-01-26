#include "vision.hpp"
#include "grip.hpp"


bool isNanOrInf(double in) {
	return isnan(in) || isinf(in);
}

struct AngularCoord {
	double x; double y;
};
// this can probably be simplfied
AngularCoord toAngularCoord(double pixX, double pixY, double pixFocalLength, int pixImageWidth, int pixImageHeight) {
	double centeredX = pixX - pixImageWidth / 2;
	double centeredY = pixImageHeight / 2 - pixY;
	
	// get coords, with pole on center of image
	double lng = atan2(centeredY, centeredX);
	double lat = atan2(sqrt(centeredX*centeredX + centeredY*centeredY), pixFocalLength);
	std::cout << "pixel: " << pixX << "," << pixY
	<< " lat: " << lat/M_PI*180 << ", lng: " << lng/M_PI*180 << std::endl;
	
	// then convert so poles are on left and right: https://en.wikipedia.org/wiki/Spherical_coordinate_system
	
	double radX = atan2(sin(lat) * cos(lng), cos(lat));
	double radY = M_PI_2 - acos(sin(lat)*sin(lng));
	std::cout << "angular coord: " << radX << ", " << radY << std::endl;
	return { radX, radY };
}

struct ProcessRectsResult {
	bool success;
	VisionData calcs;
};
// this function uses hungarian notation for numbers. It's confusing otherwise
ProcessRectsResult processRects(cv::Rect left, cv::Rect right, int pixImageWidth, int pixImageHeight) {
	VisionData result;
	
	// all the constants
	const double inchTapesHeight = 5.5; // from bottom to top of tapes
	const double inchTapesApart = 8 + 5.5*sin(14.5/180*M_PI); // from outermost edge
	const double inchHatchTapesAboveGround = 2*12+7.5 - inchTapesHeight;
	const double inchPortTapesAboveGround = 3*12+3.125 - inchTapesHeight;
	
	// maximum difference in calculated and actual tape height above ground
	const double inchTapeHeightTolerance = 1000; // for testing

	const double inchMinDistance = 1;
	
	// need precise values of these
	const double radCameraPitch = 0;//(10/180)*M_PI; // above horizontal
	const double inchCameraHeight = 9.5; // from ground
	
	
	// todo: use actual calibration data
	// see: https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
	// right now, it assumes perfect pinhole camera
	
	const double radFOV = (30.0/180.0)*M_PI; // changed to 30 for testing
	const double pixFocalLength = tan((M_PI_2) - radFOV/2) * (pixImageWidth / 2); // pixels. Estimated from the camera's FOV spec.
	
	struct { AngularCoord tl, tr, bl, br; } radTapeQuad = {
		toAngularCoord(left.tl().x, left.tl().y, pixFocalLength, pixImageWidth, pixImageHeight),
		toAngularCoord(right.br().x, right.tl().y, pixFocalLength, pixImageWidth, pixImageHeight),
		toAngularCoord(left.tl().x, left.br().y, pixFocalLength, pixImageWidth, pixImageHeight),
		toAngularCoord(right.br().x, right.br().y, pixFocalLength, pixImageWidth, pixImageHeight)
	};
	assert(abs(radTapeQuad.tl.x - radTapeQuad.bl.x) < 0.00000001
		   && abs(radTapeQuad.tr.x - radTapeQuad.br.x) < 0.00000001);
	
	
	double inchLeftDistance = inchTapesHeight /
	(tan(radCameraPitch + radTapeQuad.tl.y) - tan(radCameraPitch + radTapeQuad.bl.y));
	double inchRightDistance = inchTapesHeight /
	(tan(radCameraPitch + radTapeQuad.tr.y) - tan(radCameraPitch + radTapeQuad.br.y));
	
	double radWidth = radTapeQuad.tr.x - radTapeQuad.tl.x;
	assert(radWidth > 0);
	
	// Uses law of sines to get the angle, A, between leftDistance and tapeWidth
	// then uses law of cosines to get the remaining side of the leftDistance-A-(tapeWidth/2) triangle
	// which is centerDistance
	double inchCenterDistance = sqrt(pow(inchLeftDistance, 2) + pow(inchTapesApart/2, 2) -
							   inchLeftDistance*inchTapesApart*
							   sqrt(1 - pow(sin(radWidth) / inchTapesApart * inchRightDistance, 2))); // cos(A)
	
	// alternate version, for testing, that doesn't take into account the observed tape width
	/*double inchCenterDistance = sqrt(pow(inchLeftDistance, 2) + pow(inchTapesApart/2, 2)
	 - inchTapesApart/2/inchRightDistance * 
	 (pow(inchLeftDistance, 2) + pow(inchRightDistance, 2) - pow(inchTapesApart, 2)));*/

	// in the ideal model, these are equal.
	double inchCalcTapesAboveGroundLeft  = inchLeftDistance  * tan(radCameraPitch + radTapeQuad.bl.y) + inchCameraHeight;
	double inchCalcTapesAboveGroundRight = inchRightDistance * tan(radCameraPitch + radTapeQuad.br.y) + inchCameraHeight;
	
	double inchCalcTapesAboveGround = (inchCalcTapesAboveGroundLeft + inchCalcTapesAboveGroundRight) / 2;

	std::cout << "distance: left: " << inchLeftDistance << " right: " << inchRightDistance << 
		" center: " << inchCenterDistance << " height: " << inchCalcTapesAboveGround << " radWidth: " << radWidth << std::endl;

	if (inchCenterDistance > inchMinDistance &&
		abs(inchCalcTapesAboveGroundLeft - inchCalcTapesAboveGroundRight) < inchTapeHeightTolerance) {

		if (abs(inchCalcTapesAboveGround - inchHatchTapesAboveGround) < inchTapeHeightTolerance) {
			result.isPort = false;
		}
		else if (abs(inchCalcTapesAboveGround - inchPortTapesAboveGround) < inchTapeHeightTolerance) {
			result.isPort = true;
		}
		else return { false, {}};
	}
	else return { false, {}};
	
	result.distance = inchCenterDistance;
	
	// sin(A) / centerDistance == sin(90 + tapeAngle) / leftDistance
	result.tapeAngle = asin(sin(radWidth) / inchTapesApart * inchRightDistance
							  / inchCenterDistance * inchLeftDistance) - M_PI_2;
	
	result.robotAngle = radTapeQuad.tl.x + asin(sin(radWidth) / inchTapesApart * inchRightDistance
												  / inchCenterDistance * (inchTapesApart / 2));
	std::cout << "tapeAngle: " << result.tapeAngle << " robotAngle: " << result.robotAngle << '\n' << std::endl;
	return { true, result };
}

std::vector<VisionTarget> doVision(cv::Mat image) {
	std::vector<VisionTarget> results;

	grip::ContourGrip finder;
	finder.Process(image);
	std::vector<std::vector<cv::Point> >* contours = finder.GetFilterContoursOutput();

	
	// these rectangles are vertical, the vision targets are at an angle... it doesn't matter
	std::vector<cv::Rect> rects;

	const float minRectWidth = 10; //pixels 
	const float minRectHeight= 10;
	for (auto i : *contours) {
		cv::Rect rect = cv::boundingRect(i);
		if (rect.width >= minRectWidth && rect.height >= minRectHeight) {
			rects.push_back(rect);
		}
	}


	const float rectSizeDifferenceTolerance = 0.2; // fraction of width/height
	const float rectDistanceTolerance = 5; // multiplier of the width of one rectangle, that the whole vision target can be

	// find rects that are close enough in size and distance
	for (auto left : rects) {
		for (auto right : rects) {
			if (left != right &&
				left.br().x < right.tl().x &&
				left.tl().x + (left.width + right.width) / 2 * rectDistanceTolerance > right.br().x &&
			    abs(1 - left.width / right.width) < rectSizeDifferenceTolerance &&
				abs(1 - left.height / right.height) < rectSizeDifferenceTolerance) {

				ProcessRectsResult result = processRects(left, right, image.rows, image.cols);
				
				if (isNanOrInf(result.calcs.distance) || isNanOrInf(result.calcs.robotAngle) || isNanOrInf(result.calcs.distance)) {
					std::cout << "encountered NaN or Infinity" << std::endl;
				}
				else if (result.success) results.push_back({ result.calcs, left, right });
			}
		}
	}
	// lowest distance first
	std::sort(results.begin(), results.end(), [](VisionTarget a, VisionTarget b) -> bool {
		return a.calcs.distance < b.calcs.distance;
	});
	
	return results;
}


