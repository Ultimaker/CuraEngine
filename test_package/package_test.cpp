#include "utils/LinearAlg2D.h"

int main(int argc, char** argv)
{
	// Pick a function that's not defined in the header.
	const float angle =
		cura::LinearAlg2D::getAngleLeft
		(
			cura::Point(1.0, 1.0),
			cura::Point(1.0, 2.0),
			cura::Point(3.0, 3.0)
		);
	return (angle != 0.f) ? 0 : 1;
}
