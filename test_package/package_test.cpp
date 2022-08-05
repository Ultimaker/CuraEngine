#include "settings/Settings.h"  // For the default definition of 'CURA_ENGINE_VERSION' if none is given.
#include "utils/LinearAlg2D.h"  // For testing calling a simple non-header function.

#include <cstdio>

int main(int argc, char** argv)
{
	// Test which version do we have here?
	std::fprintf(stdout, "test-package called for version: %s\n", CURA_ENGINE_VERSION);

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
