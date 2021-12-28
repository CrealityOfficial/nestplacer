#ifndef _NESTPLACER_H
#define _NESTPLACER_H
#include "nestplacer/export.h"

#include <vector>

#include <libnest2d/libnest2d.hpp>

namespace nestplacer
{
	enum class PlaceType {
		CENTER_TO_SIDE,
		MID_TO_UP_DOWN,
		MID_TO_LEFT_RIGHT,
		LEFT_TO_RIGHT,
		RIGHT_TO_LEFT,
		UP_TO_DOWN,
		DOWN_TO_UP
	};

	struct TransMatrix
	{
		Clipper3r::cInt x;
		Clipper3r::cInt y;
		double rotation;

		TransMatrix()
		{
			x = 0;
			y = 0;
			rotation = 0.;
		}
	};

	class _NESTPLACER_API NestPlacer
	{
	public:
		NestPlacer();
		~NestPlacer();
	public:
		static bool nest2d(Clipper3r::Paths ItemsPaths, int _imageW, int _imageH, int _dist, PlaceType placeType, std::vector<TransMatrix>& transData);

	private:

	};
}



#endif