#ifndef _NESTPLACER_H
#define _NESTPLACER_H
#include "nestplacer/export.h"

#include <vector>

#include <libnest2d/libnest2d.hpp>
#include "trimesh2/Box.h"

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
		static void layout_all_nest(trimesh::box3 workspaceBox, std::vector<int> modelIndices,
			std::vector < std::vector<trimesh::vec3>> models, std::function<void(int, trimesh::vec3)> modelPositionUpdateFunc);
	private:

	};
}



#endif