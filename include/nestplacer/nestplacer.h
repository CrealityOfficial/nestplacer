#ifndef _NESTPLACER_H
#define _NESTPLACER_H
#include "nestplacer/export.h"

#include <vector>
#include "clipper3r/clipper.hpp"
#include "trimesh2/Box.h"

#define NEST_FACTOR  10000.0

namespace nestplacer
{
	enum class PlaceType {
		CENTER_TO_SIDE,
		MID_TO_UP_DOWN,
		MID_TO_LEFT_RIGHT,
		LEFT_TO_RIGHT,
		RIGHT_TO_LEFT,
		UP_TO_DOWN,
		DOWN_TO_UP,
		ONELINE
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

	typedef std::function<void(int, const TransMatrix&)> PlaceFunc;
	typedef std::function<void(const TransMatrix&)> PlaceOneFunc;
	class NestItemer
	{
	public:
		virtual ~NestItemer() {}
		virtual const Clipper3r::Path& path() const = 0;
		virtual Clipper3r::IntPoint translate() = 0;
		virtual float rotation() = 0;
	};

	class _NESTPLACER_API NestPlacer
	{
	public:
		NestPlacer();
		~NestPlacer();
	public:
		static bool nest2d(Clipper3r::Paths ItemsPaths, Clipper3r::cInt _imageW, Clipper3r::cInt _imageH, Clipper3r::cInt _dist, PlaceType placeType, std::vector<TransMatrix>& transData);
		static void layout_all_nest(trimesh::box3 workspaceBox, std::vector<int> modelIndices,
			std::vector < std::vector<trimesh::vec3>> models, PlaceType packType, std::function<void(int, trimesh::vec3)> modelPositionUpdateFunc);
		static bool layout_new_item(std::vector < std::vector<trimesh::vec3>> models, std::vector<trimesh::vec3> transData, 
			std::vector<trimesh::vec3> NewItem, trimesh::box3 workspaceBox, float dist, std::function<void(trimesh::vec3)> func);

		static bool nest2d(std::vector<NestItemer*>& items, Clipper3r::cInt w, Clipper3r::cInt h, Clipper3r::cInt d, PlaceType type, PlaceFunc func);
		static bool nest2d(std::vector<NestItemer*>& items, NestItemer* item, Clipper3r::cInt w, Clipper3r::cInt h, Clipper3r::cInt d, PlaceType type, PlaceOneFunc func);
	private:
	};
}



#endif