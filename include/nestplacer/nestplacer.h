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
		ONELINE,
		NULLTYPE
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

	struct NestParaCInt
	{
		Clipper3r::cInt workspaceW; //Æ½Ì¨¿í
		Clipper3r::cInt workspaceH; 
		Clipper3r::cInt modelsDist;
		PlaceType packType;
		bool parallel;

		NestParaCInt()
		{
			packType = PlaceType::CENTER_TO_SIDE;
			modelsDist = 0;  
			workspaceW = 0;
			workspaceH = 0;
			parallel = true;
		}

		NestParaCInt(Clipper3r::cInt w, Clipper3r::cInt h, Clipper3r::cInt dist, PlaceType type, bool _parallel)
		{
			workspaceW = w;
			workspaceH = h;
			modelsDist = dist;
			packType = type;
			parallel = _parallel;
		}
	};

	struct NestParaFloat
	{
		trimesh::box3 workspaceBox;
		float modelsDist;
		PlaceType packType;
		bool parallel;

		NestParaFloat()
		{
			workspaceBox = trimesh::box3();
			packType = PlaceType::CENTER_TO_SIDE;
			modelsDist = 0.f;
			parallel = true;
		}

		NestParaFloat(trimesh::box3 workspace, float dist, PlaceType type, bool _parallel)
		{
			workspaceBox = workspace;
			modelsDist = dist;
			packType = type;
			parallel = _parallel;
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
		static void layout_all_nest(std::vector < std::vector<trimesh::vec3>> models, std::vector<int> modelIndices, 
			NestParaFloat para, std::function<void(int, trimesh::vec3)> modelPositionUpdateFunc);
		static bool layout_new_item(std::vector < std::vector<trimesh::vec3>> models, std::vector<trimesh::vec3> transData, 
			std::vector<trimesh::vec3> NewItem, NestParaFloat para, std::function<void(trimesh::vec3)> func);

		static bool nest2d(std::vector<NestItemer*>& items, NestParaCInt para, PlaceFunc func);
		static bool nest2d(std::vector<NestItemer*>& items, NestItemer* item, NestParaCInt para, PlaceOneFunc func);
	private:
		static bool nest2d_base(Clipper3r::Paths ItemsPaths, NestParaCInt para, std::vector<TransMatrix>& transData);
		static bool nest2d_base(Clipper3r::Paths ItemsPaths, Clipper3r::Path transData, Clipper3r::Path newItemPath, NestParaCInt para, TransMatrix& NewItemTransData);
	};
}



#endif