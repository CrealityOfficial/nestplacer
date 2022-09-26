#ifndef _NESTPLACER_H
#define _NESTPLACER_H
#include "nestplacer/export.h"

#include <vector>
#include "clipper3r/clipper.hpp"
#include "trimesh2/Box.h"

#define NEST_FACTOR  100.0

namespace nestplacer
{
	enum class PlaceType {
		CENTER_TO_SIDE,
		ALIGNMENT,
		ONELINE,
		CONCAVE,
		TOP_TO_BOTTOM,
		BOTTOM_TO_TOP,
		LEFT_TO_RIGHT,
		RIGHT_TO_LEFT,
		NULLTYPE
	};

	enum class StartPoint {
		CENTER,
		TOP_LEFT,
		TOP_RIGHT,
		BOTTOM_LEFT,
		BOTTOM_RIGHT,
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

		TransMatrix(Clipper3r::cInt _x, Clipper3r::cInt _y, double _angle)
		{
			x = _x;
			y = _y;
			rotation = _angle;
		}

		void merge(TransMatrix mat)
		{
			double r = mat.rotation* M_PIf / 180;
			double c = cos(r);
			double s = sin(r);
			int x_blk = c * x - s * y;
			int y_blk = s * x + c * y;
			x = x_blk + mat.x;
			y = y_blk + mat.y;
			rotation += mat.rotation;
		}
	};

	struct NestParaCInt
	{
		Clipper3r::cInt workspaceW; //Æ½Ì¨¿í
		Clipper3r::cInt workspaceH; 
		Clipper3r::cInt modelsDist;
		PlaceType packType;
		bool parallel;
		StartPoint sp;

		NestParaCInt()
		{
			packType = PlaceType::CENTER_TO_SIDE;
			modelsDist = 0;  
			workspaceW = 0;
			workspaceH = 0;
			parallel = true;
			sp = StartPoint::NULLTYPE;
		}

		NestParaCInt(Clipper3r::cInt w, Clipper3r::cInt h, Clipper3r::cInt dist, PlaceType type, bool _parallel, StartPoint _sp)
		{
			workspaceW = w;
			workspaceH = h;
			modelsDist = dist;
			packType = type;
			parallel = _parallel;
			sp = _sp;
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

		static bool nest2d_base(Clipper3r::Paths ItemsPaths, NestParaCInt para, std::vector<TransMatrix>& transData);
		static bool nest2d_base(Clipper3r::Paths ItemsPaths, Clipper3r::Path transData, Clipper3r::Path newItemPath, NestParaCInt para, TransMatrix& NewItemTransData);	};
}



#endif