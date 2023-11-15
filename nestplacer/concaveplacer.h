#ifndef CONCAVE_NESTPLACER_H
#define CONCAVE_NESTPLACER_H
#include "nestplacer/nestplacer.h"

namespace nestplacer
{
	typedef std::vector<trimesh::vec3> ConcaveItem;
	typedef std::vector<ConcaveItem> ConcaveItems;

	typedef trimesh::vec3 NestRT; 
	typedef std::function<void(int, const NestRT& rt)> NestCallback;
	struct NestConcaveParam
	{
		trimesh::box3 box;
		float distance = 0.0f;
		float eDistance = 0.0f;
		PlaceType packType;
		float rotationAngle = 20.0f;
		NestCallback callback = {};
		ccglobal::Tracer* tracer = nullptr;
	};

	typedef std::vector<trimesh::vec3> DebugContour;
	typedef std::vector<DebugContour> DebugContours;

	struct DebugPolygon
	{
		DebugContour outline;
		DebugContours holes;
	};

	class ConcaveNestDebugger
	{
	public:
		virtual ~ConcaveNestDebugger() {}

		virtual void onNPFs(const std::vector<DebugPolygon>& nfps) = 0;
	};

	NESTPLACER_API void layout_all_nest(const ConcaveItems& models, const NestConcaveParam& param,
		ConcaveNestDebugger* debugger = nullptr);
}



#endif