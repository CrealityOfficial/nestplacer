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

	NESTPLACER_API void layout_all_nest(const ConcaveItems& models, const NestConcaveParam& param);
}



#endif