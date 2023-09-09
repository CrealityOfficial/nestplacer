#ifndef _NESTPLACER_H
#define _NESTPLACER_H
#include "nestplacer/export.h"

#include "trimesh2/Box.h"
#include <functional>
#include <vector>

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
		CONCAVE_ALL,
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

	struct NestParaFloat
	{
		trimesh::box3 workspaceBox;
		float modelsDist;
		PlaceType packType;
		bool parallel;
		int rotationStep;

		NestParaFloat()
		{
			workspaceBox = trimesh::box3();
			packType = PlaceType::CENTER_TO_SIDE;
			modelsDist = 0.f;
			parallel = true;
			rotationStep = 8;
		}

		NestParaFloat(trimesh::box3 workspace, float dist, PlaceType type, bool _parallel, int _rotationStep = 8)
		{
			workspaceBox = workspace;
			modelsDist = dist;
			packType = type;
			parallel = _parallel;
			rotationStep = _rotationStep;
		}
	};

	/*所有模型布局*/
	NESTPLACER_API void layout_all_nest(const std::vector < std::vector<trimesh::vec3>>& models, std::vector<int> modelIndices,
		NestParaFloat para, std::function<void(int, trimesh::vec3)> modelPositionUpdateFunc);
	/*新增单个模型布局*/
	NESTPLACER_API bool layout_new_item(const std::vector < std::vector<trimesh::vec3>>& models, const std::vector<trimesh::vec3>& transData,
		const std::vector<trimesh::vec3>& NewItem, NestParaFloat para, std::function<void(trimesh::vec3)> func);
	/*新增多个模型布局*/
	NESTPLACER_API void layout_new_items(const std::vector < std::vector<trimesh::vec3>>& models, const std::vector<trimesh::vec3>& transData,
		const std::vector < std::vector<trimesh::vec3>>& NewItems, NestParaFloat para, std::function<void(int, trimesh::vec3)> func);
}



#endif