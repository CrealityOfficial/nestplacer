#ifndef NESTPLACER_PLACER_1702004306712_H
#define NESTPLACER_PLACER_1702004306712_H
#include "nestplacer/export.h"
#include "ccglobal/tracer.h"

#include "trimesh2/Box.h"
#include <functional>
#include <vector>

namespace nestplacer
{
	typedef trimesh::vec3 PlacerResultRT;  // x, y translation  z rotation angle

	struct PlacerItemGeometry
	{
		std::vector<trimesh::vec3> outline;
		std::vector<std::vector<trimesh::vec3>> holes;
	};

	class PlacerItem
	{
	public:
		virtual ~PlacerItem() {}
		virtual void outline(PlacerItemGeometry& geometry) = 0;  // loop polygon
	};

	struct PlacerParameter
	{
		float itemGap = 0.0f;
		float binItemGap = 0.0f;
		bool rotate = true;
		float rotateAngle = 45.0f;
		trimesh::box2 binBox = trimesh::box2(trimesh::vec2(0.0f, 0.0f), trimesh::vec2(100.0f, 100.0f));
		ccglobal::Tracer* tracer = nullptr;

		//debug
		std::string fileName;
	};

	class BinExtendStrategy
	{
	public:
		virtual ~BinExtendStrategy() {}

		virtual trimesh::box2 bounding(int index) = 0;
	};

	/// <summary>
	/// 
	/// </summary>
	/// <param name="fixed"></param>
	/// <param name="actives"></param>
	/// <param name="parameter"></param>
	/// <param name="results"></param>   result rt, same size with actives
	/// <param name="binExtendStrategy"></param>  bin extend strategy
	/// <returns></returns>
	NESTPLACER_API void place(const std::vector<PlacerItem>& fixed, const std::vector<PlacerItem>& actives,
		const PlacerParameter& parameter, std::vector<PlacerResultRT>& results, BinExtendStrategy* binExtendStrategy = nullptr);

	/// <summary>
	/// 
	/// </summary>
	/// <param name="fixed"></param>
	/// <param name="actives"></param>
	/// <param name="parameter"></param>
	/// <param name="results"></param>  result clone positions
	/// <returns></returns>
	NESTPLACER_API void extendFill(const std::vector<PlacerItem>& fixed, const std::vector<PlacerItem>& actives,
		const PlacerParameter& parameter, std::vector<PlacerResultRT>& results);
}

#endif // NESTPLACER_PLACER_1702004306712_H