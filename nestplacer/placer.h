#ifndef NESTPLACER_PLACER_1702004306712_H
#define NESTPLACER_PLACER_1702004306712_H
#include "nestplacer/export.h"
#include "ccglobal/tracer.h"

#include "trimesh2/Box.h"
#include <functional>
#include <vector>

namespace nestplacer
{
	struct  PlacerResultRT 
	{
		trimesh::vec3 rt; // x, y translation  z rotation angle
		int binIndex = 0;
	};

	struct PlacerItemGeometry
	{
		std::vector<trimesh::vec3> outline;
		std::vector<std::vector<trimesh::vec3>> holes;
	};

	class PlacerItem
	{
	public:
		virtual ~PlacerItem() {}
		virtual void polygon(PlacerItemGeometry& geometry) = 0;  // loop polygon
	};

	struct PlacerParameter
	{
		float itemGap = 0.0f;
		float binItemGap = 0.0f;
		bool rotate = true;
		float rotateAngle = 45.0f;
		ccglobal::Tracer* tracer = nullptr;

		//debug
		std::string fileName;
	};

	class BinExtendStrategy
	{
	public:
		virtual ~BinExtendStrategy() {}

		virtual trimesh::box3 bounding(int index) const = 0;
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
	NESTPLACER_API void place(const std::vector<PlacerItem*>& fixed, const std::vector<PlacerItem*>& actives,
		const PlacerParameter& parameter, std::vector<PlacerResultRT>& results, const BinExtendStrategy& binExtendStrategy);

	/// <summary>
	/// 
	/// </summary>
	/// <param name="fixed"></param>
	/// <param name="actives"></param>
	/// <param name="parameter"></param>
	/// <param name="results"></param>  result clone positions
	/// <returns></returns>
	NESTPLACER_API void extendFill(const std::vector<PlacerItem*>& fixed, const std::vector<PlacerItem*>& actives,
		const PlacerParameter& parameter, std::vector<PlacerResultRT>& results);

	class NESTPLACER_API YDefaultBinExtendStrategy : public BinExtendStrategy
	{
	public:
		YDefaultBinExtendStrategy(const trimesh::box3& box, float dy);
		virtual ~YDefaultBinExtendStrategy();

		trimesh::box3 bounding(int index) const override;
	protected:
		trimesh::box3 m_box;
		float m_dy;
	};
}

#endif // NESTPLACER_PLACER_1702004306712_H