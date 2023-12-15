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

    class PItem : public nestplacer::PlacerItem {
    public:
        PItem(const PlacerItemGeometry& geometry);
        virtual ~PItem();

        void polygon(nestplacer::PlacerItemGeometry& geometry) override;

        std::vector<trimesh::vec3> contour;
        //std::vector<std::vector<trimesh::vec3>> holes;
    };

	struct PlacerParameter
	{
		float itemGap = 1.0f;
		float binItemGap = 1.0f;
		bool rotate = false;
		float rotateAngle = 45.0f;
        bool needAlign = false;
        int align_mode = 0;
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

    NESTPLACER_API void placeFromFile(const std::string& fileName, std::vector<PlacerResultRT>& results, const BinExtendStrategy& binExtendStrategy, ccglobal::Tracer* tracer);
    NESTPLACER_API void extendFillFromFile(const std::string& fileName, std::vector<PlacerResultRT>& results, const BinExtendStrategy& binExtendStrategy, ccglobal::Tracer* tracer);
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
	NESTPLACER_API void extendFill(const std::vector<PlacerItem*>& fixed, PlacerItem* active,
		const PlacerParameter& parameter, const trimesh::box3& binBox, std::vector<PlacerResultRT>& results);

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

    class NESTPLACER_API FreeBinExtendStrategy : public BinExtendStrategy {
    public:
        FreeBinExtendStrategy(const trimesh::box3& box, float ratio);
        virtual ~FreeBinExtendStrategy();

        trimesh::box3 bounding(int index) const override;
    protected:
        trimesh::box3 m_box;
        float m_ratio;
    };
}

#endif // NESTPLACER_PLACER_1702004306712_H