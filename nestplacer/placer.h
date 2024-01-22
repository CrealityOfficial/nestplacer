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
		int binIndex = -1;
	};

	struct PlacerItemGeometry
	{
		int binIndex = -1;
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
		float itemGap = 1.0f;
		float binItemGap = 1.0f;
		bool rotate = false;
		float rotateAngle = 30.0f;
        int startPoint = 0;
        /*
        @param align_mode:
        0 - CENER, 1 - BOTTOM_LEFT, 2 - BOTTOM_RIGHT,
        3 - TOP_LEFT, 4 - TOP_RIGHT, 5 -  DONT_ALIGN;
        if is DONT_ALIGN, is same with no needAlign.
        */
        bool needAlign = false;
        int alignMode = 0;
        bool concaveCal = false;
        trimesh::box3 box;
		ccglobal::Tracer* tracer = nullptr;

		//paramter for mutiPlates.
		std::vector<trimesh::box3> multiBins;
		float binDist = 10.0f;
		int curBinId = 0;

		//debug
		std::string fileName;
	};

	class BinExtendStrategy
	{
	public:
		virtual ~BinExtendStrategy() {}

		virtual trimesh::box3 bounding(int index) const = 0;
	};
	NESTPLACER_API void loadDataFromFile(const std::string& fileName, std::vector<PlacerItem*>& fixed, std::vector<PlacerItem*>& actives, PlacerParameter& parameter);
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

    class NESTPLACER_API DiagonalBinExtendStrategy : public BinExtendStrategy {
    public:
		DiagonalBinExtendStrategy(const trimesh::box3& box, float ratio);
        virtual ~DiagonalBinExtendStrategy();

        trimesh::box3 bounding(int index) const override;
    protected:
        trimesh::box3 m_box;
        float m_ratio;
    };

	class NESTPLACER_API MultiBinExtendStrategy : public BinExtendStrategy {
	public:
		MultiBinExtendStrategy(const std::vector<trimesh::box3>& boxes, float binDist = 10.0f, int priorBin = 0);
		virtual ~MultiBinExtendStrategy();

		trimesh::box3 bounding(int index) const override;
	protected:
		std::vector<trimesh::box3> m_boxes;
		float m_binDist = 10.0f;
		int m_curBinId = 0;
	};
}

#endif // NESTPLACER_PLACER_1702004306712_H