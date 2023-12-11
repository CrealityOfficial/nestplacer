#include "nestplacer/placer.h"
#include "data.h"
#include "conv.h"
#include "debug.h"

namespace nestplacer
{
	void place(const std::vector<PlacerItem*>& fixed, const std::vector<PlacerItem*>& actives,
		const PlacerParameter& parameter, std::vector<PlacerResultRT>& results, const BinExtendStrategy& binExtendStrategy)
	{
		std::vector<libnest2d::Item> inputs;
		for (PlacerItem* item : actives)
		{
			nestplacer::PlacerItemGeometry geometry;
			item->outline(geometry);
		}

	}

	void extendFill(const std::vector<PlacerItem*>& fixed, const std::vector<PlacerItem*>& actives,
		const PlacerParameter& parameter, std::vector<PlacerResultRT>& results)
	{

	}

	YDefaultBinExtendStrategy::YDefaultBinExtendStrategy(const trimesh::box2& box, float dy)
		: BinExtendStrategy()
		, m_box(box)
		, m_dy(dy)
	{

	}

	YDefaultBinExtendStrategy::~YDefaultBinExtendStrategy()
	{

	}

	trimesh::box2 YDefaultBinExtendStrategy::bounding(int index) const
	{
		trimesh::box2 b = m_box;
		float y = b.size().y;
		b.min += y + m_dy;
		b.max += y + m_dy;
		return b;
	}
}