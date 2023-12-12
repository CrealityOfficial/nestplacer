#include "nestplacer/placer.h"
#include "data.h"
#include "conv.h"
#include "debug.h"

namespace nestplacer
{
    void convert(const PlacerItemGeometry& geometry, libnest2d::Item& item)
    {
        Clipper3r::Path contour;
        contour.reserve(geometry.outline.size());
        for (const auto& p : geometry.outline) {
            contour.emplace_back(convert(p));
        }
        Clipper3r::Paths holes;
        holes.reserve(geometry.holes.size());
        for (const auto& hole : geometry.holes) {
            holes.emplace_back();
            holes.back().reserve(hole.size());
            for (const auto& p : hole) {
                holes.back().emplace_back(convert(p));
            }
        }
        item = libnest2d::Item(contour, holes);
    }

	void place(const std::vector<PlacerItem*>& fixed, const std::vector<PlacerItem*>& actives,
		const PlacerParameter& parameter, std::vector<PlacerResultRT>& results, const BinExtendStrategy& binExtendStrategy)
	{
		std::vector<libnest2d::Item> inputs;
		for (PlacerItem* item : actives)
		{
			nestplacer::PlacerItemGeometry geometry;
			item->polygon(geometry);
            libnest2d::Item item{};
            convert(geometry, item);
            inputs.emplace_back(item);
		}
        libnest2d::Coord distance = INT2MM(parameter.itemGap);
        trimesh::box3 binBox= binExtendStrategy.bounding(0);
        libnest2d::Box box = convert(binBox);

        libnest2d::NestControl ctl;
        libnest2d::NestConfig<libnest2d::NfpPlacer, libnest2d::FirstFitSelection> config;
        config.placer_config.starting_point = libnest2d::NfpPlacer::Config::Alignment::CENTER;
        config.placer_config.alignment = libnest2d::NfpPlacer::Config::Alignment::CENTER;
        config.placer_config.setNewAlignment(1);
        int step = (int)(360.0f / parameter.rotateAngle);
        config.placer_config.rotations.clear();
        for (int i = 0; i < step; ++i)
            config.placer_config.rotations.emplace_back(
                libnest2d::Radians(libnest2d::Degrees((double)i * parameter.rotateAngle)));

        size_t bins = nest(inputs, box, distance, config, ctl);

        //return transformed items.
        for (size_t i = 0; i < inputs.size(); ++i) {
            const libnest2d::Item& item = inputs.at(i);
            PlacerResultRT pr;
            pr.rt.x = INT2MM(item.translation().X);
            pr.rt.y = INT2MM(item.translation().Y);
            pr.rt.z = item.rotation().toDegrees();
            results.emplace_back(pr);
        }

	}

	void extendFill(const std::vector<PlacerItem*>& fixed, const std::vector<PlacerItem*>& actives,
		const PlacerParameter& parameter, std::vector<PlacerResultRT>& results)
	{

	}

	YDefaultBinExtendStrategy::YDefaultBinExtendStrategy(const trimesh::box3& box, float dy)
		: BinExtendStrategy()
		, m_box(box)
		, m_dy(dy)
	{

	}

	YDefaultBinExtendStrategy::~YDefaultBinExtendStrategy()
	{

	}

	trimesh::box3 YDefaultBinExtendStrategy::bounding(int index) const
	{
		trimesh::box3 b = m_box;
		float y = b.size().y;
		b.min += y + m_dy;
		b.max += y + m_dy;
		return b;
	}
}