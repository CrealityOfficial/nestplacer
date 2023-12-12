#include "nestplacer/placer.h"
#include "data.h"
#include "debug.h"

#define INT2UM(x) (static_cast<double>(x) / 1000000.0)
#define UM2INT(x) (static_cast<Clipper3r::cInt>((x) * 1000000.0 + 0.5 * (((x) > 0) - ((x) < 0))))

namespace nestplacer
{
    Clipper3r::IntPoint convertPoint(const trimesh::vec3& point)
    {
        return Clipper3r::IntPoint(UM2INT(point.x), UM2INT(point.y));
    }

    libnest2d::_Box<libnest2d::PointImpl> convertBox(const trimesh::box3& b)
    {
        Clipper3r::IntPoint minPoint(convertPoint(b.min));
        Clipper3r::IntPoint maxPoint(convertPoint(b.max));
        Clipper3r::IntPoint rect = maxPoint - minPoint;

        libnest2d::_Box<libnest2d::PointImpl> binBox
            = libnest2d::_Box<libnest2d::PointImpl>(rect.X, rect.Y, convertPoint(b.center()));
        return binBox;
    }

    void convertItem(const PlacerItemGeometry& geometry, libnest2d::Item& item)
    {
        Clipper3r::Path contour;
        contour.reserve(geometry.outline.size());
        for (const auto& p : geometry.outline) {
            contour.emplace_back(convertPoint(p));
        }
        Clipper3r::Paths holes;
        holes.reserve(geometry.holes.size());
        for (const auto& hole : geometry.holes) {
            holes.emplace_back();
            holes.back().reserve(hole.size());
            for (const auto& p : hole) {
                holes.back().emplace_back(convertPoint(p));
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
            convertItem(geometry, item);
            inputs.emplace_back(item);
		}
        libnest2d::Coord distance = UM2INT(parameter.itemGap);

        libnest2d::NestControl ctl;
        libnest2d::NestConfig<libnest2d::NfpPlacer, libnest2d::FirstFitSelection> config;
        config.placer_config.starting_point = libnest2d::NfpPlacer::Config::Alignment::CENTER;
        config.placer_config.alignment = libnest2d::NfpPlacer::Config::Alignment::CENTER;
        
        auto box_func = [&binExtendStrategy](const int& index) {
            trimesh::box3 binBox = binExtendStrategy.bounding(index);
            libnest2d::Box box = convertBox(binBox);
            return box;
        };
        config.placer_config.box_function = box_func;
        
        config.placer_config.setNewAlignment(1);
        int step = (int)(360.0f / parameter.rotateAngle);
        config.placer_config.rotations.clear();
        for (int i = 0; i < step; ++i)
            config.placer_config.rotations.emplace_back(
                libnest2d::Radians(libnest2d::Degrees((double)i * parameter.rotateAngle)));

        size_t bins = nest(inputs, box_func(0), distance, config, ctl);

        //return transformed items.
        for (size_t i = 0; i < inputs.size(); ++i) {
            const libnest2d::Item& item = inputs.at(i);
            PlacerResultRT pr;
            pr.rt.x = INT2UM(item.translation().X);
            pr.rt.y = INT2UM(item.translation().Y);
            pr.rt.z = item.rotation().toDegrees();
            pr.binIndex = item.binId();
            results.emplace_back(pr);
        }

	}

	void extendFill(const std::vector<PlacerItem*>& fixed, const std::vector<PlacerItem*>& actives,
		const PlacerParameter& parameter, const trimesh::box3& binBox, std::vector<PlacerResultRT>& results)
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
        b.min.y += (float)index * (y + m_dy);
        b.max.y += (float)index * (y + m_dy);
		return b;
	}

 
    FreeBinExtendStrategy::FreeBinExtendStrategy(const trimesh::box3& box, float rate)
        : BinExtendStrategy()
        , m_box(box)
        , m_ratio(rate)
    {
    }

    FreeBinExtendStrategy::~FreeBinExtendStrategy()
    {
    }

    trimesh::box3 FreeBinExtendStrategy::bounding(int index) const
    {
        trimesh::box3 b = m_box;
        trimesh::vec3 dir = b.size();
        b.min += (float)index * dir * (1.0 + m_ratio);
        b.max += (float)index * dir * (1.0 + m_ratio);
        return b;
    }
}