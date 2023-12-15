#include "msbase/utils/trimeshserial.h"
#include "nestplacer/placer.h"
#include "data.h"
#include "debug.h"

#define INT2UM(x) (static_cast<double>(x) / 1000000.0)
#define UM2INT(x) (static_cast<Clipper3r::cInt>((x) * 1000000.0 + 0.5 * (double((x) > 0) - ((x) < 0))))

namespace nestplacer
{
    class NestInput : public ccglobal::Serializeable {
    public:
        std::vector<nestplacer::PlacerItemGeometry> fixed;
        std::vector<nestplacer::PlacerItemGeometry> actives;
        PlacerParameter param;
        trimesh::box3 box;

        NestInput()
        {

        }
        ~NestInput()
        {

        }

        int version() override
        {
            return 0;
        }
        bool save(std::fstream& out, ccglobal::Tracer* tracer) override
        {
            msbase::CXNDGeometrys fgeometrys;
            fgeometrys.reserve(fixed.size());
            for (auto& fitem : fixed) {
                msbase::CXNDGeometry geo;
                geo.contour.swap(fitem.outline);
                geo.holes.swap(fitem.holes);
                fgeometrys.emplace_back(geo);
            }
            msbase::saveGeometrys(out, fgeometrys);

            msbase::CXNDGeometrys ageometrys;
            ageometrys.reserve(actives.size());
            for (auto& aitem : actives) {
                msbase::CXNDGeometry geo;
                geo.contour.swap(aitem.outline);
                geo.holes.swap(aitem.holes);
                ageometrys.emplace_back(geo);
            }
            msbase::saveGeometrys(out, ageometrys);
            ccglobal::cxndSaveT(out, box.min);
            ccglobal::cxndSaveT(out, box.max);

            ccglobal::cxndSaveT(out, param.itemGap);
            ccglobal::cxndSaveT(out, param.binItemGap);
            ccglobal::cxndSaveT(out, param.rotate);
            ccglobal::cxndSaveT(out, param.rotateAngle);
            ccglobal::cxndSaveT(out, param.needAlign);
            ccglobal::cxndSaveT(out, param.align_mode);
            
            return true;
        }

        bool load(std::fstream& in, int ver, ccglobal::Tracer* tracer) override
        {
            if (ver == 0) {
                msbase::CXNDGeometrys fgeometrys;
                msbase::loadGeometrys(in, fgeometrys);
                fixed.reserve(fgeometrys.size());
                for (auto& geometry : fgeometrys) {
                    PlacerItemGeometry pitem;
                    pitem.outline.swap(geometry.contour);
                    pitem.holes.swap(geometry.holes);
                    fixed.emplace_back(pitem);
                }

                msbase::CXNDGeometrys ageometrys;
                msbase::loadGeometrys(in, ageometrys);
                actives.reserve(ageometrys.size());
                for (auto& geometry : ageometrys) {
                    PlacerItemGeometry pitem;
                    pitem.outline.swap(geometry.contour);
                    pitem.holes.swap(geometry.holes);
                    actives.emplace_back(pitem);
                }
                ccglobal::cxndLoadT(in, box.min);
                ccglobal::cxndLoadT(in, box.max);

                ccglobal::cxndLoadT(in, param.itemGap);
                ccglobal::cxndLoadT(in, param.binItemGap);
                ccglobal::cxndLoadT(in, param.rotate);
                ccglobal::cxndLoadT(in, param.rotateAngle);
                ccglobal::cxndLoadT(in, param.needAlign);
                ccglobal::cxndLoadT(in, param.align_mode);
                return true;
            }
            return false;
        }
    };

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

    void convertPolygon(const PlacerItemGeometry& geometry, Clipper3r::Polygon& poly)
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
        poly.Contour.swap(contour);
        poly.Holes.swap(holes);
    }

    PItem::PItem(const nestplacer::PlacerItemGeometry& geometry)
    {
        contour = geometry.outline;
        //holes = geometry.holes;
    }

    PItem::~PItem()
    {

    }

    void PItem::polygon(nestplacer::PlacerItemGeometry& geometry)
    {
        geometry.holes.clear();
        geometry.outline.swap(geometry.outline);
    }

    void placeFromFile(const std::string& fileName, std::vector<PlacerResultRT>& results, const BinExtendStrategy& binExtendStrategy, ccglobal::Tracer* tracer)
    {
        NestInput input;
        if (!ccglobal::cxndLoad(input, fileName, tracer)) {
            LOGE("placeFromFile load error [%s]", fileName.c_str());
            return;
        }
        std::vector<PlacerItem*> fixed, actives;
        fixed.reserve(input.fixed.size());
        actives.reserve(input.actives.size());
        for (const auto& fix : input.fixed) {
            PItem pitem(fix);
            fixed.emplace_back(&pitem);
        }
        for (const auto& active : input.actives) {
            PItem pitem(active);
            actives.emplace_back(&pitem);
        }
        PlacerParameter param = input.param;
        return place(fixed, actives, param, results, binExtendStrategy);
    }

    void extendFillFromFile(const std::string& fileName, std::vector<PlacerResultRT>& results, const BinExtendStrategy& binExtendStrategy,  ccglobal::Tracer* tracer)
    {
        NestInput input;
        if (!ccglobal::cxndLoad(input, fileName, tracer)) {
            LOGE("placeFromFile load error [%s]", fileName.c_str());
            return;
        }
        std::vector<PlacerItem*> fixed, actives;
        fixed.reserve(input.fixed.size());
        actives.reserve(input.actives.size());
        for (const auto& fix : input.fixed) {
            PItem pitem(fix);
            fixed.emplace_back(&pitem);
        }
        for (const auto& active : input.actives) {
            PItem pitem(active);
            actives.emplace_back(&pitem);
        }
        PlacerItem* active = actives.front();
        PlacerParameter param = input.param;
        trimesh::box3 box = input.box;
        return extendFill(fixed, active, param, box, results);
    }

    void place(const std::vector<PlacerItem*>& fixed, const std::vector<PlacerItem*>& actives,
		const PlacerParameter& parameter, std::vector<PlacerResultRT>& results, const BinExtendStrategy& binExtendStrategy)
	{
        if (!parameter.fileName.empty() && (!fixed.empty() || !actives.empty())) {
            NestInput input;
            std::vector<nestplacer::PlacerItemGeometry> pfixed;
            std::vector<nestplacer::PlacerItemGeometry> pactives;
            pfixed.reserve(fixed.size());
            for (PlacerItem* pitem : fixed) {
                nestplacer::PlacerItemGeometry geometry;
                pitem->polygon(geometry);
                pfixed.emplace_back(geometry);
            }
            pactives.reserve(actives.size());
            for (PlacerItem* pitem : actives) {
                nestplacer::PlacerItemGeometry geometry;
                pitem->polygon(geometry);
                pactives.emplace_back(geometry);
            }
            input.fixed.swap(pfixed);
            input.actives.swap(pactives);
            input.param = parameter;
            ccglobal::cxndSave(input, parameter.fileName, parameter.tracer);
        }
		std::vector<libnest2d::Item> inputs;
        inputs.reserve(fixed.size() + actives.size());
        for (PlacerItem* pitem : fixed) {
            nestplacer::PlacerItemGeometry geometry;
            pitem->polygon(geometry);
            Clipper3r::Polygon sh;
            convertPolygon(geometry, sh);
            libnest2d::Item item(sh);
            item.markAsFixedInBin(0);
            inputs.emplace_back(item);
        }
		for (PlacerItem* pitem : actives){
			nestplacer::PlacerItemGeometry geometry;
			pitem->polygon(geometry);
            Clipper3r::Polygon sh;
            convertPolygon(geometry, sh);
            libnest2d::Item item(sh);
            inputs.emplace_back(item);
		}
        libnest2d::Coord itemGap = UM2INT(parameter.itemGap);
        libnest2d::Coord edgeGap = UM2INT(parameter.binItemGap);
        libnest2d::NestControl ctl;
        libnest2d::NestConfig<libnest2d::NfpPlacer, libnest2d::FirstFitSelection> config;
        config.placer_config.starting_point = libnest2d::NfpPlacer::Config::Alignment::CENTER;
        config.placer_config.binItemGap = edgeGap;
        if (parameter.needAlign) {
            config.placer_config.setAlignment(parameter.align_mode);
        } else {
            config.placer_config.alignment= libnest2d::NfpPlacer::Config::Alignment::DONT_ALIGN;
        }

        auto box_func = [&binExtendStrategy](const int& index) {
            trimesh::box3 binBox = binExtendStrategy.bounding(index);
            libnest2d::Box box = convertBox(binBox);
            return box;
        };
        config.placer_config.box_function = box_func;
        config.placer_config.needNewBin = true;

        config.placer_config.setNewAlignment(1);
        if (parameter.rotate) {
            int step = (int)(360.0f / parameter.rotateAngle);
            config.placer_config.rotations.clear();
            for (int i = 0; i < step; ++i)
                config.placer_config.rotations.emplace_back(
                    libnest2d::Radians(libnest2d::Degrees((double)i * parameter.rotateAngle)));
        }

        size_t bins = nest(inputs, box_func(0), itemGap, config, ctl);

        //return transformed items.
        std::vector<PlacerResultRT> activeRts;
        activeRts.reserve(actives.size());
        for (size_t i = 0; i < inputs.size(); ++i) {
            const libnest2d::Item& item = inputs.at(i);
            PlacerResultRT pr;
            pr.rt.x = INT2UM(item.translation().X);
            pr.rt.y = INT2UM(item.translation().Y);
            pr.rt.z = item.rotation().toDegrees();
            pr.binIndex = item.binId();
            if (item.isFixed()) results.emplace_back(pr);
            else activeRts.emplace_back(pr);
        }
        results.insert(results.end(), activeRts.begin(), activeRts.end());
	}

	void extendFill(const std::vector<PlacerItem*>& fixed, PlacerItem* active,
		const PlacerParameter& parameter, const trimesh::box3& binBox, std::vector<PlacerResultRT>& results)
	{
        if (!parameter.fileName.empty() && (!fixed.empty() || !active)) {
            NestInput input;
            std::vector<nestplacer::PlacerItemGeometry> pfixed;
            std::vector<nestplacer::PlacerItemGeometry> pactives;
            pfixed.reserve(fixed.size());
            for (PlacerItem* pitem : fixed) {
                nestplacer::PlacerItemGeometry geometry;
                pitem->polygon(geometry);
                pfixed.emplace_back(geometry);
            }

            {
                nestplacer::PlacerItemGeometry geometry;
                active->polygon(geometry);
                pactives.emplace_back(geometry);
            }
            input.fixed.swap(pfixed);
            input.actives.swap(pactives);
            input.param = parameter;
            input.box = binBox;
            ccglobal::cxndSave(input, parameter.fileName, parameter.tracer);
        }
        std::vector<libnest2d::Item> inputs;
        libnest2d::Box box = convertBox(binBox);
        {
            if (!active) return;
            libnest2d::Coord binArea = box.area();
            nestplacer::PlacerItemGeometry geometry;
            active->polygon(geometry);
            Clipper3r::Polygon sh;
            convertPolygon(geometry, sh);
            libnest2d::Item item(sh);
            const auto& contour = item.rawShape().Contour;
            libnest2d::Coord itemArea = Clipper3r::Area(contour);
            int nums = std::floor(binArea / std::fabs(itemArea));
            inputs.reserve(fixed.size() + nums);
            for (int i = 0; i < nums; ++i) {
                inputs.emplace_back(item);
            }
        }
        for (PlacerItem* pitem : fixed) {
            nestplacer::PlacerItemGeometry geometry;
            pitem->polygon(geometry);
            Clipper3r::Polygon sh;
            convertPolygon(geometry, sh);
            libnest2d::Item item(sh);
            item.markAsFixedInBin(0);
            inputs.emplace_back(item);
        }
        libnest2d::Coord itemGap = UM2INT(parameter.itemGap);
        libnest2d::Coord edgeGap = UM2INT(parameter.binItemGap);
        libnest2d::NestControl ctl;
        libnest2d::NestConfig<libnest2d::NfpPlacer, libnest2d::FirstFitSelection> config;
        config.placer_config.starting_point = libnest2d::NfpPlacer::Config::Alignment::CENTER;
        config.placer_config.binItemGap = edgeGap;
        if (parameter.needAlign) {
            config.placer_config.setAlignment(parameter.align_mode);
        } else {
            config.placer_config.alignment = libnest2d::NfpPlacer::Config::Alignment::DONT_ALIGN;
        }

        auto box_func = [&binBox](const int& index) {
            libnest2d::Box box;
            if (index == 0) box = convertBox(binBox);
            return box;
        };
        config.placer_config.box_function = box_func;

        config.placer_config.setNewAlignment(1);
        if (parameter.rotate) {
            int step = (int)(360.0f / parameter.rotateAngle);
            config.placer_config.rotations.clear();
            for (int i = 0; i < step; ++i)
                config.placer_config.rotations.emplace_back(
                    libnest2d::Radians(libnest2d::Degrees((double)i * parameter.rotateAngle)));
        }

        size_t bins = nest(inputs, box_func(0), itemGap, config, ctl);

        //return transformed items.
        std::vector<PlacerResultRT> activeRts;
        activeRts.reserve(inputs.size());
        for (size_t i = 0; i < inputs.size(); ++i) {
            const libnest2d::Item& item = inputs.at(i);
            if (item.binId() < 0) continue;
            PlacerResultRT pr;
            pr.rt.x = INT2UM(item.translation().X);
            pr.rt.y = INT2UM(item.translation().Y);
            pr.rt.z = item.rotation().toDegrees();
            pr.binIndex = item.binId();
            if (item.isFixed()) results.emplace_back(pr);
            else activeRts.emplace_back(pr);
        }
        results.insert(results.end(), activeRts.begin(), activeRts.end());
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