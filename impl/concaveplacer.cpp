#include "nestplacer/concaveplacer.h"
#include "data.h"
#include "conv.h"

#include <libnest2d/libnest2d.hpp>

namespace nestplacer
{
    void convert(const ConcaveItem& _items, Clipper3r::Path& path)
    {
        size_t size = _items.size();
        if (size > 0)
        {
            path.resize(size);
            for (size_t i = 0; i < size; ++i)
                path.at(i) = convert(_items.at(i));
        }
    }

    typedef libnest2d::NestConfig<libnest2d::NfpPlacer, libnest2d::FirstFitSelection> NfpFisrtFitConfig;

    void initConfig(NfpFisrtFitConfig& config, const NestConcaveParam& param)
    {
        config.placer_config.starting_point = libnest2d::NfpPlacer::Config::Alignment::CENTER;
        config.placer_config.alignment = libnest2d::NfpPlacer::Config::Alignment::CENTER;

        //config.placer_config.object_function = [](const libnest2d::Item& item)  //优化方向
        //{
        //    return 3;
        //};
    }

    void initDebugger(NfpFisrtFitConfig& config, ConcaveNestDebugger* debugger)
    {
        if (!debugger)
            return;

        using namespace libnest2d;
        NfpPlacer::Config& nfpConfig = config.placer_config;
        nfpConfig.debug_nfps = [debugger](const nfp::Shapes<PolygonImpl>& nfps) {
            size_t size = nfps.size();
            if (size > 0)
            {
                std::vector<DebugPolygon> oNfps(size);
                for (size_t i = 0; i < size; ++i)
                    convertPolygon(nfps.at(i), oNfps.at(i));
                debugger->onNPFs(oNfps);
            }
        };
    }

	void layout_all_nest(const ConcaveItems& models, const NestConcaveParam& param, ConcaveNestDebugger* debugger)
	{
        size_t size = models.size();
        if (size == 0)
            return;

        libnest2d::NestControl ctl;
        ctl.progressfn = [&size, &param](int remain) {
            if (param.tracer)
            {
                param.tracer->progress((float)((int)size - remain) / (float)size);
            }
        };
        ctl.stopcond = [&param]()->bool {
            if (param.tracer)
                return param.tracer->interrupt();
            return false;
        };

        NfpFisrtFitConfig config;
        initConfig(config, param);
        initDebugger(config, debugger);

        Clipper3r::cInt distance = MM2INT(param.distance);

        std::vector<libnest2d::Item> inputs;
        for (int i = 0; i < size; i++)
        {
            Clipper3r::Path path;
            convert(models.at(i), path);
            inputs.emplace_back(libnest2d::Item(std::move(path)));
            inputs.back().convexCal(false);
        }

        Clipper3r::IntPoint minPoint(convert(param.box.min));
        Clipper3r::IntPoint maxPoint(convert(param.box.max));
        Clipper3r::IntPoint rect = maxPoint - minPoint;

        libnest2d::Box binBox = libnest2d::Box(rect.X, rect.Y, convert(param.box.center()));

        std::size_t result = libnest2d::nest(inputs, binBox, distance, config, ctl);

        for (size_t i = 0; i < size; ++i)
        {
            const libnest2d::Item& item = inputs.at(i);
            NestRT rt;
            rt.x = INT2MM(item.translation().X);
            rt.y = INT2MM(item.translation().Y);
            rt.z = item.rotation().toDegrees();
            param.callback((int)i, rt);
        }
	}
}
