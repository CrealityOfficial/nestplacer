#include "nestplacer/concaveplacer.h"
#include "data.h"
#include "conv.h"
#include "debug.h"

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

    void initConfig(NfpFisrtFitConfig& config, const NestConcaveParam& param)
    {
        config.placer_config.starting_point = libnest2d::NfpPlacer::Config::Alignment::CENTER;
        config.placer_config.alignment = libnest2d::NfpPlacer::Config::Alignment::CENTER;

        //config.placer_config.object_function = [](const libnest2d::Item& item)  //优化方向
        //{
        //    return 3;
        //};
    }

    void initControl(libnest2d::NestControl& control, size_t size, ccglobal::Tracer* tracer)
    {
        control.progressfn = [&size, tracer](int remain) {
            if (tracer)
            {
                tracer->progress((float)((int)size - remain) / (float)size);
            }
        };
        control.stopcond = [tracer]()->bool {
            if (tracer)
                return tracer->interrupt();
            return false;
        };
    }

	void layout_all_nest(const ConcaveItems& models, const NestConcaveParam& param, ConcaveNestDebugger* debugger)
	{
        size_t size = models.size();
        if (size == 0)
            return;

        libnest2d::NestControl ctl;
        initControl(ctl, size, param.tracer);

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

        libnest2d::Box binBox = convert(param.box);

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

    class NestPlacerImpl
    {
    public:
        NestPlacerImpl()
            : size(0)
        {

        }

        ~NestPlacerImpl()
        {

        }

        void setInputs(const ConcaveItems& models) 
        {
            inputs.clear();
            size = (int)models.size();

            for (int i = 0; i < size; i++)
            {
                Clipper3r::Path path;
                convert(models.at(i), path);

                double a = Clipper3r::Area(path);
                if (Clipper3r::Orientation(path))
                    Clipper3r::ReversePath(path);

                inputs.emplace_back(libnest2d::Item(std::move(path)));
                inputs.back().convexCal(false);
            }
        }

        bool valid() 
        {
            return size > 0;
        }

        void invokeCallback(NestCallback callback){

            if (callback)
            {
                for (size_t i = 0; i < size; ++i)
                {
                    const libnest2d::Item& item = inputs.at(i);
                    NestRT rt;
                    rt.x = INT2MM(item.translation().X);
                    rt.y = INT2MM(item.translation().Y);
                    rt.z = item.rotation().toDegrees();
                    callback((int)i, rt);
                }
            }
        }

        std::vector<libnest2d::Item> inputs;
        int size;
    };

    NestPlacer::NestPlacer()
        : impl(new NestPlacerImpl())
    {
    }

    NestPlacer::~NestPlacer()
    {

    }

    void NestPlacer::setInput(const ConcaveItems& models)
    {
        impl->setInputs(models);
    }

    void NestPlacer::layout(const NestConcaveParam& param, ConcaveNestDebugger* _debugger)
    {
        libnest2d::NestControl ctl;
        initControl(ctl, impl->size, param.tracer);

        NfpFisrtFitConfig config;
        initConfig(config, param);
        initDebugger(config, _debugger);

        Clipper3r::cInt distance = MM2INT(param.distance);
        libnest2d::Box binBox = convert(param.box);

        std::size_t result = libnest2d::nest(impl->inputs, binBox, distance, config, ctl);

        impl->invokeCallback(param.callback);
    }

    void NestPlacer::testNFP(trimesh::vec3& point, DebugPolygon& poly, std::vector<trimesh::vec3>& lines)
    {
        using namespace libnest2d;
        if (impl->inputs.size() != 2)
            return;

        const Item& fixedItem = impl->inputs.at(0);
        const Item& orbItem = impl->inputs.at(1);

        auto& fixedp = fixedItem.transformedShape();
        auto& orbp = orbItem.transformedShape();

        struct NfpDebugger {
            void onEdges(const std::vector<_Segment<PointImpl>>& edges) {
                for (const _Segment<PointImpl>& segment : edges)
                {
                    lines.push_back(convert(segment.first()));
                    lines.push_back(convert(segment.second()));
                }
            }

            std::vector<trimesh::vec3> lines;
        } nfpDebugger;

        auto subnfp_r = nfp::nfpConvexOnly<PolygonImpl, double, NfpDebugger>(fixedp, orbp, &nfpDebugger);
        placers::correctNfpPosition(subnfp_r, fixedItem, orbItem);

        point = convert(subnfp_r.second);
        convertPolygon(subnfp_r.first, poly);
        lines = nfpDebugger.lines;
    }
}
