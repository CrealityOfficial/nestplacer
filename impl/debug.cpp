#include "debug.h"
#include "conv.h"

namespace nestplacer
{
    void initDebugger(NfpFisrtFitConfig& config, ConcaveNestDebugger* debugger)
    {
        if (!debugger)
            return;

        debugger->rotates.clear();
        debugger->rotates.reserve(10);

        using namespace libnest2d;
        NfpPlacer::Config& nfpConfig = config.placer_config;

#if _DEBUG
        nfpConfig.parallel = false;
#endif
        nfpConfig.debug_items = [debugger](const nfp::Shapes<PolygonImpl>& items,
            const nfp::Shapes<PolygonImpl>& merged_piles,
            const nfp::Shapes<PolygonImpl>& nfps) {
            RotateDebugger rotate;
            convertPolygons(items, rotate.items);
            convertPolygons(merged_piles, rotate.merged_piles);
            convertPolygons(nfps, rotate.nfps);

            debugger->rotates.emplace_back(std::move(rotate));
        };
    }
}
