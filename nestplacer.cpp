#include"nestplacer/nestplacer.h"

#include <iostream>
#include <libnest2d/libnest2d.hpp>

const double BP2D_CONSTEXPR Pi = 3.141592653589793238463;

namespace nestplacer
{

	NestPlacer::NestPlacer()
	{

	}
	NestPlacer::~NestPlacer()
	{

	}

    void InitCfg(libnest2d::NestConfig<libnest2d::NfpPlacer, libnest2d::FirstFitSelection>& cfg, PlaceType type, bool parallel)
    {
        cfg.placer_config.rotations.push_back(libnest2d::Radians(Pi / 4.0));//多边形可用旋转角
        cfg.placer_config.rotations.push_back(libnest2d::Radians(Pi * 3 / 4.0));
        cfg.placer_config.rotations.push_back(libnest2d::Radians(Pi * 5 / 4.0));
        cfg.placer_config.rotations.push_back(libnest2d::Radians(Pi * 7 / 4.0));
        cfg.placer_config.parallel = parallel;  //启用单线程
        cfg.placer_config.starting_point = libnest2d::NfpPlacer::Config::Alignment::CENTER;
        //cfg.placer_config.accuracy; //优化率 
        //cfg.placer_config.explore_holes;  //孔内是否放置图元,目前源码中还未实现
        //cfg.placer_config.before_packing; //摆放下一个Item之前，先对前面已经摆放好的Item进行调整的函数，若为空，则Item拍好位置后将不再变动。传入函数接口
        //cfg.placer_config.object_function;  //添加优化方向，向函数输出值最小化优化，以此改变排放方式，传入函数接口

        //cfg.selector_config;

        if (type != PlaceType::NULLTYPE)
        {
            cfg.placer_config.object_function = [type](const libnest2d::Item&)  //优化方向
            {
                switch (type)
                {
                case PlaceType::CENTER_TO_SIDE: return 3; break;
                case PlaceType::ALIGNMENT: return 4; break;
                case PlaceType::ONELINE: return 5; break;
                }
            };
        }
    }

    TransMatrix getTransMatrixFromItem(libnest2d::Item iitem, Clipper3r::cInt offsetX, Clipper3r::cInt offsetY)
    {
        TransMatrix itemTrans;
        itemTrans.rotation = iitem.rotation().toDegrees();
        itemTrans.x = iitem.translation().X + offsetX;
        itemTrans.y = iitem.translation().Y + offsetY;
        return itemTrans;
    }

    libnest2d::Item getItem(NestItemer* itemer)
    {
        Clipper3r::Path newItemPath;
        newItemPath = libnest2d::shapelike::convexHull(itemer->path());
        if (Clipper3r::Orientation(newItemPath))
        {
            Clipper3r::ReversePath(newItemPath);
        }
        libnest2d::Item item = libnest2d::Item(newItemPath);
        return item;
    }
    
    Clipper3r::Path getItemPath(NestItemer* itemer)
    {
        Clipper3r::Path newItemPath;
        newItemPath = libnest2d::shapelike::convexHull(itemer->path());
        if (Clipper3r::Orientation(newItemPath))
        {
            Clipper3r::ReversePath(newItemPath);
        }
        return newItemPath;
    }

    Clipper3r::Path ItemPathDataTrans(std::vector<trimesh::vec3> model, bool pretreatment)
    {
        Clipper3r::Path result;
        for (trimesh::vec3 pt : model)
        {
            Clipper3r::IntPoint pt_t;
            pt_t.X = pt.x * NEST_FACTOR;
            pt_t.Y = pt.y * NEST_FACTOR;
            result.push_back(pt_t);
        }
        if (pretreatment)
        {
            if (Clipper3r::Orientation(result))
            {
                Clipper3r::ReversePath(result);
            }
            result = libnest2d::shapelike::convexHull(result);
        }
        return result;
    }

    int bOnTheEdge(Clipper3r::Path item_path, int width, int height)
    {
        Clipper3r::Path rect = {
            {0, 0},
            {width, 0},
            {width, height},
            {0, height}
        };
        Clipper3r::Paths result;
        Clipper3r::Clipper a;
        a.AddPath(rect, Clipper3r::ptSubject, true);
        a.AddPath(item_path, Clipper3r::ptClip, true);
        a.Execute(Clipper3r::ctIntersection, result, Clipper3r::pftNonZero, Clipper3r::pftNonZero);
        if (result.empty())
            return 0;
        double inter_area = fabs(Clipper3r::Area(result[0]));
        double diff_area = fabs(inter_area - fabs(Clipper3r::Area(item_path)));
        if (inter_area > 0 && diff_area > 1000) return 1;
        if (diff_area < 1000) return 2;
        return 0;
    }

    bool NestPlacer::nest2d_base(Clipper3r::Paths ItemsPaths, NestParaCInt para, std::vector<TransMatrix>& transData)
    {
        PlaceType tpye = para.packType;
        size_t size = ItemsPaths.size();
        transData.resize(size);

        libnest2d::NestControl ctl;
        libnest2d::NestConfig<libnest2d::NfpPlacer, libnest2d::FirstFitSelection> cfg;
        InitCfg(cfg, tpye, para.parallel);
        cfg.placer_config.alignment = libnest2d::NfpPlacer::Config::Alignment::CENTER;

        auto convert = [&ItemsPaths, tpye](Clipper3r::Path& oItem, int index) {
            Clipper3r::Path lines = ItemsPaths.at(index);
            Clipper3r::Clipper a;
            a.AddPath(lines, Clipper3r::ptSubject, true);
            Clipper3r::IntRect r = a.GetBounds();
            oItem.resize(4);
            oItem[0] = Clipper3r::IntPoint(r.left, r.bottom);
            oItem[1] = Clipper3r::IntPoint(r.right, r.bottom);
            oItem[2] = Clipper3r::IntPoint(r.right, r.top);
            oItem[3] = Clipper3r::IntPoint(r.left, r.top);
        };

        std::vector<libnest2d::Item> input;
        for (int i = 0; i < size; i++)
        {
            Clipper3r::Path ItemPath;
            if (tpye == PlaceType::ALIGNMENT) convert(ItemPath, i);
            else
            {
                ItemPath = ItemsPaths[i];
            }
                
            if (Clipper3r::Orientation(ItemPath))
            {
                Clipper3r::ReversePath(ItemPath);//正轮廓倒序
            }
            ItemPath = libnest2d::shapelike::convexHull(ItemPath);//仅支持凸轮廓排样
            input.push_back(libnest2d::Item(ItemPath));
        }

        Clipper3r::cInt imgW_dst = para.workspaceW, imgH_dst = para.workspaceH;

        double offsetX = 0., offsetY = 0.;

        int egde_dist = 100;//排样到边缘最近距离为50单位
        if (input.size() == 1) egde_dist = 0;
        if (egde_dist > para.modelsDist) egde_dist = para.modelsDist;
        imgW_dst += para.modelsDist - egde_dist;
        imgH_dst += para.modelsDist - egde_dist;
        offsetX += (para.modelsDist - egde_dist) / 2;
        offsetY += (para.modelsDist - egde_dist) / 2;
        if (para.modelsDist == 0) para.modelsDist = 1;
        libnest2d::Box maxBox = libnest2d::Box(imgW_dst, imgH_dst, { imgW_dst / 2, imgH_dst / 2 });
        std::size_t result = libnest2d::nest(input, maxBox, para.modelsDist, cfg, ctl);//只能处理凸包


        Clipper3r::Clipper a;
        for (int i = 0; i < size; i++)
        {
            double angle = input.at(i).rotation().toDegrees();
            if (angle != -90.)//区域内模型
            {
                auto trans_item = input.at(i).transformedShape_s();
                a.AddPath(trans_item.Contour, Clipper3r::ptSubject, true);
            }
        }
        Clipper3r::IntRect ibb_dst = a.GetBounds();
        int center_offset_x = 0.5 * para.workspaceW - (0.5 * (ibb_dst.right + ibb_dst.left) + offsetX);
        int center_offset_y = 0.5 * para.workspaceH - (0.5 * (ibb_dst.bottom + ibb_dst.top) + offsetY);
        if (tpye == PlaceType::ONELINE) center_offset_y = -ibb_dst.top - offsetY;

        std::vector<libnest2d::Item> input_outer_items(1,
            {
                {0, para.workspaceH},
                {para.workspaceW, para.workspaceH},
                {para.workspaceW, 0},
                {0, 0},
                {0, para.workspaceH}
            });//定义第一个轮廓遮挡中心排样区域

        input_outer_items[0].translation({ 1, 0 }); //packed mark

        for (int i = 0; i < size; i++)
        {
            libnest2d::Item& iitem = input.at(i);

            int model_offset_x = iitem.translation().X;
            int model_offset_y = iitem.translation().Y;
            double angle = iitem.rotation().toDegrees();

            if (angle == -90.)
            {
                input_outer_items.push_back(libnest2d::Item(iitem.rawShape()));//收集排样区域外模型
            }
            else
            {
                model_offset_x += center_offset_x;
                model_offset_y += center_offset_y;
                TransMatrix itemTrans;
                itemTrans.rotation = input[i].rotation().toDegrees();
                itemTrans.x = model_offset_x + offsetX;
                itemTrans.y = model_offset_y + offsetY;
                transData[i] = itemTrans;
            }
        }

          //////区域外模型排样
        if (input_outer_items.size() == 1) return true;
        cfg.placer_config.object_function = NULL;
        cfg.placer_config.starting_point = libnest2d::NfpPlacer::Config::Alignment::CENTER;
        cfg.placer_config.alignment = libnest2d::NfpPlacer::Config::Alignment::DONT_ALIGN;
        imgW_dst = para.workspaceW * 1001; imgH_dst = para.workspaceH * 1001;
        maxBox = libnest2d::Box(imgW_dst, imgH_dst, { para.workspaceW / 2, para.workspaceH / 2 });
        std::size_t result_dst = libnest2d::nest(input_outer_items, maxBox, para.modelsDist, cfg, ctl);
        int idx = 1;
        for (int i = 0; i < size; i++)
        {
            libnest2d::Item& iitem = input.at(i);
            double angle = iitem.rotation().toDegrees();
            if (angle == -90.)
            {
                libnest2d::Item& iitem_dst = input_outer_items.at(idx);
                TransMatrix itemTrans;
                itemTrans.rotation = iitem_dst.rotation().toDegrees();
                itemTrans.x = iitem_dst.translation().X;
                itemTrans.y = iitem_dst.translation().Y;
                transData[i] = itemTrans;
                idx++;
            }
        }

        return result;
    }

    void NestPlacer::layout_all_nest(std::vector < std::vector<trimesh::vec3>> models, std::vector<int> modelIndices,
        NestParaFloat para, std::function<void(int, trimesh::vec3)> modelPositionUpdateFunc)
    {
        trimesh::box3 basebox = para.workspaceBox;
        Clipper3r::Paths allItem;
        allItem.reserve(models.size());
        for (std::vector<trimesh::vec3> m : models)
        {
            Clipper3r::Path oItem;
            int m_size = m.size();
            oItem.resize(m_size);
            for (int i = 0; i < m_size; i++)
            {
                oItem.at(i).X = (Clipper3r::cInt)(m.at(i).x * NEST_FACTOR);
                oItem.at(i).Y = (Clipper3r::cInt)(m.at(i).y * NEST_FACTOR);
            }
            allItem.push_back(oItem);
        }

        Clipper3r::cInt _imageW = (basebox.max.x - basebox.min.x) * NEST_FACTOR;
        Clipper3r::cInt _imageH = (basebox.max.y - basebox.min.y) * NEST_FACTOR;
        Clipper3r::cInt _dist = para.modelsDist * NEST_FACTOR;
        std::vector<TransMatrix> transData;
        NestParaCInt para_cInt = NestParaCInt(_imageW, _imageH, _dist, para.packType, para.parallel);
        nest2d_base(allItem, para_cInt, transData);

        /////settle models that can be settled inside
        trimesh::vec3 total_offset;
        for (size_t i = 0; i < modelIndices.size(); i++)
        {
            trimesh::vec3 newBoxCenter;
            newBoxCenter.x = transData.at(i).x / NEST_FACTOR;
            newBoxCenter.y = transData.at(i).y / NEST_FACTOR;
            newBoxCenter.z = transData.at(i).rotation;
            int modelIndexI = modelIndices[i];
            modelPositionUpdateFunc(modelIndexI, newBoxCenter);
        }

    }

    bool NestPlacer::nest2d(std::vector<NestItemer*>& items, NestParaCInt para, PlaceFunc func)
    {
        Clipper3r::Paths ItemsPaths;
        for (NestItemer* itemer : items)
            ItemsPaths.emplace_back(getItemPath(itemer));
        std::vector<TransMatrix> transData;
        bool nestResult = nest2d_base(ItemsPaths, para, transData);

        if (func && nestResult)
        {
            size_t size = transData.size();
            for (size_t i = 0; i < size; i++)
                func((int)i, transData.at(i));
        }
        return nestResult;
    }

    bool NestPlacer::nest2d_base(Clipper3r::Paths ItemsPaths, Clipper3r::Path transData, Clipper3r::Path newItemPath, NestParaCInt para, TransMatrix& NewItemTransData)
    {
        Clipper3r::cInt offsetX = 0, offsetY = 0;
        int egde_dist = 100;
        if (egde_dist > para.modelsDist) egde_dist = para.modelsDist;
        offsetX += (para.modelsDist - egde_dist) / 2;
        offsetY += (para.modelsDist - egde_dist) / 2;

        libnest2d::NestConfig<libnest2d::NfpPlacer, libnest2d::FirstFitSelection> cfg;
        cfg.placer_config.alignment = libnest2d::NfpPlacer::Config::Alignment::DONT_ALIGN;
        InitCfg(cfg, PlaceType::NULLTYPE, para.parallel);
        std::vector<libnest2d::Item> input;
        std::vector<libnest2d::Item> input_out_pack(1,
            {
                {0, para.workspaceH},
                {para.workspaceW, para.workspaceH},
                {para.workspaceW, 0},
                {0, 0},
                {0, para.workspaceH}
            });//定义第一个轮廓遮挡中心排样区域
        input_out_pack[0].translation({ 1, 0 }); //packed mark
        for (int i = 0; i < ItemsPaths.size(); i++)
        {
            Clipper3r::Path ItemPath = ItemsPaths[i];
            libnest2d::Item item = libnest2d::Item(ItemPath);
            Clipper3r::IntPoint trans_data = transData[i];
            item.translation({ trans_data.X, trans_data.Y });
            auto trans_item = item.transformedShape_s();
            if (bOnTheEdge(trans_item.Contour, para.workspaceW, para.workspaceH) == 2)
            {
                input.push_back(item);
            }
            else
            {
                input_out_pack.push_back(item);
            }
        }

        libnest2d::Item newItem = libnest2d::Item(newItemPath);

        bool can_pack = false;
        Clipper3r::cInt imgW_dst = para.workspaceW, imgH_dst = para.workspaceH;
        libnest2d::Box maxBox = libnest2d::Box(imgW_dst + 2 * offsetX, imgH_dst + 2 * offsetY, { para.workspaceW / 2, para.workspaceH / 2 });
        newItem.translation({ para.workspaceW / 2, para.workspaceH / 2 });
        for (auto rot : cfg.placer_config.rotations) {
            newItem.rotation(rot);
            auto trans_item = newItem.transformedShape_s();
            if (bOnTheEdge(trans_item.Contour, para.workspaceW, para.workspaceH) == 2)
            {
                can_pack = true;
                break;
            }
        }
        if (!can_pack)
        {
            newItem.translation({ 0, 0 });
            newItem.rotation(0);
            input_out_pack.push_back(newItem);
        }
        else
        {
            newItem.translation({ 0, 0 });
            input.push_back(newItem);
            if (para.modelsDist == 0) para.modelsDist = 1;
            std::size_t result = libnest2d::nest(input, maxBox, para.modelsDist, cfg, libnest2d::NestControl());

            int total_size = input.size();
            libnest2d::Item iitem = input.at(total_size - 1);
            if (iitem.rotation().toDegrees() == -90.)
            {
                iitem.rotation(0);
                iitem.translation({ 0, 0 });
                input_out_pack.push_back(iitem);
            }
            else
            {
                NewItemTransData.x = iitem.translation().X;
                NewItemTransData.y = iitem.translation().Y;
                NewItemTransData.rotation = iitem.rotation().toDegrees();
                return true;
            }
        }

        cfg.placer_config.object_function = NULL;
        cfg.placer_config.starting_point = libnest2d::NfpPlacer::Config::Alignment::CENTER;
        imgW_dst = para.workspaceW * 1001; imgH_dst = para.workspaceH * 1001;
        maxBox = libnest2d::Box(imgW_dst, imgH_dst, { para.workspaceW / 2, para.workspaceH / 2 });
        libnest2d::nest(input_out_pack, maxBox, para.modelsDist, cfg, libnest2d::NestControl());
        libnest2d::Item oitem = input_out_pack.at(input_out_pack.size() - 1);
        NewItemTransData.x = oitem.translation().X;
        NewItemTransData.y = oitem.translation().Y;
        NewItemTransData.rotation = oitem.rotation().toDegrees();
        return false;

    }

    bool NestPlacer::layout_new_item(std::vector < std::vector<trimesh::vec3>> models, std::vector<trimesh::vec3> transData,
        std::vector<trimesh::vec3> NewItem, NestParaFloat para, std::function<void(trimesh::vec3)> func)
    {
        trimesh::box3 basebox = para.workspaceBox;
        Clipper3r::cInt w = (basebox.max.x - basebox.min.x) * NEST_FACTOR;
        Clipper3r::cInt h = (basebox.max.y - basebox.min.y) * NEST_FACTOR;
        Clipper3r::cInt d = para.modelsDist * NEST_FACTOR;
        NestParaCInt para_cInt = NestParaCInt(w, h, d, PlaceType::NULLTYPE, para.parallel);
        Clipper3r::Paths ItemsPaths;
        for (int i = 0; i < models.size(); i++)
        {
            std::vector<trimesh::vec3> model = models[i];
            ItemsPaths.push_back(ItemPathDataTrans(model, true));
        }
        Clipper3r::Path transData_cInt = ItemPathDataTrans(transData, false);
        Clipper3r::Path newItemPath = ItemPathDataTrans(NewItem, true);
        TransMatrix NewItemTransData;
        bool can_pack = nest2d_base(ItemsPaths, transData_cInt, newItemPath, para_cInt, NewItemTransData);

        trimesh::vec3 tmp;
        tmp.x = NewItemTransData.x / NEST_FACTOR;
        tmp.y = NewItemTransData.y / NEST_FACTOR;
        tmp.z = NewItemTransData.rotation;
        func(tmp);
        return can_pack;
    }

    bool NestPlacer::nest2d(std::vector<NestItemer*>& items, NestItemer* item, NestParaCInt para, PlaceOneFunc func)
    {
        Clipper3r::Paths ItemsPaths;
        Clipper3r::Path transData_cInt;
        for (NestItemer* itemer : items)
        {
            ItemsPaths.emplace_back(getItemPath(itemer));
            transData_cInt.emplace_back(itemer->translate());
        }
            
        Clipper3r::Path newItemPath = getItemPath(item);
        TransMatrix NewItemTransData;
        bool can_pack = nest2d_base(ItemsPaths, transData_cInt, newItemPath, para, NewItemTransData);

        func(NewItemTransData);
        return can_pack;
    }


}