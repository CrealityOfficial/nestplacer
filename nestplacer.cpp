#include"nestplacer/nestplacer.h"

#include <iostream>
#include <libnest2d/libnest2d.hpp>

const double BP2D_CONSTEXPR Pi = 3.141592653589793238463;
const double FACTOR = 10000.0;

namespace nestplacer
{

	NestPlacer::NestPlacer()
	{

	}
	NestPlacer::~NestPlacer()
	{

	}

    bool NestPlacer::nest2d(Clipper3r::Paths ItemsPaths, int _imageW, int _imageH, int _dist, PlaceType placeType, std::vector<TransMatrix>& transData)
    {
        if (ItemsPaths.size() == 2)
            placeType = PlaceType::CENTER_TO_SIDE;

        size_t size = ItemsPaths.size();
        transData.resize(size);


        libnest2d::NestConfig<libnest2d::NfpPlacer, libnest2d::FirstFitSelection> cfg;

        cfg.placer_config.alignment = libnest2d::NfpPlacer::Config::Alignment::CENTER;
        switch (placeType)
        {
        case PlaceType::CENTER_TO_SIDE: cfg.placer_config.starting_point = libnest2d::NfpPlacer::Config::Alignment::CENTER; break;
        case PlaceType::MID_TO_UP_DOWN: cfg.placer_config.starting_point = libnest2d::NfpPlacer::Config::Alignment::CENTER; break;
            //从Y中轴线向上下两方向排样，starting_point = CENTER，alignment = DONT_ALIGN或alignment = CENTER
        case PlaceType::MID_TO_LEFT_RIGHT: cfg.placer_config.starting_point = libnest2d::NfpPlacer::Config::Alignment::CENTER; break;
            //从X中轴线向左右两方向排样，starting_point = CENTER，alignment = DONT_ALIGN或alignment = CENTER
        case PlaceType::LEFT_TO_RIGHT: cfg.placer_config.starting_point = libnest2d::NfpPlacer::Config::Alignment::BOTTOM_LEFT; break;
            //从X轴0向右方向排样，starting_point = BOTTOM_LEFT或starting_point = TOP_LEFT，alignment = DONT_ALIGN
        case PlaceType::RIGHT_TO_LEFT: cfg.placer_config.starting_point = libnest2d::NfpPlacer::Config::Alignment::BOTTOM_RIGHT; break;
            //从X轴max向左方向排样，starting_point = BOTTOM_RIGHT或starting_point = TOP_RIGHT，alignment = DONT_ALIGN
        case PlaceType::UP_TO_DOWN: cfg.placer_config.starting_point = libnest2d::NfpPlacer::Config::Alignment::TOP_LEFT; break;
            //从Y轴0向下方向排样，starting_point = BOTTOM_LEFT或starting_point = BOTTOM_RIGHT，alignment = DONT_ALIGN
        case PlaceType::DOWN_TO_UP: cfg.placer_config.starting_point = libnest2d::NfpPlacer::Config::Alignment::BOTTOM_LEFT; break;
            //从Y轴max向上方向排样，starting_point = TOP_LEFT或starting_point = TOP_RIGHT，alignment = DONT_ALIGN
        }

        cfg.placer_config.rotations.push_back(libnest2d::Radians(Pi / 4.0));//多边形可用旋转角
        cfg.placer_config.rotations.push_back(libnest2d::Radians(Pi * 3 / 4.0));
        cfg.placer_config.rotations.push_back(libnest2d::Radians(Pi * 5 / 4.0));
        cfg.placer_config.rotations.push_back(libnest2d::Radians(Pi * 7 / 4.0));
        //cfg.placer_config.accuracy; //优化率 
        //cfg.placer_config.explore_holes;  //孔内是否放置图元,目前源码中还未实现
        //cfg.placer_config.parallel;  //启用多线程
        //cfg.placer_config.before_packing; //摆放下一个Item之前，先对前面已经摆放好的Item进行调整的函数，若为空，则Item拍好位置后将不再变动。传入函数接口
        //cfg.placer_config.object_function;  //添加优化方向，向函数输出值最小化优化，以此改变排放方式，传入函数接口

        //cfg.selector_config;

        libnest2d::NestControl ctl;

        cfg.placer_config.object_function = [placeType](const libnest2d::Item&)  //优化方向
        {
            switch (placeType)
            {
            case PlaceType::CENTER_TO_SIDE: return 3; break;
            case PlaceType::MID_TO_UP_DOWN: return 4; break;
            case PlaceType::MID_TO_LEFT_RIGHT: return 5; break;
            case PlaceType::LEFT_TO_RIGHT: return 6; break;
            case PlaceType::RIGHT_TO_LEFT: return 7; break;
            case PlaceType::DOWN_TO_UP: return 8; break;
            case PlaceType::UP_TO_DOWN: return 9; break;
            }
        };

        auto convert = [&ItemsPaths](Clipper3r::Path& oItem, int index) {
            Clipper3r::Path lines = ItemsPaths.at(index);
            size_t size = lines.size();
            if (size > 0)
            {
                oItem.resize(size);
                for (size_t i = 0; i < size; ++i)
                {
                    oItem.at(i).X = (Clipper3r::cInt)(lines.at(i).X);
                    oItem.at(i).Y = (Clipper3r::cInt)(lines.at(i).Y);
                }
            }
        };

        std::vector<libnest2d::Item> input;
        for (int i = 0; i < size; i++)
        {
            Clipper3r::Path ItemPath;
            convert(ItemPath, i);

            if (Clipper3r::Orientation(ItemPath))
            {
                Clipper3r::ReversePath(ItemPath);//正轮廓倒序
            }
            ItemPath = libnest2d::shapelike::convexHull(ItemPath);//仅支持凸轮廓排样
            input.push_back(libnest2d::Item(ItemPath));
        }

        Clipper3r::cInt imgW = _imageW;   //排样区域宽
        Clipper3r::cInt imgH = _imageH;   //排样区域高
        Clipper3r::cInt dist = _dist;     //排样间距

        Clipper3r::cInt imgW_dst = imgW, imgH_dst = imgH;

        double offsetX = 0., offsetY = 0.;

        int egde_dist = 100;//排样到边缘最近距离为50单位
        if (input.size() == 1) egde_dist = 0;
        if (egde_dist > dist) egde_dist = dist;
        imgW_dst += dist - egde_dist;
        imgH_dst += dist - egde_dist;
        offsetX += (dist - egde_dist) / 2;
        offsetY += (dist - egde_dist) / 2;

        libnest2d::Box maxBox = libnest2d::Box(imgW_dst, imgH_dst, { imgW_dst / 2, imgH_dst / 2 });
        std::size_t result = libnest2d::nest(input, maxBox, dist, cfg, ctl);//只能处理凸包


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
        int center_offset_x = 0.5 * imgW - (0.5 * (ibb_dst.right + ibb_dst.left) + offsetX);
        int center_offset_y = 0.5 * imgH - (0.5 * (ibb_dst.bottom + ibb_dst.top) + offsetY);

        std::vector<libnest2d::Item> input_outer_items(1,
            {
                {0, imgH},
                {imgW, imgH},
                {imgW, 0},
                {0, 0},
                {0, imgH}
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
        imgW_dst = imgW * 1001; imgH_dst = imgH * 1001;
        maxBox = libnest2d::Box(imgW_dst, imgH_dst, { imgW / 2, imgH / 2 });
        std::size_t result_dst = libnest2d::nest(input_outer_items, maxBox, dist, cfg, ctl);
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

        return result == size;
    }


    void NestPlacer::layout_all_nest(trimesh::box3 workspaceBox, std::vector<int> modelIndices,
        std::vector < std::vector<trimesh::vec3>> models, std::function<void(int, trimesh::vec3)> modelPositionUpdateFunc)
    {
        double scaleFactor = 10000.0;
        trimesh::box3 basebox = workspaceBox;
        double imageW = basebox.max.x - basebox.min.x;
        double imageH = basebox.max.y - basebox.min.y;
        double dist = 10.0f;

        Clipper3r::Paths allItem;
        allItem.reserve(models.size());
        for (std::vector<trimesh::vec3> m : models)
        {
            Clipper3r::Path oItem;
            int m_size = m.size();
            oItem.resize(m_size);
            for (int i = 0; i < m_size; i++)
            {
                oItem.at(i).X = (Clipper3r::cInt)(m.at(i).x * scaleFactor);
                oItem.at(i).Y = (Clipper3r::cInt)(m.at(i).y * scaleFactor);
            }
            allItem.push_back(oItem);
        }

        int _imageW = (basebox.max.x - basebox.min.x) * scaleFactor;
        int _imageH = (basebox.max.y - basebox.min.y) * scaleFactor;
        int _dist = dist * scaleFactor;

        std::vector<TransMatrix> transData;
        PlaceType packType = PlaceType::CENTER_TO_SIDE;
        nest2d(allItem, _imageW, _imageH, _dist, packType, transData);

        /////settle models that can be settled inside
        trimesh::vec3 total_offset;
        for (size_t i = 0; i < modelIndices.size(); i++)
        {
            trimesh::vec3 newBoxCenter;
            newBoxCenter.x = transData.at(i).x / scaleFactor;
            newBoxCenter.y = transData.at(i).y / scaleFactor;
            newBoxCenter.z = transData.at(i).rotation;
            int modelIndexI = modelIndices[i];
            modelPositionUpdateFunc(modelIndexI, newBoxCenter);
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

    void InitCfg(libnest2d::NestConfig<libnest2d::NfpPlacer, libnest2d::FirstFitSelection>& cfg)
    {
        cfg.placer_config.alignment = libnest2d::NfpPlacer::Config::Alignment::DONT_ALIGN;
        cfg.placer_config.rotations.push_back(libnest2d::Radians(Pi / 4.0));//多边形可用旋转角
        cfg.placer_config.rotations.push_back(libnest2d::Radians(Pi * 3 / 4.0));
        cfg.placer_config.rotations.push_back(libnest2d::Radians(Pi * 5 / 4.0));
        cfg.placer_config.rotations.push_back(libnest2d::Radians(Pi * 7 / 4.0));
        //cfg.placer_config.accuracy; //优化率 
        //cfg.placer_config.explore_holes;  //孔内是否放置图元,目前源码中还未实现
        //cfg.placer_config.parallel;  //启用多线程
        //cfg.placer_config.before_packing; //摆放下一个Item之前，先对前面已经摆放好的Item进行调整的函数，若为空，则Item拍好位置后将不再变动。传入函数接口
        //cfg.placer_config.object_function;  //添加优化方向，向函数输出值最小化优化，以此改变排放方式，传入函数接口

        //cfg.selector_config;
    }

    bool NestPlacer::nest2d(std::vector<NestItemer*>& items, Clipper3r::cInt w, Clipper3r::cInt h, Clipper3r::cInt d, PlaceType type, PlaceFunc func)
    {
        std::vector<TransMatrix> transData;
        std::vector<libnest2d::Item> nestItems;
        for (NestItemer* itemer : items)
            nestItems.emplace_back(getItem(itemer));

        bool nestResult = false; 

        if (nestItems.size() == 2)
            type = PlaceType::CENTER_TO_SIDE;

        size_t size = nestItems.size();
        transData.resize(size);


        libnest2d::NestConfig<libnest2d::NfpPlacer, libnest2d::FirstFitSelection> cfg;
        InitCfg(cfg);

        switch (type)
        {
        case PlaceType::CENTER_TO_SIDE: cfg.placer_config.starting_point = libnest2d::NfpPlacer::Config::Alignment::CENTER; break;
        case PlaceType::MID_TO_UP_DOWN: cfg.placer_config.starting_point = libnest2d::NfpPlacer::Config::Alignment::CENTER; break;
            //从Y中轴线向上下两方向排样，starting_point = CENTER，alignment = DONT_ALIGN或alignment = CENTER
        case PlaceType::MID_TO_LEFT_RIGHT: cfg.placer_config.starting_point = libnest2d::NfpPlacer::Config::Alignment::CENTER; break;
            //从X中轴线向左右两方向排样，starting_point = CENTER，alignment = DONT_ALIGN或alignment = CENTER
        case PlaceType::LEFT_TO_RIGHT: cfg.placer_config.starting_point = libnest2d::NfpPlacer::Config::Alignment::BOTTOM_LEFT; break;
            //从X轴0向右方向排样，starting_point = BOTTOM_LEFT或starting_point = TOP_LEFT，alignment = DONT_ALIGN
        case PlaceType::RIGHT_TO_LEFT: cfg.placer_config.starting_point = libnest2d::NfpPlacer::Config::Alignment::BOTTOM_RIGHT; break;
            //从X轴max向左方向排样，starting_point = BOTTOM_RIGHT或starting_point = TOP_RIGHT，alignment = DONT_ALIGN
        case PlaceType::UP_TO_DOWN: cfg.placer_config.starting_point = libnest2d::NfpPlacer::Config::Alignment::TOP_LEFT; break;
            //从Y轴0向下方向排样，starting_point = BOTTOM_LEFT或starting_point = BOTTOM_RIGHT，alignment = DONT_ALIGN
        case PlaceType::DOWN_TO_UP: cfg.placer_config.starting_point = libnest2d::NfpPlacer::Config::Alignment::BOTTOM_LEFT; break;
            //从Y轴max向上方向排样，starting_point = TOP_LEFT或starting_point = TOP_RIGHT，alignment = DONT_ALIGN
        }

        cfg.placer_config.object_function = [type](const libnest2d::Item&)  //优化方向
        {
            switch (type)
            {
            case PlaceType::CENTER_TO_SIDE: return 3; break;
            case PlaceType::MID_TO_UP_DOWN: return 4; break;
            case PlaceType::MID_TO_LEFT_RIGHT: return 5; break;
            case PlaceType::LEFT_TO_RIGHT: return 6; break;
            case PlaceType::RIGHT_TO_LEFT: return 7; break;
            case PlaceType::DOWN_TO_UP: return 8; break;
            case PlaceType::UP_TO_DOWN: return 9; break;
            }
        };

        Clipper3r::cInt imgW_dst = w, imgH_dst = h;

        double offsetX = 0., offsetY = 0.;
        //switch (type)  //排样区域放大适配
        //{
        //case PlaceType::CENTER_TO_SIDE: {imgW_dst = w * 3; imgH_dst = h * 3; offsetX = -1.0 * w; offsetY = -1.0 * h; }; break;
        //case PlaceType::MID_TO_UP_DOWN: {imgH_dst = h * 3;  offsetY = -1.0 * h; }; break;
        //case PlaceType::MID_TO_LEFT_RIGHT: {imgW_dst = w * 3; offsetX = -1.0 * w; }; break;
        //case PlaceType::LEFT_TO_RIGHT: {imgW_dst = w * 3; }; break;
        //case PlaceType::RIGHT_TO_LEFT: {imgW_dst = w * 3; offsetX = -2.0 * w; }; break;
        //case PlaceType::UP_TO_DOWN: {imgH_dst = h * 3; offsetY = -2.0 * h; }; break;
        //case PlaceType::DOWN_TO_UP: {imgH_dst = h * 3; }; break;
        //}

        int egde_dist = 100;//排样到边缘最近距离为50单位
        if (nestItems.size() == 1) egde_dist = 0;
        if (egde_dist > d) egde_dist = d;
        imgW_dst += d - egde_dist;
        imgH_dst += d - egde_dist;
        offsetX += (d - egde_dist) / 2;
        offsetY += (d - egde_dist) / 2;

        libnest2d::Box maxBox = libnest2d::Box(imgW_dst, imgH_dst, { imgW_dst / 2, imgH_dst / 2 });
        nestResult |= libnest2d::nest(nestItems, maxBox, d, cfg, libnest2d::NestControl());//只能处理凸包


        Clipper3r::Clipper a;
        for (int i = 0; i < size; i++)
        {
            double angle = nestItems.at(i).rotation().toDegrees();
            if (angle != -90.)//区域内模型
            {
                auto trans_item = nestItems.at(i).transformedShape_s();
                a.AddPath(trans_item.Contour, Clipper3r::ptSubject, true);
            }
        }
        Clipper3r::IntRect ibb_dst = a.GetBounds();
        int center_offset_x = 0;// .5 * w - (0.5 * (ibb_dst.right + ibb_dst.left) + offsetX);
        int center_offset_y = 0;// .5 * h - (0.5 * (ibb_dst.bottom + ibb_dst.top) + offsetY);

        std::vector<libnest2d::Item> input_outer_items(1,
            {
                {0, h},
                {w, h},
                {w, 0},
                {0, 0},
                {0, h}
            });//定义第一个轮廓遮挡中心排样区域

        input_outer_items[0].translation({ 1, 0 }); //packed mark

        for (int i = 0; i < size; i++)
        {
            libnest2d::Item& iitem = nestItems.at(i);
            double angle = iitem.rotation().toDegrees();

            if (angle == -90.)
            {
                input_outer_items.push_back(libnest2d::Item(iitem.rawShape()));//收集排样区域外模型
            }
            else
            {
                transData[i] = getTransMatrixFromItem(iitem, center_offset_x + offsetX, center_offset_y + offsetY);
            }
        }

        //////区域外模型排样
        if (input_outer_items.size() > 1)
        {
            cfg.placer_config.object_function = NULL;
            cfg.placer_config.starting_point = libnest2d::NfpPlacer::Config::Alignment::CENTER;
            imgW_dst = w * 1001; imgH_dst = h * 1001;
            maxBox = libnest2d::Box(imgW_dst, imgH_dst, { w / 2, h / 2 });
            nestResult |= libnest2d::nest(input_outer_items, maxBox, d, cfg, libnest2d::NestControl());
            int idx = 1;
            for (int i = 0; i < size; i++)
            {
                libnest2d::Item& iitem = nestItems.at(i);
                double angle = iitem.rotation().toDegrees();
                if (angle == -90.)
                {
                    libnest2d::Item& iitem_dst = input_outer_items.at(idx);
                    transData[i] = getTransMatrixFromItem(iitem_dst, 0, 0);
                    idx++;
                }
            }
        }

        if (func && nestResult)
        {
            size_t size = transData.size();
            for (size_t i = 0; i < size; i++)
                func((int)i, transData.at(i));
        }

        return true;
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

    bool NestPlacer::nest2d(std::vector<NestItemer*>& items, NestItemer* item, Clipper3r::cInt w, Clipper3r::cInt h, Clipper3r::cInt d, PlaceType type, PlaceOneFunc func)
    {
        std::vector<libnest2d::Item> nestedItems;
        for (NestItemer* itemer : items)
            nestedItems.emplace_back(getItem(itemer));

        Clipper3r::cInt offsetX = 0, offsetY = 0;
        int egde_dist = 100;//???ùμ?±??μ?àà??a1??????
        if (egde_dist > d) egde_dist = d;
        offsetX += (d - egde_dist) / 2;
        offsetY += (d - egde_dist) / 2;

        libnest2d::NestConfig<libnest2d::NfpPlacer, libnest2d::FirstFitSelection> cfg;
        InitCfg(cfg);
        std::vector<libnest2d::Item> input;
        std::vector<libnest2d::Item> input_out_pack;
        for (NestItemer* itemer : items)
        {
            Clipper3r::Path ItemPath = itemer->path();
            if (Clipper3r::Orientation(ItemPath))
            {
                Clipper3r::ReversePath(ItemPath);
            }
            ItemPath = libnest2d::shapelike::convexHull(ItemPath);
            libnest2d::Item item = libnest2d::Item(ItemPath);
            Clipper3r::IntPoint trans_data = itemer->translate();
            item.translation({ Clipper3r::cInt(trans_data.X), Clipper3r::cInt(trans_data.Y) });
            float rot = itemer->rotation();
            item.rotation(libnest2d::Radians(rot));
            auto trans_item = item.transformedShape_s();
            if (bOnTheEdge(trans_item.Contour, w, h) == 2)
            {
                item.translation({ Clipper3r::cInt(trans_data.X), Clipper3r::cInt(trans_data.Y) });
                input.push_back(item);
            }
            else
            {
                input_out_pack.push_back(item);
            }
        }

        libnest2d::Item newItem = getItem(item);

        bool can_pack = false;
        newItem.translation({ w / 2, h / 2 });
        for (auto rot : cfg.placer_config.rotations) {
            newItem.rotation(rot);
            auto trans_item = newItem.transformedShape_s();
            if (bOnTheEdge(trans_item.Contour, w, h) == 2)
            {
                can_pack = true;
                break;
            }
        }
        if (!can_pack)
            return false;
        newItem.translation({ 0, 0 });
        input.push_back(newItem);

        Clipper3r::cInt imgW_dst = w, imgH_dst = h;
        libnest2d::Box maxBox = libnest2d::Box(imgW_dst + 2 * offsetX, imgH_dst + 2 * offsetY, { w / 2, h / 2});
        std::size_t result = libnest2d::nest(input, maxBox, d, cfg, libnest2d::NestControl());

        int total_size = input.size();
        libnest2d::Item iitem = input.at(total_size - 1);
        if (iitem.rotation().toDegrees() == -90.)
        {
            iitem.rotation(0);
            iitem.translation({ 0, 0 });
        }
        else
        {         
            func(getTransMatrixFromItem(iitem, 0, 0));
            return true;
        }

        return false;
    }
}