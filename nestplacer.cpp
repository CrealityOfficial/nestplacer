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

    bool NestPlacer::nest2d(Clipper3r::Paths ItemsPaths, int _imageW, int _imageH, int _dist, PlaceType placeType, std::vector<TransMatrix>& transData)
    {
        if (ItemsPaths.size() == 2)
            placeType = PlaceType::CENTER_TO_SIDE;

        size_t size = ItemsPaths.size();
        transData.resize(size);


        libnest2d::NestConfig<libnest2d::NfpPlacer, libnest2d::FirstFitSelection> cfg;

        cfg.placer_config.alignment = libnest2d::NfpPlacer::Config::Alignment::DONT_ALIGN;
        switch (placeType)
        {
        case PlaceType::CENTER_TO_SIDE: cfg.placer_config.starting_point = libnest2d::NfpPlacer::Config::Alignment::CENTER; break;
        case PlaceType::MID_TO_UP_DOWN: cfg.placer_config.starting_point = libnest2d::NfpPlacer::Config::Alignment::CENTER; break;
            //��Y������������������������starting_point = CENTER��alignment = DONT_ALIGN��alignment = CENTER
        case PlaceType::MID_TO_LEFT_RIGHT: cfg.placer_config.starting_point = libnest2d::NfpPlacer::Config::Alignment::CENTER; break;
            //��X������������������������starting_point = CENTER��alignment = DONT_ALIGN��alignment = CENTER
        case PlaceType::LEFT_TO_RIGHT: cfg.placer_config.starting_point = libnest2d::NfpPlacer::Config::Alignment::BOTTOM_LEFT; break;
            //��X��0���ҷ���������starting_point = BOTTOM_LEFT��starting_point = TOP_LEFT��alignment = DONT_ALIGN
        case PlaceType::RIGHT_TO_LEFT: cfg.placer_config.starting_point = libnest2d::NfpPlacer::Config::Alignment::BOTTOM_RIGHT; break;
            //��X��max������������starting_point = BOTTOM_RIGHT��starting_point = TOP_RIGHT��alignment = DONT_ALIGN
        case PlaceType::UP_TO_DOWN: cfg.placer_config.starting_point = libnest2d::NfpPlacer::Config::Alignment::TOP_LEFT; break;
            //��Y��0���·���������starting_point = BOTTOM_LEFT��starting_point = BOTTOM_RIGHT��alignment = DONT_ALIGN
        case PlaceType::DOWN_TO_UP: cfg.placer_config.starting_point = libnest2d::NfpPlacer::Config::Alignment::BOTTOM_LEFT; break;
            //��Y��max���Ϸ���������starting_point = TOP_LEFT��starting_point = TOP_RIGHT��alignment = DONT_ALIGN
        }

        cfg.placer_config.rotations.push_back(libnest2d::Radians(Pi / 4.0));//����ο�����ת��
        cfg.placer_config.rotations.push_back(libnest2d::Radians(Pi * 3 / 4.0));
        cfg.placer_config.rotations.push_back(libnest2d::Radians(Pi * 5 / 4.0));
        cfg.placer_config.rotations.push_back(libnest2d::Radians(Pi * 7 / 4.0));
        //cfg.placer_config.accuracy; //�Ż��� 
        //cfg.placer_config.explore_holes;  //�����Ƿ����ͼԪ,ĿǰԴ���л�δʵ��
        //cfg.placer_config.parallel;  //���ö��߳�
        //cfg.placer_config.before_packing; //�ڷ���һ��Item֮ǰ���ȶ�ǰ���Ѿ��ڷźõ�Item���е����ĺ�������Ϊ�գ���Item�ĺ�λ�ú󽫲��ٱ䶯�����뺯���ӿ�
        //cfg.placer_config.object_function;  //����Ż������������ֵ��С���Ż����Դ˸ı��ŷŷ�ʽ�����뺯���ӿ�

        //cfg.selector_config;

        libnest2d::NestControl ctl;

        cfg.placer_config.object_function = [placeType](const libnest2d::Item&)  //�Ż�����
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

        double scaleFactor = 100.0;
        auto convert = [&scaleFactor, &ItemsPaths](Clipper3r::Path& oItem, int index) {
            Clipper3r::Path lines = ItemsPaths.at(index);
            size_t size = lines.size();
            if (size > 0)
            {
                oItem.resize(size);
                for (size_t i = 0; i < size; ++i)
                {
                    oItem.at(i).X = (Clipper3r::cInt)(lines.at(i).X * scaleFactor);
                    oItem.at(i).Y = (Clipper3r::cInt)(lines.at(i).Y * scaleFactor);
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
                Clipper3r::ReversePath(ItemPath);//����������
            }
            ItemPath = libnest2d::shapelike::convexHull(ItemPath);//��֧��͹��������
            input.push_back(libnest2d::Item(ItemPath));
        }

        int imgW = (Clipper3r::cInt)(_imageW * scaleFactor);   //���������
        int imgH = (Clipper3r::cInt)(_imageH * scaleFactor);   //���������
        int dist = (Clipper3r::cInt)(_dist * scaleFactor);     //�������

        int imgW_dst = imgW, imgH_dst = imgH;

        double offsetX = 0., offsetY = 0.;
        switch (placeType)  //��������Ŵ�����
        {
        case PlaceType::CENTER_TO_SIDE: {imgW_dst = imgW * 3; imgH_dst = imgH * 3; offsetX = -1.0 * imgW; offsetY = -1.0 * imgH; }; break;
        case PlaceType::MID_TO_UP_DOWN: {imgH_dst = imgH * 3;  offsetY = -1.0 * imgH; }; break;
        case PlaceType::MID_TO_LEFT_RIGHT: {imgW_dst = imgW * 3; offsetX = -1.0 * imgW; }; break;
        case PlaceType::LEFT_TO_RIGHT: {imgW_dst = imgW * 3; }; break;
        case PlaceType::RIGHT_TO_LEFT: {imgW_dst = imgW * 3; offsetX = -2.0 * imgW; }; break;
        case PlaceType::UP_TO_DOWN: {imgH_dst = imgH * 3; offsetY = -2.0 * imgH; }; break;
        case PlaceType::DOWN_TO_UP: {imgH_dst = imgH * 3; }; break;
        }
#if(0)
        int edge_dist = 2 * scaleFactor;//��������Ե�������Ϊ1um
        if (edge_dist > dist) edge_dist = dist;
        imgW_dst += dist - edge_dist;
        imgH_dst += dist - edge_dist;
        offsetX += (dist - edge_dist) / 2;
        offsetY += (dist - edge_dist) / 2;
#else 
        imgW_dst += dist;
        imgH_dst += dist;
        offsetX += dist / 2;
        offsetY += dist / 2;
#endif

        libnest2d::Box maxBox = libnest2d::Box(imgW_dst, imgH_dst, { imgW_dst / 2, imgH_dst / 2 });
        std::size_t result = libnest2d::nest(input, maxBox, dist, cfg, ctl);//ֻ�ܴ���͹��


        Clipper3r::Clipper a;
        for (int i = 0; i < size; i++)
        {
            double angle = input.at(i).rotation().toDegrees();
            if (angle != -90.)//������ģ��
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
            });//�����һ�������ڵ�������������

        input_outer_items[0].translation({ 1, 0 }); //packed mark

        for (int i = 0; i < size; i++)
        {
            libnest2d::Item& iitem = input.at(i);

            int model_offset_x = iitem.translation().X;
            int model_offset_y = iitem.translation().Y;
            double angle = iitem.rotation().toDegrees();

            if (angle == -90.)
            {
                input_outer_items.push_back(libnest2d::Item(iitem.rawShape()));//�ռ�����������ģ��
            }
            else
            {
                model_offset_x += center_offset_x;
                model_offset_y += center_offset_y;
                TransMatrix itemTrans;
                itemTrans.rotation = input[i].rotation().toDegrees();
                itemTrans.x = (model_offset_x + offsetX - 0.5 * imgW) / scaleFactor;
                itemTrans.y = (model_offset_y + offsetY - 0.5 * imgH) / scaleFactor;
                transData[i] = itemTrans;
            }
        }

          //////������ģ������
        if (input_outer_items.size() == 1) return true;
        cfg.placer_config.object_function = NULL;
        cfg.placer_config.starting_point = libnest2d::NfpPlacer::Config::Alignment::CENTER;
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
                itemTrans.x = (iitem_dst.translation().X - 0.5 * imgW) / scaleFactor;
                itemTrans.y = (iitem_dst.translation().Y - 0.5 * imgH) / scaleFactor;
                transData[i] = itemTrans;
                idx++;
            }
        }

        return result == size;
    }


    void NestPlacer::layout_all_nest(trimesh::box3 workspaceBox, std::vector<int> modelIndices,
        std::vector < std::vector<trimesh::vec3>> models, std::function<void(int, trimesh::vec3)> modelPositionUpdateFunc)
    {
        double scaleFactor = 1000.0;
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
            newBoxCenter.x = transData.at(i).x / scaleFactor + 0.5 * imageW;
            newBoxCenter.y = transData.at(i).y / scaleFactor + 0.5 * imageH;
            newBoxCenter.z = transData.at(i).rotation;
            int modelIndexI = modelIndices[i];
            modelPositionUpdateFunc(modelIndexI, newBoxCenter);
        }

    }

}