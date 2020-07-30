#include "decompaction.h"

bool comp(std::pair<int, point_2d> a, std::pair<int, point_2d> b)
{
    return a.second < b.second;
}

void Decompaction::createRTree()
{
    const double buffer_distance = m_db.getLargestClearance() / 2;
    const double via_buffer_distance = 0.15;
    const int points_per_circle = 36;
    boost::geometry::strategy::buffer::distance_symmetric<coordinate_type> distance_strategy(buffer_distance);
    boost::geometry::strategy::buffer::distance_symmetric<coordinate_type> via_distance_strategy(via_buffer_distance);
    boost::geometry::strategy::buffer::join_round join_strategy(points_per_circle);
    boost::geometry::strategy::buffer::end_round end_strategy(points_per_circle);
    boost::geometry::strategy::buffer::point_circle circle_strategy(points_per_circle);
    boost::geometry::strategy::buffer::side_straight side_strategy;
    m_numLayer = m_db.getNumCopperLayers();
    m_rtrees.resize(m_numLayer);
    std::vector<net> nets = m_db.getNets();
    int id = 0;
    double clearance = 2 * (m_db.getLargestClearance() / 2); //For bus!!!
    std::cout << "clearance: " << clearance << std::endl;
    for (auto &&net : nets)
    {
        bool isBus = net.isBus();
        std::vector<Pin> pins = net.getPins();
        std::vector<Segment> segments = net.getSegments();
        std::vector<Via> vias = net.getVias();

        for (auto &&s : segments)
        {
            points_2d line = s.getPos();
            auto width = s.getWidth();
            //std::cout << "width: " << width << std::endl;
            int layerId = m_db.getLayerId(s.getLayer());
            points_2d coord = segmentToOctagon(line, width, clearance);
            points_2d coordR = segmentToRelativeOctagon(line, width, clearance);
            //std::cout << "Segment: (" << line[0].m_x << "," << line[0].m_y << ")" << "(" << line[1].m_x << "," << line[1].m_y << ")" << std::endl;
            polygon_t polygon;
            //std::cout << "Polygon(";
            for (auto &&p : coord)
            {
                //    std::cout << "(" << p.m_x << "," << p.m_y << "),";
                double x = p.m_x;
                double y = p.m_y;
                bg::append(polygon.outer(), point(x, y));
            }
            // std::cout << std::endl;
            bg::append(polygon.outer(), point(coord[0].m_x, coord[0].m_y));
            linestring ls;
            bg::append(ls, point(line[0].m_x, line[0].m_y));
            bg::append(ls, point(line[1].m_x, line[1].m_y));
            multipoly mpoly;
            boost::geometry::buffer(ls, mpoly, distance_strategy, side_strategy, join_strategy, end_strategy, circle_strategy);

            box b = bg::return_envelope<box>(polygon);
            Object obj(ObjectType::SEGMENT, s.getId(), s.getNetId(), -1, -1);
            obj.setShape(coord);
            obj.setPoly(polygon);
            obj.setRelativeShape(coordR);
            obj.setBBox(b);
            obj.setLayer(layerId);
            obj.setPos(s.getPos());
            obj.setIsBus(isBus);
            obj.setPreviousPosition();
            obj.setMultipoly(mpoly);
            if (layerId >= m_numLayer)
            {
                layerId = m_numLayer - 1;
            }

            m_rtrees[layerId].insert(std::make_pair(b, id));
            obj.setRTreeId(std::make_pair(layerId, id));

            m_objects.push_back(obj);
            ++id;
        }

        for (auto &&v : vias)
        {
            point_2d pos = v.getPos();
            std::cout << "Via: " << pos.m_x << "," << pos.m_y << std::endl;
            auto size = v.getSize();
            points_2d coord = viaToOctagon(size, pos, clearance);
            points_2d coordRe = viaToRelativeOctagon(size, clearance);
            polygon_t polygon;
            std::cout << "Polygon(";
            for (auto &&p : coord)
            {
                std::cout << "(" << p.m_x << "," << p.m_y << "),";
                double x = p.m_x;
                double y = p.m_y;
                bg::append(polygon.outer(), point(x, y));
            }
            std::cout << std::endl;
            bg::append(polygon.outer(), point(coord[0].m_x, coord[0].m_y));
            std::vector<std::string> layers = v.getLayers();
            box b = bg::return_envelope<box>(polygon);
            point p{point{pos.m_x, pos.m_y}};
            multipoly mpoly;
            boost::geometry::buffer(p, mpoly, via_distance_strategy, side_strategy, join_strategy, end_strategy, circle_strategy);
            Object obj(ObjectType::VIA, v.getId(), v.getNetId(), -1, -1);

            obj.setShape(coord);
            obj.setPoly(polygon);
            obj.setMultipoly(mpoly);
            obj.setRelativeShape(coordRe);
            obj.setBBox(b);
            obj.setPos(pos);
            obj.setIsBus(isBus);
            int layerId1 = m_db.getLayerId(layers[0]), layerId2 = m_db.getLayerId(layers[1]);
            if (layerId2 >= m_numLayer)
                layerId2 = m_numLayer - 1;
            for (int layerId = layerId1; layerId <= layerId2; ++layerId)
            {
                m_rtrees[layerId].insert(std::make_pair(b, id));
                obj.setRTreeId(std::make_pair(layerId, id));
            }
            ++id;
            m_objects.push_back(obj);
        }

        for (auto &&pin : pins)
        {
            int padId = pin.getPadstackId();
            int compId = pin.getCompId();
            int instId = pin.getInstId();
            polygon_t polygon;
            component comp = m_db.getComponent(compId);
            instance inst = m_db.getInstance(instId);

            auto &pad = comp.getPadstack(padId);
            auto pinPos = point_2d{};
            m_db.getPinPosition(pin, &pinPos);
            points_2d shape = shape_to_coords(pad.getSize(), pinPos, pad.getPadShape(), inst.getAngle(), pad.getAngle(), pad.getRoundRectRatio(), 36);
            points_2d coord = pinShapeToOctagon(pad.getSize(), pad.getPos(), clearance, inst.getAngle(), pad.getAngle(), pad.getPadShape());
            auto coordRe = points_2d{};
            m_db.getPinShapeRelativeCoordsToModule(pad, inst, coord, &coordRe);
            std::cout << "net: " << net.getId() << " comp Name: " << comp.getName() << " inst Name: " << inst.getName() << " pad: " << pad.getName() << std::endl;
            std::cout << " \tinst pos: (" << inst.getX() << "," << inst.getY() << ")" << std::endl;
            std::cout << " \tpin pos: (" << pinPos.m_x << "," << pinPos.m_y << ")" << std::endl;
            for (auto &&p : coord)
            {
                p.m_x = p.m_x + pinPos.m_x;
                p.m_y = p.m_y + pinPos.m_y;
            }

            std::cout << "Polygon(";
            for (auto &&p : coord)
            {
                std::cout << "(" << p.m_x << "," << p.m_y << "),";
                double x = p.m_x;
                double y = p.m_y;
                bg::append(polygon.outer(), point(x, y));
            }
            std::cout << std::endl;
            bg::append(polygon.outer(), point(coord[0].m_x, coord[0].m_y));
            std::vector<int> layers = m_db.getPinLayer(instId, padId);
            box b = bg::return_envelope<box>(polygon);
            multipoly mpoly;
            mpoly.resize(1);
            for (auto &&p : shape)
            {
                double x = p.m_x;
                double y = p.m_y;
                bg::append(mpoly[0].outer(), point{x, y});
            }
            Object obj(ObjectType::PIN, pad.getId(), net.getId(), compId, instId);
            obj.setPos(pinPos);
            obj.setRelativeShape(coordRe);
            obj.setShape(coord);
            obj.setPoly(polygon);
            obj.setBBox(b);
            obj.setLocked(inst.isLocked());
            obj.setIsBus(isBus);
            obj.setMultipoly(mpoly);
            for (auto &&layer : layers)
            {
                int layerId = layer;
                if (layer >= m_numLayer)
                {
                    layerId = m_numLayer - 1;
                }
                m_rtrees[layerId].insert(std::make_pair(b, id));
                obj.setRTreeId(std::make_pair(layerId, id));
            }
            m_objects.push_back(obj);
            ++id;
        }
    }

    std::vector<Pin> pins = m_db.getUnconnectedPins();
    for (auto &&pin : pins)
    {
        int padId = pin.getPadstackId();
        int compId = pin.getCompId();
        int instId = pin.getInstId();
        polygon_t polygon;
        component comp = m_db.getComponent(compId);
        instance inst = m_db.getInstance(instId);
        auto &pad = comp.getPadstack(padId);
        auto pinPos = point_2d{};
        points_2d shape = shape_to_coords(pad.getSize(), pinPos, pad.getPadShape(), inst.getAngle(), pad.getAngle(), pad.getRoundRectRatio(), 36);
        points_2d coord = pinShapeToOctagon(pad.getSize(), pad.getPos(), 0.1, inst.getAngle(), pad.getAngle(), pad.getPadShape());
        auto coordRe = points_2d{};
        m_db.getPinShapeRelativeCoordsToModule(pad, inst, coord, &coordRe);

        m_db.getPinPosition(pin, &pinPos);

        for (auto &&p : coord)
        {
            p.m_x = p.m_x + pinPos.m_x;
            p.m_y = p.m_y + pinPos.m_y;
        }

        for (auto &&p : coord)
        {
            double x = p.m_x;
            double y = p.m_y;
            bg::append(polygon.outer(), point(x, y));
        }
        bg::append(polygon.outer(), point(coord[0].m_x, coord[0].m_y));
        std::vector<int> layers = m_db.getPinLayer(instId, padId);
        box b = bg::return_envelope<box>(polygon);
        multipoly mpoly;
        mpoly.resize(1);
        for (auto &&p : shape)
        {
            double x = p.m_x;
            double y = p.m_y;
            bg::append(mpoly[0].outer(), point(x, y));
        }
        Object obj(ObjectType::PIN, pad.getId(), -1, compId, instId);
        obj.setPos(pinPos);
        obj.setShape(coord);
        obj.setRelativeShape(coordRe);
        obj.setPoly(polygon);
        obj.setBBox(b);

        obj.setMultipoly(mpoly);

        obj.setLocked(inst.isLocked());

        for (auto &&layer : layers)
        {
            int layerId = layer;
            if (layer >= m_numLayer)
            {
                layerId = m_numLayer - 1;
            }
            m_rtrees[layerId].insert(std::make_pair(b, id));
            obj.setRTreeId(std::make_pair(layerId, id));
        }
        m_objects.push_back(obj);
        ++id;
    }
}
/*
bool Decompaction::checkIntersection()
{
    
    

    return true;
}*/

std::vector<std::vector<double>> Decompaction::buildRelation(int &obj1Id, const int &obj2Id)
{
    auto &&obj1 = m_objects[obj1Id];
    auto &&obj2 = m_objects[obj2Id];
    auto &&shape1 = obj1.getShape();
    auto &&shape2 = obj2.getShape();
    auto &&reShape1 = obj1.getRelativeShape();
    auto &&reShape2 = obj2.getRelativeShape();
    point_2d center;
    points_2d pos = obj1.getPos();
    points_2d proCoord1, proCoord2, coord;

    if (obj1.getType() == ObjectType::SEGMENT)
    {
        center.m_x = (pos[0].m_x + pos[1].m_x) / 2;
        center.m_y = (pos[0].m_y + pos[1].m_y) / 2;
    }
    else
    {
        center = pos[0];
    }

    std::vector<std::vector<std::pair<int, point_2d>>> fourCoods;
    double dist;
    bool overlap = true;
    std::vector<std::pair<int, point_2d>> proj1, proj2;
    std::map<double, std::vector<std::pair<int, int>>> overlapResult, nonoverlapResult;

    for (int i = 0; i < 4; ++i)
    {
        proCoord1 = projection(center, shape1, i * 45);
        proCoord2 = projection(center, shape2, i * 45);

        /*std::cout << "shape1" << std::endl;
        for (auto &&p : proCoord1)
        {
            std::cout << p.m_x << " " << p.m_y << std::endl;
        }
        std::cout << "shape2" << std::endl;
        for (auto &&p : proCoord2)
        {
            std::cout << p.m_x << " " << p.m_y << std::endl;
        }*/

        std::vector<std::pair<int, point_2d>> projectionVec;
        std::vector<std::pair<int, point_2d>> project1, project2;

        for (int i = 0; i < proCoord1.size(); ++i)
        {
            int id = 10 + i;
            projectionVec.push_back(std::make_pair(id, proCoord1[i]));
            project1.push_back(std::make_pair(i, proCoord1[i]));
        }
        for (int i = 0; i < proCoord2.size(); ++i)
        {
            int id = 20 + i;
            projectionVec.push_back(std::make_pair(id, proCoord2[i]));
            project2.push_back(std::make_pair(i, proCoord2[i]));
        }
        std::sort(projectionVec.begin(), projectionVec.end(), comp);
        std::sort(project1.begin(), project1.end(), comp);
        std::sort(project2.begin(), project2.end(), comp);

        for (auto &&p : project1)
        {
        }

        // Case1:
        //    obj1         obj2
        // *--------*
        //        *---------:intersect
        if (project1[0].second < project2[0].second && project2[0].second < project1[7].second &&
            project1[7].second < project2[7].second)
        {
            dist = project1[7].second.getDistance(project1[7].second, project2[0].second);
            overlapResult[dist].push_back(std::make_pair(project1[6].first, project1[7].first));
            overlapResult[dist].push_back(std::make_pair(project2[0].first, project2[1].first));
            overlap = true;
            if (m_verbose)
                std::cout << "intersect1: " << dist << std::endl;
        }
        else if (project2[0].second < project1[0].second && project1[0].second < project2[7].second &&
                 project2[7].second < project1[7].second)
        {

            dist = project2[7].second.getDistance(project2[7].second, project1[0].second);
            overlapResult[dist].push_back(std::make_pair(project1[0].first, project1[1].first));
            overlapResult[dist].push_back(std::make_pair(project2[6].first, project2[7].first));
            overlap = true;
            if (m_verbose)
                std::cout << "intersect2: " << dist << std::endl;
        }
        // Case2:
        //
        // *--------*   *---------*:sep
        else if (project1[7].second <= project2[0].second)
        {
            dist = project1[7].second.getDistance(project1[7].second, project2[0].second);
            nonoverlapResult[dist].push_back(std::make_pair(project1[6].first, project1[7].first));
            nonoverlapResult[dist].push_back(std::make_pair(project2[0].first, project2[1].first));
            overlap = false;
            if (m_verbose)
                std::cout << "nonoverlap1: " << dist << std::endl;
        }
        else if (project2[7].second <= project1[0].second)
        {
            dist = project1[7].second.getDistance(project1[7].second, project2[0].second);
            nonoverlapResult[dist].push_back(std::make_pair(project1[0].first, project1[1].first));
            nonoverlapResult[dist].push_back(std::make_pair(project2[6].first, project2[7].first));
            overlap = false;
            if (m_verbose)
                std::cout << "nonoverlap2: " << dist << std::endl;
        }
        // Case3:
        //     *-----*
        // *-------------*
        else if (project1[0].second <= project2[0].second && project2[0].second < project1[7].second && project1[0].second < project2[7].second && project2[7].second <= project1[7].second)
        {
            double dist1 = 0, dist2 = 0;
            dist1 = project2[0].second.getDistance(project2[0].second, project1[7].second);
            dist2 = project1[0].second.getDistance(project1[0].second, project2[7].second);

            overlap = true;
            if (dist1 <= dist2)
            {
                overlapResult[dist1].push_back(std::make_pair(project1[6].first, project1[7].first));
                overlapResult[dist1].push_back(std::make_pair(project2[0].first, project2[1].first));
                if (m_verbose)
                    std::cout << "overlap1-1: " << dist << std::endl;
            }
            else
            {
                overlapResult[dist2].push_back(std::make_pair(project1[0].first, project1[1].first));
                overlapResult[dist2].push_back(std::make_pair(project2[6].first, project2[7].first));
                if (m_verbose)
                    std::cout << "overlap2-1: " << dist << std::endl;
            }
        }
        else if (project2[0].second <= project1[0].second && project1[0].second < project2[7].second && project2[0].second < project1[7].second &&
                 project1[7].second <= project2[7].second)
        {
            double dist1 = 0, dist2 = 0;
            dist1 = project1[0].second.getDistance(project1[0].second, project2[7].second);
            dist2 = project2[0].second.getDistance(project2[0].second, project1[7].second);

            if (dist1 <= dist2)
            {
                overlapResult[dist1].push_back(std::make_pair(project1[0].first, project1[1].first));
                overlapResult[dist1].push_back(std::make_pair(project2[6].first, project2[7].first));
                if (m_verbose)
                    std::cout << "overlap2-1: " << dist << std::endl;
            }
            else
            {
                overlapResult[dist2].push_back(std::make_pair(project1[6].first, project1[7].first));
                overlapResult[dist2].push_back(std::make_pair(project2[0].first, project2[1].first));
                if (m_verbose)
                    std::cout << "overlap2-2: " << dist << std::endl;
            }
            overlap = true;
        }
        else
        {
            if (m_verbose)
                std::cout << "no suitable" << std::endl;
        }

        //if(overlap)
        /*

        std::cout << "degree :" << i*45 << std::endl;
        int obj1L, obj1R, obj2L, obj2R;
        bool obj1SepObj2 = false, obj2SepObj1 = false, obj1interObj2 = false,
             obj2interObj1 = false, obj1ConObj2 = false, obj2ConObj1 = false;

        // *---------*
        //       *----------*:inter
        //        *----*
        //  *----------------*:con
        int count1 = 0, count2 = 0;

        for (int j = 0 ; j < projectionVec.size(); ++j) {
        //    std::cout << projectionVec[j].first << std::endl;
        //    printPoint(projectionVec[j].second);
            if(projectionVec[j].first < 20) {
                ++count1;
                if(count1 == 1) obj1L = projectionVec[j].first;
                else if(count1 == 8 && count2 == 0) {
                    obj1R = projectionVec[j].first;
                    obj1SepObj2 = true;
                } 
            } else {
                 ++count2;
                if(count2 == 1) obj2L = projectionVec[j].first;
                else if(count2 == 8 && count1 == 0) {
                    obj2R = projectionVec[j].first;
                    obj2SepObj1 = true;
                }
            }
        }*/
    }

    int obj1Angle = obj1.getAngle();
    std::vector<std::vector<double>> equ;
    auto centerPos = obj1.getCenterPos();
    if (nonoverlapResult.empty())
    {
        auto &&p = overlapResult.begin(); //std::map<double, std::vector<std::pair<int, int>>> dist. {obj1.point, obj2.point}
        auto &&point = p->second;
        //equ = lineEquation(shape1[point[0].first], shape1[point[0].second], reShape2[point[1].first], reShape2[point[1].second]);
        std::cout << shape1[point[0].first] << std::endl;
        std::cout << shape1[point[0].second] << std::endl;
        std::cout << reShape2[point[1].first] << std::endl;
        std::cout << reShape2[point[1].second] << std::endl;
        equ = inequalityLineEquation(shape1[point[0].first], shape1[point[0].second], reShape2[point[1].first], reShape2[point[1].second], centerPos);
        if (m_verbose)
        {
            std::cout << "point: " << shape1[point[0].first] << ", " << shape1[point[0].second] << std::endl;
            std::cout << "relative point: " << reShape2[point[1].first] << ", " << reShape2[point[1].second] << std::endl;
            std::cout << "obj2 relative point: ";
            printPolygon(reShape2);
        }
    }
    else
    {
        auto &&p = nonoverlapResult.rbegin();
        auto &&point = p->second;
        //equ = lineEquation(shape1[point[0].first], shape1[point[0].second], reShape2[point[1].first], reShape2[point[1].second]);
        equ = inequalityLineEquation(shape1[point[0].first], shape1[point[0].second], reShape2[point[1].first], reShape2[point[1].second], centerPos);
        if (m_verbose)
        {
            std::cout << "point: " << shape1[point[0].first] << ", " << shape1[point[0].second] << std::endl;
            std::cout << "relative point: " << reShape2[point[1].first] << ", " << reShape2[point[1].second] << std::endl;
            std::cout << "obj2 relative point: ";
            printPolygon(reShape2);
        }
    }
    return equ;
}

void Decompaction::traverseRTree()
{
    std::cout << "=========================" << std::endl;
    double MAX_DIST = 1.5;
    bg::model::point<double, 2, bg::cs::cartesian> point1;
    bg::model::point<double, 2, bg::cs::cartesian> point2;
    for (auto &&obj1 : m_objects)
    {
        /*auto &&obj = m_objects[1];
        if (obj != obj1)
            continue;*/
        /*int layer = obj1.getLayer();
        if (layer != 0)
            continue;*/
        if (obj1.getType() == ObjectType::PIN)
        {
            auto compId = obj1.getCompId();
            auto instId = obj1.getInstId();
            auto padId = obj1.getDBId();
            component comp = m_db.getComponent(compId);
            instance inst = m_db.getInstance(instId);
            auto &pad = comp.getPadstack(padId);
            std::cout << "comp: " << comp.getName() << " inst: " << inst.getName();
            std::cout << " pad: " << pad.getName() << std::endl;
        }
        auto bbox = obj1.getBBox();
        std::vector<std::pair<int, int>> &rtreeId = obj1.getRtreeId();
        for (size_t i = 0; i < rtreeId.size(); ++i)
        {

            int rId = rtreeId[i].first;
            std::cout << "Object Id: " << rtreeId[i].first << "," << rtreeId[i].second << std::endl;
            auto coord = obj1.getShape();
            printPolygon(coord);
            auto centerPos = obj1.getCenterPos();

            double minX = bg::get<bg::min_corner, 0>(bbox);
            double minY = bg::get<bg::min_corner, 1>(bbox);
            double maxX = bg::get<bg::max_corner, 0>(bbox);
            double maxY = bg::get<bg::max_corner, 1>(bbox);
            minX -= MAX_DIST;
            minY -= MAX_DIST;
            maxX += MAX_DIST;
            maxY += MAX_DIST;
            point1.set<0>(minX);
            point1.set<1>(minY);
            point2.set<0>(maxX);
            point2.set<1>(maxY);
            box query_box(point1, point2);
            std::vector<value> result_s;
            m_rtrees[rId].query(bgi::intersects(query_box), std::back_inserter(result_s));

            std::cout << "spatial query box:" << std::endl;
            std::cout << bg::wkt<box>(query_box) << std::endl;
            std::cout << "spatial query result:" << std::endl;
            BOOST_FOREACH (value const &v, result_s)
            {
                std::cout << bg::wkt<box>(v.first) << " - " << v.second << std::endl;

                auto &&obj2 = m_objects[v.second];
                std::deque<polygon_t> output;
                polygon_t poly1 = obj1.getPoly();
                polygon_t poly2 = obj2.getPoly();
                std::vector<std::pair<int, int>> &rtreeId2 = obj2.getRtreeId();
                std::cout << "Object2 Id: " << rtreeId2[0].first << "," << rtreeId2[0].second << std::endl;
                auto coord = obj2.getShape();
                printPolygon(coord);
                auto reCoord = obj2.getRelativeShape();
                auto centerPos2 = obj2.getCenterPos();
                if (obj2.getType() == ObjectType::PIN)
                {
                    instance inst = m_db.getInstance(obj2.getInstId());
                    centerPos2 = point_2d{inst.getX(), inst.getY()};
                }
                auto co = points_2d{};
                for (auto &&coor : reCoord)
                {
                    co.push_back(point_2d{coor.m_x + centerPos2.m_x, coor.m_y + centerPos2.m_y});
                }
                /*std::cout << "relative coord" << std::endl;
                printPolygon(reCoord);
                std::cout << "relatvie coord + pos" << std::endl;
                printPolygon(co);*/

                if (obj1.getNetId() != obj2.getNetId())
                {
                    std::vector<std::vector<double>> equ = buildRelation(rtreeId[0].second, v.second);
                    //std::vector<double> eq = getInequalityEquation(equ[0], centerPos);
                    obj2.addEquation(equ[0]);
                    printInequalityEquation(equ[0]);
                }
                /*std::cout << "!!Overlap!!" << std::endl;
            if (boost::geometry::intersects(poly1, poly2)) {
                boost::geometry::intersection(poly1, poly2, output);
                BOOST_FOREACH(polygon_t const& p, output)
                {
                    std::cout << "\tarea:" << boost::geometry::area(p) << std::endl;
                    std::cout << " obj1 id: " << obj1.getId() << ", obj2 id: " << obj2.getId() << std::endl;
                }

                if (obj2.getType() == ObjectType::SEGMENT) {
                    auto dbId = obj2.getDBId();
                    points_2d pos = obj2.getPos();
                    std::cout << "segment: ";
                    for(auto &&p : pos){ 
                        std::cout << "(" << p.m_x << "," << p.m_y << ") ";
                        
                    }
                    std::cout << std::endl;

                    std::cout << "Polygon(";
                    auto coord = obj2.getShape();
                    for(auto &&p : coord)
                    {
                        std::cout << "(" << p.m_x << "," << p.m_y << "),";

                    }
                    std::cout << std::endl;
                }
            }*/
            }
        }
    }

    /*auto center = point_2d{0,0};
        for (auto &&p : obj1.getShape())
        {
            center.m_x += p.m_x;
            center.m_y += p.m_y;
        }   */
    //}
}

points_2d Decompaction::projection(point_2d &center, points_2d &octagon, int degree)
{
    auto coord = points_2d{};
    if (degree == 0)
    {
        for (auto &&p : octagon)
        {
            coord.push_back(point_2d{p.m_x, center.m_y});
        }
        return coord;
    }
    else if (degree == 45)
    {
        double b1 = center.m_y - center.m_x;
        for (auto &&p : octagon)
        {
            double b2 = p.m_x + p.m_y;
            auto point = point_2d{};
            point.m_x = 0.5 * (b2 - b1);
            point.m_y = 0.5 * (b1 + b2);
            coord.push_back(point);
        }
        return coord;
    }
    else if (degree == 90)
    {
        for (auto &&p : octagon)
        {
            coord.push_back(point_2d{center.m_x, p.m_y});
        }
        return coord;
    }
    else if (degree == 135)
    {
        double b1 = center.m_x + center.m_y;
        for (auto &&p : octagon)
        {
            double b2 = p.m_y - p.m_x;
            auto point = point_2d{};
            point.m_x = 0.5 * (b1 - b2);
            point.m_y = 0.5 * (b1 + b2);
            coord.push_back(point);
        }
        return coord;
    }
}

void Decompaction::testProjection()
{
    for (auto &&obj1 : m_objects)
    {
        for (auto &&obj2 : m_objects)
        {
            auto coord1 = obj1.getShape();
            auto coord2 = obj2.getShape();
            auto cen = point_2d{0, 0};

            for (size_t i = 0; i < 8; ++i)
            {
                cen.m_x += coord1[i].m_x;
                cen.m_y += coord1[i].m_y;
            }

            cen.m_x = cen.m_x / 8;
            cen.m_y = cen.m_y / 8;
            auto p1 = projection(cen, coord1, 0);
            auto p2 = projection(cen, coord2, 0);
            std::cout << "=================================" << std::endl;

            std::cout << "Polygon(";
            for (auto &&p : coord1)
            {
                std::cout << "(" << p.m_x << "," << p.m_y << "),";
            }
            std::cout << std::endl;
            for (auto &&p : p1)
            {
                std::cout << "Point({" << p.m_x << "," << p.m_y << "})" << std::endl;
            }

            std::cout << "Polygon(";
            for (auto &&p : coord2)
            {
                std::cout << "(" << p.m_x << "," << p.m_y << "),";
            }
            std::cout << std::endl;
            for (auto &&p : p2)
            {
                std::cout << "Point({" << p.m_x << "," << p.m_y << "})" << std::endl;
            }
        }
    }
}

/*
void Decompaction::checkClearance()
{
    if (obj1.getType() == ObjectType::PIN){
        auto compId = obj1.getCompId();
        auto instId = obj1.getInstId();
        auto padId = obj1.getDBId();
        component comp = m_db.getComponent(compId);
        instance inst = m_db.getInstance(instId);
        auto &pad = comp.getPadstack(padId);
        std::cout << "comp: " << comp.getName() << " inst: " << inst.getName();
        std::cout << " pad: " << pad.getName() << std::endl;
    }
        auto bbox = obj1.getBBox();
        std::vector< std::pair<int,int> > & rtreeId = obj1.getRtreeId();
        int rId = rtreeId[0].first;
        std::cout << "Object Id: " << rtreeId[0].first << "," << rtreeId[0].second << std::endl;
        auto coord = obj1.getShape();
        printPolygon(coord);

        double minX = bg::get<bg::min_corner, 0>(bbox);
        double minY = bg::get<bg::min_corner, 1>(bbox);
        double maxX = bg::get<bg::max_corner, 0>(bbox);
        double maxY = bg::get<bg::max_corner, 1>(bbox);
        minX -= MAX_DIST;
        minY -= MAX_DIST;
        maxX += MAX_DIST;
        maxY += MAX_DIST;
        point1.set<0>(minX);
        point1.set<1>(minY);
        point2.set<0>(maxX);
        point2.set<1>(maxY);
        box query_box(point1, point2);
        std::vector<value> result_s;
        m_rtrees[rId].query(bgi::intersects(query_box), std::back_inserter(result_s));


        std::cout << "spatial query box:" << std::endl;
        std::cout << bg::wkt<box>(query_box) << std::endl;
        std::cout << "spatial query result:" << std::endl;
        BOOST_FOREACH(value const& v, result_s) {
            std::cout << bg::wkt<box>(v.first) << " - " << v.second << std::endl;
            

            auto &&obj2 = m_objects[v.second];
            std::deque<polygon_t> output;
            polygon_t poly1 = obj1.getPoly();
            polygon_t poly2 = obj2.getPoly();
            std::vector< std::pair<int,int> > & rtreeId2 = obj2.getRtreeId();
            std::cout << "Object2 Id: " << rtreeId2[0].first << "," << rtreeId2[0].second << std::endl;
                auto coord = obj2.getShape();
                printPolygon(coord);

            if (obj1.getNetId() != obj2.getNetId())
                buildRelation(rtreeId[0].second, v.second);

            /*std::cout << "!!Overlap!!" << std::endl;
            if (boost::geometry::intersects(poly1, poly2)) {
                boost::geometry::intersection(poly1, poly2, output);
                BOOST_FOREACH(polygon_t const& p, output)
                {
                    std::cout << "\tarea:" << boost::geometry::area(p) << std::endl;
                    std::cout << " obj1 id: " << obj1.getId() << ", obj2 id: " << obj2.getId() << std::endl;
                }*/

//}

void Decompaction::printDrc()
{
    int count = 0;
    for (auto &&obj1 : m_objects)
    {
        for (auto &&obj2 : m_objects)
        {
            if (obj1.getNetId() == obj2.getNetId())
                continue;
            if (obj1.getType() == ObjectType::PIN && obj2.getType() == ObjectType::PIN && obj1.getInstId() == obj2.getInstId())
                continue;
            std::deque<polygon_t> output;
            polygon_t blue = obj1.getPoly();
            polygon_t green = obj2.getPoly();

            //bool b = boost::geometry::intersects(blue, green);
            //std::cout << b << std::endl;
            /*
            int i = 0;
            BOOST_FOREACH(polygon_t const& p, output)
            {
                std::cout << i++ << ": " << boost::geometry::area(p) << std::endl;
            }
            */
            if (boost::geometry::intersects(blue, green))
            { //output.size() != 0) {

                boost::geometry::intersection(green, blue, output);
                BOOST_FOREACH (polygon_t const &p, output)
                {

                    if (boost::geometry::area(p) > 0.01)
                    {
                        m_db.addClearanceDrc(obj1, obj2);
                        count++;

                        std::cout << "----------CONFLICT---------- " << std::endl;
                        std::cout << "obj1 id: " << obj1.getId() << ", obj2 id: " << obj2.getId() << ", area:" << boost::geometry::area(p) << std::endl;
                        if (obj1.getType() == ObjectType::PIN)
                        {
                            auto compId = obj1.getCompId();
                            auto instId = obj1.getInstId();
                            auto padId = obj1.getDBId();
                            component comp = m_db.getComponent(compId);
                            instance inst = m_db.getInstance(instId);
                            auto &pad = comp.getPadstack(padId);
                            std::cout << "Component: " << comp.getName() << " Instance: " << inst.getName();
                            std::cout << " Pad: " << pad.getName() << std::endl;

                            printPolygon(obj1.getShape());
                        }
                        else if (obj1.getType() == ObjectType::SEGMENT)
                        {
                            auto dbId = obj1.getDBId();
                            points_2d pos = obj1.getPos();
                            std::cout << "segment: ";
                            for (auto &&p : pos)
                            {
                                std::cout << "(" << p.m_x << "," << p.m_y << ") ";
                            }
                            std::cout << std::endl;

                            printPolygon(obj1.getShape());
                        }
                        else if (obj1.getType() == ObjectType::VIA)
                        {
                            auto dbId = obj1.getDBId();
                            points_2d pos = obj1.getPos();
                            std::cout << "via: (" << pos[0].m_x << "," << pos[0].m_y << ")" << std::endl;

                            printPolygon(obj1.getShape());
                        }

                        if (obj2.getType() == ObjectType::PIN)
                        {
                            auto compId = obj2.getCompId();
                            auto instId = obj2.getInstId();
                            auto padId = obj2.getDBId();
                            component comp = m_db.getComponent(compId);
                            instance inst = m_db.getInstance(instId);
                            auto &pad = comp.getPadstack(padId);
                            std::cout << "Component: " << comp.getName() << " Instance: " << inst.getName();
                            std::cout << " Pad: " << pad.getName() << std::endl;
                            std::cout << " obj id: " << obj2.getId() << std::endl;

                            printPolygon(obj2.getShape());
                        }
                        else if (obj2.getType() == ObjectType::SEGMENT)
                        {
                            auto dbId = obj2.getDBId();
                            points_2d pos = obj2.getPos();
                            std::cout << "segment: ";
                            for (auto &&p : pos)
                            {
                                std::cout << "(" << p.m_x << "," << p.m_y << ") ";
                            }
                            std::cout << std::endl;

                            printPolygon(obj2.getShape());
                        }
                        else if (obj2.getType() == ObjectType::VIA)
                        {
                            auto dbId = obj2.getDBId();
                            points_2d pos = obj2.getPos();
                            std::cout << "via: (" << pos[0].m_x << "," << pos[0].m_y << ")" << std::endl;
                            printPolygon(obj2.getShape());
                        }
                    }
                }
            }
        }
    }

    std::cout << "###############SUMMARY#################" << std::endl;
    std::cout << "DRC count: " << count / 2 << std::endl;
}

int Decompaction::checkDRC()
{
    int count = 0;
    double clearance = m_db.getLargestClearance();
    for (auto &&obj1 : m_objects)
    {
        for (auto &&obj2 : m_objects)
        {
            if (obj1.getNetId() == obj2.getNetId())
                continue;
            if (obj1.getType() == ObjectType::PIN && obj2.getType() == ObjectType::PIN && obj1.getInstId() == obj2.getInstId())
                continue;

            multipoly poly1 = obj1.getMultipoly();
            multipoly poly2 = obj2.getMultipoly();
            auto dist = boost::geometry::distance(poly1, poly2);
            if (dist <= clearance)
            {
                m_db.addClearanceDrc(obj1, obj2);
                count++;

                std::cout << "----------CONFLICT---------- " << std::endl;
                std::cout << "obj1 id: " << obj1.getId() << ", obj2 id: " << obj2.getId() << ", distance:" << dist << std::endl;
                if (obj1.getType() == ObjectType::PIN)
                {
                    auto compId = obj1.getCompId();
                    auto instId = obj1.getInstId();
                    auto padId = obj1.getDBId();
                    component comp = m_db.getComponent(compId);
                    instance inst = m_db.getInstance(instId);
                    auto &pad = comp.getPadstack(padId);
                    std::cout << "Component: " << comp.getName() << " Instance: " << inst.getName();
                    std::cout << " Pad: " << pad.getName() << std::endl;

                    printPolygon(obj1.getShape());
                }
                else if (obj1.getType() == ObjectType::SEGMENT)
                {
                    auto dbId = obj1.getDBId();
                    points_2d pos = obj1.getPos();
                    std::cout << "segment: ";
                    for (auto &&p : pos)
                    {
                        std::cout << "(" << p.m_x << "," << p.m_y << ") ";
                    }
                    std::cout << std::endl;

                    printPolygon(obj1.getShape());
                }
                else if (obj1.getType() == ObjectType::VIA)
                {
                    auto dbId = obj1.getDBId();
                    points_2d pos = obj1.getPos();
                    std::cout << "via: (" << pos[0].m_x << "," << pos[0].m_y << ")" << std::endl;

                    printPolygon(obj1.getShape());
                }

                if (obj2.getType() == ObjectType::PIN)
                {
                    auto compId = obj2.getCompId();
                    auto instId = obj2.getInstId();
                    auto padId = obj2.getDBId();
                    component comp = m_db.getComponent(compId);
                    instance inst = m_db.getInstance(instId);
                    auto &pad = comp.getPadstack(padId);
                    std::cout << "Component: " << comp.getName() << " Instance: " << inst.getName();
                    std::cout << " Pad: " << pad.getName() << std::endl;
                    std::cout << " obj id: " << obj2.getId() << std::endl;

                    printPolygon(obj2.getShape());
                }
                else if (obj2.getType() == ObjectType::SEGMENT)
                {
                    auto dbId = obj2.getDBId();
                    points_2d pos = obj2.getPos();
                    std::cout << "segment: ";
                    for (auto &&p : pos)
                    {
                        std::cout << "(" << p.m_x << "," << p.m_y << ") ";
                    }
                    std::cout << std::endl;

                    printPolygon(obj2.getShape());
                }
                else if (obj2.getType() == ObjectType::VIA)
                {
                    auto dbId = obj2.getDBId();
                    points_2d pos = obj2.getPos();
                    std::cout << "via: (" << pos[0].m_x << "," << pos[0].m_y << ")" << std::endl;
                    printPolygon(obj2.getShape());
                }
            }
        }
    }

    std::cout << "###############SUMMARY#################" << std::endl;
    std::cout << "DRC count: " << count / 2 << std::endl;

    return count / 2;
}

void Decompaction::printObject()
{
    std::cout << std::endl;
    std::cout << "#####################################" << std::endl;
    std::cout << "###                               ###" << std::endl;
    std::cout << "###             OBJ               ###" << std::endl;
    std::cout << "###                               ###" << std::endl;
    std::cout << "#####################################" << std::endl;
    for (auto &&obj : m_objects)
    {
        obj.printObject();
        std::vector<std::pair<int, int>> ids = obj.getRtreeId();
        for (auto &&id : ids)
        {
            std::cout << "(" << id.first << "," << id.second << ") ";
        }
        std::cout << std::endl;
    }
}

void Decompaction::printObject(int &id)
{
    std::cout << "########## OBJECT " << id << " ############" << std::endl;
    auto &&obj = m_objects[id];
    obj.printObject();
    auto equs = obj.getEquations();
    for (auto &&equ : equs)
    {
        printInequalityEquation(equ);
    }
}

std::vector<double> Decompaction::lineEquation(point_2d &p1, point_2d &p2)
{
    double y1 = p1.m_y, y2 = p2.m_y, x1 = p1.m_x, x2 = p2.m_x;
    double slope = 0;
    if (x2 != x1)
        slope = (y2 - y1) / (x2 - x1);
    std::cout << "slope: " << slope << std::endl;
    double b = y1 - slope * x1;
    std::vector<double> equ;
    if (x1 == x2)
    {
        equ.push_back(0);
        equ.push_back(1);
        equ.push_back(-x1);
    }
    else
    {
        equ.push_back(1);
        equ.push_back(-slope);
        equ.push_back(-b);
    }

    return equ;
}

std::vector<std::vector<double>> Decompaction::inequalityLineEquation(point_2d &p1, point_2d &p2, point_2d &r1, point_2d &r2, point_2d &center)
{
    std::cout << "Center point: " << center << std::endl;
    std::vector<std::vector<double>> equs;
    std::vector<double> e;
    auto equ = lineEquation(p1, p2);
    printEquation(equ);

    double slope = -1 * equ[1];
    double value = equ[0] * center.m_y + equ[1] * center.m_x + equ[2];
    if (slope > 0)
    {
        if (value > 0)
            equ.push_back(1);
        else if (value < 0)
            equ.push_back(0);
    }
    else
    {
        if (value > 0)
            equ.push_back(1);
        else if (value < 0)
            equ.push_back(0);
    }

    e = equ;
    e[2] = equ[2] + equ[0] * r1.m_y + equ[1] * r1.m_x;
    equs.push_back(e);
    e[2] = equ[2] + equ[0] * r2.m_y + equ[1] * r2.m_x;
    equs.push_back(e);
    return equs;
}

std::vector<std::vector<double>> Decompaction::lineEquation(point_2d &p1, point_2d &p2, point_2d &r1, point_2d &r2)
{
    std::vector<std::vector<double>> equs;
    std::vector<double> e;
    auto equ = lineEquation(p1, p2);
    printEquation(equ);
    e = equ;
    e[2] = equ[2] + equ[0] * r1.m_y + equ[1] * r1.m_x;
    equs.push_back(e);
    e[2] = equ[2] + equ[0] * r2.m_y + equ[1] * r2.m_x;
    equs.push_back(e);
    return equs;
}

void Decompaction::printEquation(std::vector<double> &equ)
{
    if (equ[0] != 0)
        std::cout << equ[0] << "y ";
    if (equ[1] > 0)
        std::cout << "+ " << equ[1] << "x ";
    else if (equ[1] < 0)
        std::cout << equ[1] << "x ";

    if (equ[2] > 0)
        std::cout << "+ " << equ[2];
    else if (equ[2] < 0)
        std::cout << equ[2];
    std::cout << " > 0 " << std::endl;
}

//////////////////////
//  equ[0] y + equ[1] x + equ[2] equ[3]
//  equ[3]:0 denotes >=
//  equ[3]:1 denotes <=
std::vector<double> Decompaction::getInequalityEquation(std::vector<double> &equ, point_2d &center)
{
    std::vector<double> eq(equ);
    double slope = -1 * equ[1];
    double value = equ[0] * center.m_y + equ[1] * center.m_x + equ[2];
    if (slope > 0)
    {
        if (value > 0)
            eq.push_back(1);
        else if (value < 0)
            eq.push_back(0);
    }
    else
    {
        if (value > 0)
            eq.push_back(0);
        else if (value < 0)
            eq.push_back(1);
    }
    return eq;
}
void Decompaction::printInequalityEquation(std::vector<double> &equ, point_2d &center)
{
    std::cout << "center point : " << center << std::endl;
    double value = equ[0] * center.m_y + equ[1] * center.m_x + equ[2];
    double slope = -1 * equ[1];
    if (equ[0] != 0)
        std::cout << equ[0] << "y ";
    if (equ[1] > 0)
        std::cout << "+ " << equ[1] << "x ";
    else if (equ[1] < 0)
        std::cout << equ[1] << "x ";

    if (equ[2] > 0)
        std::cout << "+ " << equ[2];
    else if (equ[2] < 0)
        std::cout << equ[2];

    if (slope > 0)
    {
        if (value > 0)
            std::cout << " <= 0 " << std::endl;
        else if (value < 0)
            std::cout << " >= 0 " << std::endl;
    }
    else
    {
        if (value > 0)
            std::cout << " >= 0 " << std::endl;
        else if (value < 0)
            std::cout << " <= 0 " << std::endl;
    }
}

void Decompaction::printInequalityEquation(std::vector<double> &equ)
{

    double slope = -1 * equ[1];
    if (equ[0] != 0)
        std::cout << equ[0] << "y ";
    if (equ[1] > 0)
        std::cout << "+ " << equ[1] << "x ";
    else if (equ[1] < 0)
        std::cout << equ[1] << "x ";

    if (equ[2] > 0)
        std::cout << "+ " << equ[2];
    else if (equ[2] < 0)
        std::cout << equ[2];

    if (equ[3] == 0)
    {
        std::cout << " >= 0" << std::endl;
    }
    else if (equ[3] == 1)
    {
        std::cout << "<= 0" << std::endl;
    }
}

void Decompaction::printInequalityEquation(std::vector<std::vector<double>> &equs, point_2d &center)
{
    auto equ = equs[0];
    printInequalityEquation(equ, center);
    equ = equs[1];
    printInequalityEquation(equ, center);
}

void Decompaction::printEquation(std::vector<std::vector<double>> &equs)
{
    auto equ = equs[0];
    printEquation(equ);
    equ = equs[1];
    printEquation(equ);
}

void Decompaction::printPolygon(points_2d &coord)
{
    std::cout << "Polygon(";
    for (size_t i = 0; i < coord.size(); ++i)
    {

        std::cout << coord[i];
        if (i != 7)
            std::cout << ", ";
    }
    std::cout << ")" << std::endl;
}

void Decompaction::printPoint(point_2d &p)
{
    std::cout << "Point({" << p.m_x << "," << p.m_y << "})" << std::endl;
}

void Decompaction::printSegment(points_2d &line)
{
    std::cout << "Segment: (" << line[0].m_x << "," << line[0].m_y << ")"
              << "(" << line[1].m_x << "," << line[1].m_y << ")" << std::endl;
}

//////////////////////////////////////
//  equ[0] y + equ[1] x + equ[2] equ[3]
//  equ[3]:0 denotes >=
//  equ[3]:1 denotes <=
void Decompaction::writeLPfile(std::string &fileName)
{
    std::ofstream file;
    int slackWeight = 100000;
    auto instEqus = collectNonoverlapInstEqu();
    file.open(fileName);
    file << "Minimize" << std::endl;
    int count = 0, ini = 0;
    std::vector<bool> usedInst(m_db.getInstancesCount(), false);
    for (auto &&obj : m_objects)
    {
        if (obj.isLocked())
            continue;
        auto &equs = obj.getEquations();
        if (equs.empty())
            continue;
        if (obj.getType() == ObjectType::PIN)
        {
            //continue;
            int instId = obj.getInstId();
            if (usedInst[instId])
                continue;
            usedInst[instId] = true;
            if (ini == 0)
            {
                file << "xti_" << instId << " + yti_" << instId;
                ++ini;
            }
            else

                file << " + xti_" << instId << " + yti_" << instId;
            for (auto &&equ : equs)
            {
                file << " + " << slackWeight << " s_" << count;
                ++count;
            }
        }
        else
        {
            int objId = obj.getId();
            if (ini == 0)
            {
                file << "xt_" << objId << " + yt_" << objId;
                ++ini;
            }
            else
                file << " + xt_" << objId << " + yt_" << objId;
            for (auto &&equ : equs)
            {
                file << " + " << slackWeight << " s_" << count;
                ++count;
            }
        }
    }

    count = 0;
    file << std::endl;
    file << std::endl;
    file << "Subject To" << std::endl;
    for (auto &&obj : m_objects)
    {
        if (obj.isLocked())
            continue;
        auto &equs = obj.getEquations();
        if (equs.empty())
            continue;
        if (obj.getType() == ObjectType::PIN)
        {
            //continue;
            int instId = obj.getInstId();
            for (auto &&equ : equs)
            {
                file << equ[0] << " yi_" << instId << " + " << equ[1] << " xi_" << instId << " ";
                if (equ[3] == 0)
                    file << " + s_" << count;
                else if (equ[3] == 1)
                    file << " - s_" << count;

                if (equ[3] == 0)
                    file << " >= ";
                else if (equ[3] == 1)
                    file << " <= ";
                double value = -1 * equ[2];
                file << value;

                file << std::endl;
                ++count;
            }

            auto &&inst = m_db.getInstance(instId);
            double x = inst.getX(), y = inst.getY();
            file << "xi_" << instId << " - "
                 << "xti_" << instId << " <= " << x << std::endl;
            file << "xi_" << instId << " + "
                 << "xti_" << instId << " >= " << x << std::endl;
            file << "yi_" << instId << " - "
                 << "yti_" << instId << " <= " << y << std::endl;
            file << "yi_" << instId << " + "
                 << "yti_" << instId << " >= " << y << std::endl;
        }
        else
        {

            int objId = obj.getId();
            for (auto &&equ : equs)
            {
                file << equ[0] << " y_" << objId << " + " << equ[1] << " x_" << objId << " ";
                if (equ[3] == 0)
                    file << " + s_" << count;
                else if (equ[3] == 1)
                    file << " - s_" << count;
                if (equ[3] == 0)
                    file << " >= ";
                else if (equ[3] == 1)
                    file << " <= ";
                double value = -1 * equ[2];
                file << value;

                file << std::endl;
                ++count;
            }

            auto &&center = obj.getCenterPos();
            file << "x_" << objId << " - "
                 << "xt_" << objId << " <= " << center.m_x << std::endl;
            file << "x_" << objId << " + "
                 << "xt_" << objId << " >= " << center.m_x << std::endl;
            file << "y_" << objId << " - "
                 << "yt_" << objId << " <= " << center.m_y << std::endl;
            file << "y_" << objId << " + "
                 << "yt_" << objId << " >= " << center.m_y << std::endl;
        }
    }

    /*for (const auto &str : instEqus)
    {
        file << str << std::endl;
    }*/

    file << std::endl;
    file << "Bounds" << std::endl;

    count = 0;
    for (auto &&obj : m_objects)
    {
        if (obj.isLocked())
            continue;
        auto &equs = obj.getEquations();
        if (equs.empty())
            continue;
        if (obj.getType() == ObjectType::PIN)
        {
            continue;
            int instId = obj.getInstId();
            file << "xi_" << instId << " >= 0" << std::endl;
            // file << "xt_" << objId << " <= 1" << std::endl;
            file << "yi_" << instId << " >= 0" << std::endl;
        }
        else
        {

            int objId = obj.getId();

            file << "x_" << objId << " >= 0" << std::endl;
            //file << "xt_" << objId << " <= 1" << std::endl;
            file << "y_" << objId << " >= 0" << std::endl;
            //file << "yt_" << objId << " <= 1" << std::endl;
        }

        for (auto &&equ : equs)
        {
            file << "s_" << count << " >= 0" << std::endl;
            ++count;
        }
    }

    file << std::endl;
    /*file << "Binary" << std::endl;

    for (int i = 0; i < m_db.getInstancesCount(); ++i)
    {
        for (int j = i + 1; j < m_db.getInstancesCount(); ++j)
        {
            file << "p" << i << "_" << j << std::endl;
            file << "q" << i << "_" << j << std::endl;
        }
    }*/

    file << "End" << std::endl;

    file.close();
}

void Decompaction::writeLPInstFile(std::string &fileName)
{
    std::ofstream file;

    auto instEqus = collectNonoverlapInstEqu();
    file.open(fileName);
    file << "Minimize" << std::endl;
    int ini = 0;

    for (int instId = 0; instId < m_db.getInstancesCount(); ++instId)
    {
        if (ini == 0)
        {
            file << "xti_" << instId << " + yti_" << instId;
            ++ini;
        }
        else

            file << " + xti_" << instId << " + yti_" << instId;
    }

    file << std::endl;
    file << std::endl;
    file << "Subject To" << std::endl;
    for (int instId = 0; instId < m_db.getInstancesCount(); ++instId)
    {
        auto &&inst = m_db.getInstance(instId);
        double x = inst.getX(), y = inst.getY();
        file << "xi_" << instId << " - "
             << "xti_" << instId << " <= " << x << std::endl;
        file << "xi_" << instId << " + "
             << "xti_" << instId << " >= " << x << std::endl;
        file << "yi_" << instId << " - "
             << "yti_" << instId << " <= " << y << std::endl;
        file << "yi_" << instId << " + "
             << "yti_" << instId << " >= " << y << std::endl;
    }

    for (const auto &str : instEqus)
    {
        file << str << std::endl;
    }

    file << std::endl;
    file << "Bounds" << std::endl;

    for (int instId = 0; instId < m_db.getInstancesCount(); ++instId)
    {
        file << "xi_" << instId << " >= 0" << std::endl;
        // file << "xt_" << objId << " <= 1" << std::endl;
        file << "yi_" << instId << " >= 0" << std::endl;
    }

    file << std::endl;
    file << "Binary" << std::endl;

    for (int i = 0; i < m_db.getInstancesCount(); ++i)
    {
        for (int j = i + 1; j < m_db.getInstancesCount(); ++j)
        {
            file << "p" << i << "_" << j << std::endl;
            file << "q" << i << "_" << j << std::endl;
        }
    }

    file << "End" << std::endl;

    file.close();
}

void Decompaction::readLPSolution(std::string &fileName)
{
    m_instDiffPos.clear();
    m_instDiffPos.resize(m_db.getInstancesCount());
    for (auto &&instDiff : m_instDiffPos)
    {
        instDiff.m_x = 0;
        instDiff.m_y = 0;
    }
    std::ifstream file(fileName);
    std::string line;
    std::getline(file, line); //objective function
    while (std::getline(file, line))
    {
        std::istringstream iss(line);
        std::string objNo;
        int objId;
        double coor;
        iss >> objNo >> coor;
        std::cout << objNo << " " << coor;
        if (objNo[0] == 's')
        {
        }
        else if (objNo[0] == 'y')
        {
            if (objNo[1] == 't' && objNo[2] == 'i')
            {
            }
            else if (objNo[1] == '_')
            {
                objId = std::stoi(objNo.substr(2));
                updateValue(objId, "y", coor, ObjectType::NONE);
            }
            else if (objNo[1] == 'i')
            {
                objId = std::stoi(objNo.substr(3));
                updateValue(objId, "y", coor, ObjectType::PIN);
            }
        }
        else if (objNo[0] == 'x')
        {
            if (objNo[1] == 'i')
            {
                objId = std::stoi(objNo.substr(3));
                updateValue(objId, "x", coor, ObjectType::PIN);
            }
            else if (objNo[1] == '_')
            {
                objId = std::stoi(objNo.substr(2));
                updateValue(objId, "x", coor, ObjectType::NONE);
            }
            else if (objNo[1] == 't' && objNo[2] == 'i')
            {
            }
        }
        else if (objNo[0] == 'w')
        {
            objId = std::stoi(objNo.substr(4));
            if (objNo[2] == 'r')
            {
                updateExtraSpace(objId, "right", coor);
            }
            else if (objNo[2] == 'l')
            {
                updateExtraSpace(objId, "left", coor);
            }
        }
        //std::cout << objId << std::endl;
    }
}

void Decompaction::updateValue(int &objId, std::string type, double &coor, ObjectType otype)
{
    if (otype == ObjectType::PIN)
    {
        auto &&inst = m_db.getInstance(objId);
        if (inst.isLocked())
            return;

        if (type == "x")
        {
            auto instX = inst.getX();
            m_instDiffPos[objId].m_x = coor - instX;
            inst.setX(coor);
        }
        else if (type == "y")
        {
            auto instY = inst.getY();
            m_instDiffPos[objId].m_y = coor - instY;
            inst.setY(coor);
        }
        return;
        int i;
        for (i = 0; i < m_objects.size(); ++i)
        {
            auto &&obj = m_objects[i];
            if (obj.getType() == ObjectType::PIN)
            {
                int instId = obj.getInstId();
                if (instId == objId)
                    break;
            }
        }
        auto &&obj = m_objects[i];
        if (type == "x")
        {
            obj.setX(coor);
        }
        else if (type == "y")
        {
            obj.setY(coor);
        }
        return;
    }
    auto &&obj = m_objects[objId];
    auto &&objType = obj.getType();
    auto pos = obj.getPos();
    double diff = 0;
    if (objType == ObjectType::SEGMENT)
    {

        if (type == "x")
        {
            double x = obj.getX();
            diff = coor - x;
            pos[0].m_x = pos[0].m_x + diff;
            pos[1].m_x = pos[1].m_x + diff;
        }
        else if (type == "y")
        {
            double y = obj.getY();
            diff = coor - y;
            pos[0].m_y = pos[0].m_y + diff;
            pos[1].m_y = pos[1].m_y + diff;
        }
    }
    else if (objType == ObjectType::VIA)
    {
        if (type == "x")
        {
            double x = obj.getX();
            diff = coor - x;
            pos[0].m_x = coor;
        }
        else if (type == "y")
        {
            double y = obj.getY();
            diff = coor - y;
            pos[0].m_y = coor;
        }
    }
    //std::cout << "diff: " << diff << std::endl;
    obj.updateShape(type, diff);
    obj.setPos(pos);
}

void Decompaction::updateExtraSpace(int &objId, std::string type, double &width)
{
    auto &&obj = m_objects[objId];
    auto &&objType = obj.getType();
    if (objType != ObjectType::SEGMENT)
        return;
    obj.setExtraSpace(width, type);
}

void Decompaction::updateDatabase()
{
    for (auto &&obj : m_objects)
    {
        auto &type = obj.getType();
        if (type == ObjectType::PIN)
            continue;
        else if (type == ObjectType::SEGMENT)
        {
            int netId = obj.getNetId();
            auto &&net = m_db.getNet(netId);
            auto &&seg = net.getSegment(obj.getDBId());
            auto pos = obj.getPos();
            seg.setPosition(pos);
        }
        else if (type == ObjectType::VIA)
        {
            int netId = obj.getNetId();
            auto &&net = m_db.getNet(netId);
            auto &&via = net.getVia(obj.getDBId());
            auto pos = obj.getPos();
            via.setPosition(pos[0]);
        }
    }

    /*for (int i = 0; i < m_instDiffPos.size(); ++i)
    {
        auto &&diffPos = m_instDiffPos[i];
        auto &&inst = m_db.getInstance(i);

        auto instX = inst.getX(), instY = inst.getY();
        double x = instX + diffPos.m_x, y = instY + diffPos.m_y;
        inst.setX(x);
        inst.setY(y);

        std::cout << "i: " << i << " (" << x << "," << y << ")" << std::endl;
    }*/
}

void Decompaction::clearEquations()
{
    for (auto &&obj : m_objects)
    {
        obj.clearEquation();
    }
}

void Decompaction::updatePinsShapeAndPosition()
{

    for (auto &&obj : m_objects)
    {
        auto &&objType = obj.getType();
        if (objType != ObjectType::PIN)
            continue;

        int instId = obj.getInstId();
        auto instDiffPos = m_instDiffPos[instId];
        auto pinPos = obj.getPos();

        pinPos[0].m_x = pinPos[0].m_x + instDiffPos.m_x;
        pinPos[0].m_y = pinPos[0].m_y + instDiffPos.m_y;
        obj.updateShape("x", instDiffPos.m_x);
        obj.updateShape("y", instDiffPos.m_x);
    }
}

double Decompaction::maxLength()
{
    std::vector<net> nets = m_db.getNets();
    double maxLength = 0;
    std::string name;
    for (auto &&net : nets)
    {
        if (!net.isBus())
            continue;
        double netLength = 0;
        std::vector<Segment> segments = net.getSegments();
        for (auto &&seg : segments)
        {
            netLength += seg.getLength();
        }
        if (netLength > maxLength)
        {
            name = net.getName();
            maxLength = netLength;
        }
    }

    std::cout << "Maximum length in bus is: " << name << " with length: " << maxLength << std::endl;
    return maxLength;
}

void Decompaction::writeLPfileForBus(std::string &fileName)
{

    std::vector<std::vector<int>> netToSegment;
    netToSegment.resize(m_db.getNumNets());
    for (auto &&obj : m_objects)
    {
        if (obj.getType() != ObjectType::SEGMENT)
            continue;
        if (!obj.isBus())
            continue;
        int netId = obj.getNetId();
        int objId = obj.getId();
        netToSegment[netId].push_back(objId);
    }

    std::ofstream file;
    int slackWeight = 100000;
    file.open(fileName);
    file << "Minimize" << std::endl;
    int count = 0, ini = 0;
    double segWidth = 0.127;
    double clearance = 3 * m_db.getLargestClearance();
    double widthReci = 1 / (segWidth + clearance);
    double maxL = maxLength();

    for (auto &&net : netToSegment)
    {
        /*if (net.size() == 0)
            continue;
        if (ini == 0)
        {
            file << maxL;
            ++ini;
        }
        else
        {
            file << " + " << maxL;
        }*/

        for (auto &&seg : net)
        {
            auto &&obj = m_objects[seg];
            int objId = obj.getId();
            auto &equs = obj.getEquations();
            bool leftWidth = false, rightWidth = false;
            for (auto &equ : equs)
            {
                if (equ[4] == 1)
                    rightWidth = true;
                else if (equ[4] == 0)
                    leftWidth = true;
            }

            if (rightWidth && leftWidth)
            {
                file << " + w_r_" << objId << " + w_l_" << objId;
            }
            else if (rightWidth && !leftWidth)
            {
                file << " + w_r_" << objId;
            }
            else if (!rightWidth && leftWidth)
            {
                file << " + w_l_" << objId;
            }
            /*else
            {
                file << " - 1";
            }*/

            file << " + xt_" << objId << " + yt_" << objId;
            //auto &equs = obj.getEquations();
            for (auto &&equ : equs)
            {
                file << " + " << slackWeight << " s_" << count;
                ++count;
            }
        }
    }

    count = 0;
    file << std::endl;
    file << std::endl;
    file << "Subject To" << std::endl;
    for (auto &&net : netToSegment)
    {
        if (net.size() == 0)
            continue;
        for (auto &&seg : net)
        {
            auto &&obj = m_objects[seg];

            auto &equs = obj.getEquations();

            int objId = obj.getId();
            for (auto &&equ : equs)
            {
                if (equ[4] == -1)
                {
                    double value = 0;
                    file << equ[0] << " y_" << objId << " + " << equ[1] << " x_" << objId << " ";

                    if (equ[3] == 0)
                        file << " + s_" << count;
                    else if (equ[3] == 1)
                        file << " - s_" << count;

                    if (equ[3] == 0)
                    {

                        file << " >= ";
                    }
                    else if (equ[3] == 1)
                    {

                        file << " <= ";
                    }
                    value += -1 * equ[2];
                    file << value;

                    file << std::endl;

                    continue;
                }
                double value = 0;

                file << equ[0] << " y_" << objId << " + " << equ[1] << " x_" << objId << " ";

                if (equ[3] == 0)
                    file << " + s_" << count;
                else if (equ[3] == 1)
                    file << " - s_" << count;
                if (equ[4] == 1)
                    file << " + w_r_" << objId;
                else if (equ[4] == 0)
                    file << " - w_l_" << objId;
                if (equ[3] == 0)
                {
                    file << " + 1000 B_" << objId;
                    file << " >= ";
                }
                else if (equ[3] == 1)
                {
                    file << " - 1000 B_" << objId;
                    file << " <= ";
                }
                value += -1 * equ[2];
                file << value;

                file << std::endl;

                value = 0;

                file << equ[0] << " y_" << objId << " + " << equ[1] << " x_" << objId << " ";

                if (equ[3] == 0)
                    file << " + s_" << count;
                else if (equ[3] == 1)
                    file << " - s_" << count;
                if (equ[4] == 1)
                    file << " + w_r_" << objId;
                else if (equ[4] == 0)
                    file << " - w_l_" << objId;
                if (equ[3] == 0)
                {
                    file << " - 1000 B_" << objId;
                    file << " >= ";
                    if (equ[4] == 1 || equ[4] == 0)
                    {
                        value += m_db.getLargestClearance();
                        value -= 1000;
                    }
                }
                else if (equ[3] == 1)
                {
                    file << " + 1000 B_" << objId;
                    file << " <= ";
                    if (equ[4] == 1 || equ[4] == 0)
                    {
                        value -= m_db.getLargestClearance();
                        value += 1000;
                    }
                }
                value += -1 * equ[2];
                file << value;

                file << std::endl;

                ++count;
            }

            auto &&center = obj.getCenterPos();
            file << "x_" << objId << " - "
                 << "xt_" << objId << " <= " << center.m_x << std::endl;
            file << "x_" << objId << " + "
                 << "xt_" << objId << " >= " << center.m_x << std::endl;
            file << "y_" << objId << " - "
                 << "yt_" << objId << " <= " << center.m_y << std::endl;
            file << "y_" << objId << " + "
                 << "yt_" << objId << " >= " << center.m_y << std::endl;
        }
    }

    for (auto &&net : netToSegment)
    {
        if (net.size() == 0)
            continue;
        //file << maxL;
        bool noDeltaWidth = true;
        double totalLength = 0;
        for (auto &&seg : net)
        {
            auto &&obj = m_objects[seg];
            int objId = obj.getId();
            auto &equs = obj.getEquations();
            bool leftWidth = false, rightWidth = false;
            double segLength = obj.getLength();
            totalLength += segLength;
            double coeff = segLength * widthReci;
            for (auto &equ : equs)
            {
                if (equ[4] == 1)
                {
                    rightWidth = true;
                    noDeltaWidth = false;
                }
                else if (equ[4] == 0)
                {
                    leftWidth = true;
                    noDeltaWidth = false;
                }
            }

            if (rightWidth && leftWidth)
            {
                file << " + " << coeff << " w_r_" << objId << " + " << coeff << " w_l_" << objId;
                //file << " - 1 - 7.874 w_r_" << objId << " - 7.874 w_l_" << objId;
            }
            else if (rightWidth && !leftWidth)
            {
                file << " + " << coeff << " w_r_" << objId;
                //file << " - 1 - 7.874 w_r_" << objId;
            }
            else if (!rightWidth && leftWidth)
            {
                file << " + " << coeff << " w_l_" << objId;
                //file << " - 1 - 7.874 w_l_" << objId;
            }
            else
            {

                //file << " - 1";
            }
        }
        double remain = maxL - totalLength;
        if (!noDeltaWidth)
            file << " >= " << remain << std::endl;
    }

    for (auto &&net : netToSegment)
    {
        if (net.size() == 0)
            continue;
        //file << maxL;
        bool noDeltaWidth = true;
        double totalLength = 0;
        double minWidth = 0.327;
        double bigNumber = 1000;
        for (auto &&seg : net)
        {
            auto &&obj = m_objects[seg];
            int objId = obj.getId();
            auto &equs = obj.getEquations();
            bool leftWidth = false, rightWidth = false;

            for (auto &equ : equs)
            {
                if (equ[4] == 1)
                {
                    rightWidth = true;
                    noDeltaWidth = false;
                }
                else if (equ[4] == 0)
                {
                    leftWidth = true;
                    noDeltaWidth = false;
                }
            }

            if (rightWidth && leftWidth)
            {
                file << " w_r_" << objId << " + "
                     << " w_l_" << objId << " - " << bigNumber << " B_" << objId << " >= " << (minWidth - bigNumber) << std::endl;

                file << bigNumber << " B_" << objId << " + w_r_" << objId << " + "
                     << " w_l_" << objId << " >= 0" << std::endl;

                file << -bigNumber << " B_" << objId << " + w_r_" << objId << " + "
                     << " w_l_" << objId << " <= 0" << std::endl;
            }
            else if (rightWidth && !leftWidth)
            {
                file << " w_r_" << objId << " - " << bigNumber << " B_" << objId << " >= " << (minWidth - bigNumber) << std::endl;
                file << bigNumber << " B_" << objId << " + w_r_" << objId << " >= 0" << std::endl;
                file << -bigNumber << " B_" << objId << " + w_r_" << objId << " <= 0" << std::endl;
            }
            else if (!rightWidth && leftWidth)
            {
                file << " w_l_" << objId << " - " << bigNumber << " B_" << objId << " >= " << (minWidth - bigNumber) << std::endl;
                file << bigNumber << " B_" << objId << " + w_l_" << objId << " >= 0" << std::endl;
                file << -bigNumber << " B_" << objId << " + w_l_" << objId << " <= 0" << std::endl;
            }
            else
            {
            }
        }
    }

    file << std::endl;
    file << "Bounds" << std::endl;

    count = 0;

    for (auto &&net : netToSegment)
    {
        if (net.size() == 0)
            continue;
        for (auto &&seg : net)
        {
            auto &&obj = m_objects[seg];
            int objId = obj.getId();
            file << "x_" << objId << " >= 0" << std::endl;
            //file << "xt_" << objId << " <= 3" << std::endl;
            file << "y_" << objId << " >= 0" << std::endl;
            //file << "w_" << objId << " >= 0" << std::endl;
            //file << "w_" << objId << " <= 10" << std::endl;
            //file << "yt_" << objId << " <= 3" << std::endl;

            auto &equs = obj.getEquations();
            for (auto &equ : equs)
            {
                if (equ[4] == 1)
                {
                    file << "w_r_" << objId << " >= 0" << std::endl;
                    //   file << "w_r_" << objId << " <= 10" << std::endl;
                }
                else if (equ[4] == 0)
                {
                    file << "w_l_" << objId << " >= 0" << std::endl;
                    //    file << "w_l_" << objId << " <= 10" << std::endl;
                }
            }

            for (auto &&equ : equs)
            {
                file << "s_" << count << " >= 0" << std::endl;
                ++count;
            }
        }
    }

    file << std::endl;
    file << "Binary" << std::endl;
    for (auto &&net : netToSegment)
    {
        if (net.size() == 0)
            continue;
        for (auto &&seg : net)
        {
            auto &&obj = m_objects[seg];
            int objId = obj.getId();
            auto &equs = obj.getEquations();
            bool deltaWidth = false;
            for (auto &equ : equs)
            {
                if (equ[4] == 1 || equ[4] == 0)
                {
                    deltaWidth = true;
                }
            }
            if (deltaWidth)
                file << "B_" << objId << std::endl;
        }
    }

    file << "End" << std::endl;

    file.close();
}

void Decompaction::addWidthToBusSegmentEquation()
{
    for (auto &&obj : m_objects)
    {
        if (!obj.isBus())
            continue;
        if (obj.getType() != ObjectType::SEGMENT)
            continue;
        auto &equs = obj.getEquations();
        int angle = obj.getAngle();
        int id = obj.getId();
        auto &points = obj.getPos();
        double length = obj.getLength();

        std::cout << "obj id: " << id << " angle: " << angle << " length: " << length << " slope: " << std::endl;
        for (auto &&equ : equs)
        {
            if (length < 3)
            {
                equ.push_back(-1);
                continue;
            }
            double slope;
            if (equ[0] == 0)
                slope = 10000;
            else
                slope = (equ[1]) / (equ[0]);

            std::cout << "\t" << equ[0] << " " << equ[1] << " " << slope << std::endl;
            if (angle == 0 && slope == 0)
            {
                if (equ[3] == 1)
                    equ.push_back(1); //rigth width
                else
                    equ.push_back(0); //left width
            }
            else if (angle == 45 && slope < 0 && slope > -2)
            {
                if (equ[3] == 1)
                    equ.push_back(1);
                else
                    equ.push_back(0);
            }
            else if (angle == 90 && slope == 10000)
            {
                if (equ[3] == 1)
                    equ.push_back(1);
                else
                    equ.push_back(0);
            }
            else if (angle == 135 && slope > 0 && slope < 2)
            {
                if (equ[3] == 1)
                    equ.push_back(1);
                else
                    equ.push_back(0);
            }
            else
            {
                equ.push_back(-1);
            }
        }
    }
}

void Decompaction::getSnaking()
{
    //////////////////TEST///////////////
    double clearance = m_db.getLargestClearance();
    double width = 0.127;
    point_2d ll(123.26, 138.05);
    point_2d rr(132.11, 139);
    /*point_2d ll(0, 0);
    point_2d rr(10, 3);*/
    points_2d bbox, point;
    bbox.push_back(ll);
    bbox.push_back(rr);
    point_2d start(123.26, 138.05);
    point_2d end(132.11, 138.05);
    /*point_2d start(0, 0);
    point_2d end(10, 3);*/
    point.push_back(start);
    point.push_back(end);
    int netId = 2;
    Snaking snake(bbox, clearance, width, point, netId);
    auto pattern = snake.getSnakingPattern();
    auto &net = m_db.getNet(2);
    for (auto p : pattern)
    {
        std::cout << "Add segment: " << p.getId() << std::endl;
        net.addSegment(p);
    }
}

void Decompaction::getBoundingBox()
{
    std::pair<double, double> width = std::make_pair<double, double>(1.5, 2.5);
    int angle = 45;
    auto start = point_2d{1, 1};
    auto end = point_2d{7, 7};
    auto bboxObj = BoundingBox{width, angle, start, end};
    auto bbox = bboxObj.getBBox();
    std::cout << "ll: " << bbox[0] << " rr: " << bbox[1] << std::endl;

    bboxObj.printBBox();
    bboxObj.printBBox(-angle);

    auto rotatedStart = bboxObj.getPoint(start, angle);
    auto rotatedEnd = bboxObj.getPoint(end, angle);
    std::cout << "start:" << rotatedStart << std::endl;
    std::cout << "end:" << rotatedEnd << std::endl;
}

void Decompaction::testBBoxSnaking()
{
    int angle = 45;
    double segWidth = 0.127;
    double clearance = m_db.getLargestClearance();
    auto start = point_2d{119.81, 133.95};
    auto end = point_2d{123.61, 137.75};
    std::pair<double, double> width = std::make_pair<double, double>(0.13, 1.5);
    auto bboxObj = BoundingBox{width, angle, start, end};
    auto llrr = bboxObj.getBBox();

    points_2d bbox, point;
    bbox.push_back(llrr[0]);
    bbox.push_back(llrr[2]);
    point.emplace_back(bboxObj.getPoint(start, angle));
    point.emplace_back(bboxObj.getPoint(end, angle));
    int netId = 1;
    Snaking snake(bbox, clearance, segWidth, point, netId);
    auto pattern = snake.getSnakingPattern(-angle);
    auto &net = m_db.getNet(1);
    for (auto p : pattern)
    {
        std::cout << "Add segment: " << p.getId() << std::endl;
        net.addSegment(p);
    }
}

/////////////
// angle  | start end point
// 0      | start.m_x < end.m_x
// 45     | start.m_x < end.m_y
// 90     | start.m_y < end.m_y
// 135    | start.m_y < end.m_y
/////////////
void Decompaction::addSnakingPatterns()
{
    double segWidth = 0.127;
    for (auto &&obj : m_objects)
    {
        if (obj.getType() != ObjectType::SEGMENT)
            continue;
        auto space = obj.getExtraSpace();
        if (space.first == 0 && space.second == 0)
            continue;

        //space.first += 0.5 * segWidth;
        //space.second += 0.5 * segWidth;
        //space.first -= m_db.getLargestClearance();
        //space.second -= m_db.getLargestClearance();
        int angle = obj.getAngle();
        auto p = obj.getPos();
        point_2d start, end;
        points_2d startEnd, llrtPt;
        bool change = false;
        if (angle == 0 || angle == 45)
        {
            if (p[0].m_x > p[1].m_x)
                change = true;
            start = (p[0].m_x < p[1].m_x) ? p[0] : p[1];
            end = (p[0].m_x < p[1].m_x) ? p[1] : p[0];
            double temp = space.second;
            space.second = space.first;
            space.first = temp;
        }
        else if (angle == 90 || angle == 135)
        {
            if (p[0].m_y > p[1].m_y)
                change = true;
            start = (p[0].m_y < p[1].m_y) ? p[0] : p[1];
            end = (p[0].m_y < p[1].m_y) ? p[1] : p[0];
        }

        /*
        if (change)
        {
            start = p[1];
            end = p[0];
            double temp = space.second;
            space.second = space.first;
            space.first = temp;
        }
        else
        {
            start = p[0];
            end = p[1];
        }
        */

        auto bboxObj = BoundingBox{space, angle, start, end};
        auto bbox = bboxObj.getBBox();
        double clearance = m_db.getLargestClearance();
        startEnd.emplace_back(bboxObj.getPoint(start, angle));
        ;
        startEnd.emplace_back(bboxObj.getPoint(end, angle));
        ;
        llrtPt.push_back(bbox[0]);
        llrtPt.push_back(bbox[2]);

        int netId = obj.getNetId(), layerId = obj.getLayer(), segId = obj.getDBId();
        std::cout << "net id: " << netId << " objId: " << obj.getId() << std::endl;
        std::string layer = m_db.getLayerName(layerId);
        Snaking snake(llrtPt, clearance, segWidth, startEnd, netId, layer);
        auto pattern = snake.getSnakingPattern(-angle);

        auto &net = m_db.getNet(netId);
        auto &&seg = net.getSegment(segId);
        seg.setDisplay(false);
        int segCnt = net.getSegmentCount();
        for (auto p : pattern)
        {
            p.setId(p.getId() + segCnt);
            std::cout << "Add segment: " << p.getId() << std::endl;
            net.addSegment(p);
        }

        if (obj.isMoved())
        {
            std::cout << "MOVE" << std::endl;
            points_2d startSegPos, endSegPos;
            points_2d prePos = obj.getPreviousPosition();
            if (change)
            {
                std::cout << "change\t";
                startSegPos.emplace_back(prePos[1]);
                startSegPos.emplace_back(start);
                endSegPos.emplace_back(prePos[0]);
                endSegPos.emplace_back(end);
                std::cout << "start: " << prePos[1] << " " << start << "   end: " << prePos[0] << " " << end << std::endl;
            }
            else
            {
                startSegPos.emplace_back(prePos[0]);
                startSegPos.emplace_back(start);
                endSegPos.emplace_back(prePos[1]);
                endSegPos.emplace_back(end);

                std::cout << "start: " << prePos[0] << " " << start << "   end: " << prePos[1] << " " << end << std::endl;
            }

            segCnt = net.getSegmentCount();
            Segment startSeg(segCnt + 1, netId, segWidth, layer);
            startSeg.setPosition(startSegPos);
            Segment endSeg(segCnt + 2, netId, segWidth, layer);
            endSeg.setPosition(endSegPos);
            net.addSegment(startSeg);
            net.addSegment(endSeg);
        }
    }
}

double Decompaction::getNetLength(int &netId)
{
    auto &&net = m_db.getNet(netId);
    if (!net.isBus())
        return -1;
    double netLength = 0;
    std::vector<Segment> segments = net.getSegments();
    for (auto &&seg : segments)
    {
        netLength += seg.getLength();
    }

    return netLength;
}

void Decompaction::printAllNetLength()
{
    std::cout << "############### NET LENGTH#############" << std::endl;
    int numNet = m_db.getNumNets();
    for (int i = 0; i < numNet; ++i)
    {
        double length = getNetLength(i);
        std::cout << "Net id: " << i << ", length: " << length << std::endl;
    }
}

vector<string> Decompaction::collectNonoverlapInstEqu()
{
    double minX, minY, W, H;
    m_db.getBoardBoundaryByEdgeCuts(minX, W, minY, H);
    std::vector<instance> insts = m_db.getInstances();
    vector<string> equs;

    for (int i = 0; i < insts.size(); ++i)
    {
        for (int j = i + 1; j < insts.size(); ++j)
        {
            auto &inst1 = insts[i];
            auto &inst2 = insts[j];

            auto angle1 = inst1.getAngle();
            auto compId1 = inst1.getComponentId();
            point_2d wh1;
            m_db.getCompBBox(compId1, &wh1);
            if (angle1 == 90 || angle1 == 270)
            {
                auto temp = wh1.m_x;
                wh1.m_x = wh1.m_y;
                wh1.m_y = temp;
            }
            auto angle2 = inst2.getAngle();
            auto compId2 = inst2.getComponentId();

            point_2d wh2;
            m_db.getCompBBox(compId2, &wh2);
            if (angle2 == 90 || angle2 == 270)
            {
                auto temp = wh2.m_x;
                wh2.m_x = wh2.m_y;
                wh2.m_y = temp;
            }

            double w = (wh1.m_x + wh2.m_x) / 2;
            double h = (wh1.m_y + wh2.m_y) / 2;

            string eq1 = "xi_" + to_string(j) + " - xi_" + to_string(i) + " + " + to_string(W) + " p" + to_string(i) + "_" + to_string(j) + " + " + to_string(W) + " q" + to_string(i) + "_" + to_string(j) + " >= " + to_string(w);
            double val = w - W;
            string eq2 = "xi_" + to_string(i) + " - xi_" + to_string(j) + " - " + to_string(W) + " p" + to_string(i) + "_" + to_string(j) + " + " + to_string(W) + " q" + to_string(i) + "_" + to_string(j) + " >= " + to_string(val);
            val = h - H;
            string eq3 = "yi_" + to_string(j) + " - yi_" + to_string(i) + " + " + to_string(H) + " p" + to_string(i) + "_" + to_string(j) + " - " + to_string(H) + " q" + to_string(i) + "_" + to_string(j) + " >= " + to_string(val);
            val = h - 2 * H;
            string eq4 = "yi_" + to_string(i) + " - yi_" + to_string(j) + " - " + to_string(H) + " p" + to_string(i) + "_" + to_string(j) + " - " + to_string(H) + " q" + to_string(i) + "_" + to_string(j) + " >= " + to_string(val);

            equs.emplace_back(eq1);
            equs.emplace_back(eq2);
            equs.emplace_back(eq3);
            equs.emplace_back(eq4);
        }
    }

    for (int i = 0; i < insts.size(); ++i)
    {
        auto &inst1 = insts[i];
        if (!inst1.isLocked())
            continue;
        double x = inst1.getX(), y = inst1.getY();
        string eq1 = "xi_" + to_string(i) + " = " + to_string(x);
        string eq2 = "yi_" + to_string(i) + " = " + to_string(y);
        equs.emplace_back(eq1);
        equs.emplace_back(eq2);
    }

    return equs;
}