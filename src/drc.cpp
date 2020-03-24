#include "drc.h"

bool comp(std::pair<int, point_2d> a, std::pair<int, point_2d> b)
{
    return a.second < b.second;
}

void Drc::createRTree()
{
    m_numLayer = m_db.getNumCopperLayers();
    m_rtrees.resize(m_numLayer);
    std::vector<net> nets = m_db.getNets();
    int id = 0;
    double clearance = (m_db.getLargestClearance() / 2);
    std::cout << "clearance: " << clearance << std::endl;
    for (auto &&net : nets)
    {
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

            box b = bg::return_envelope<box>(polygon);
            Object obj(ObjectType::SEGMENT, s.getId(), s.getNetId(), -1, -1);
            obj.setShape(coord);
            obj.setPoly(polygon);
            obj.setRelativeShape(coordR);
            obj.setBBox(b);
            obj.setPos(s.getPos());
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
            Object obj(ObjectType::VIA, v.getId(), v.getNetId(), -1, -1);

            obj.setShape(coord);
            obj.setPoly(polygon);
            obj.setRelativeShape(coordRe);
            obj.setBBox(b);
            obj.setPos(pos);
            int layerId;
            for (auto layer : layers)
            {
                layerId = m_db.getLayerId(layer);
                if (layerId >= m_numLayer)
                    layerId = m_numLayer - 1;

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
            Object obj(ObjectType::PIN, pad.getId(), net.getId(), compId, instId);
            obj.setPos(pinPos);
            obj.setRelativeShape(coordRe);
            obj.setShape(coord);
            obj.setPoly(polygon);
            obj.setBBox(b);
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
        points_2d coord = pinShapeToOctagon(pad.getSize(), pad.getPos(), 0.1, inst.getAngle(), pad.getAngle(), pad.getPadShape());
        auto coordRe = points_2d{};
        m_db.getPinShapeRelativeCoordsToModule(pad, inst, coord, &coordRe);
        auto pinPos = point_2d{};
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
        Object obj(ObjectType::PIN, pad.getId(), -1, compId, instId);
        obj.setPos(pinPos);
        obj.setShape(coord);
        obj.setRelativeShape(coordRe);
        obj.setPoly(polygon);
        obj.setBBox(b);
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
bool Drc::checkIntersection()
{
    
    

    return true;
}*/

std::vector<std::vector<double>> Drc::buildRelation(int &obj1Id, const int &obj2Id)
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

        std::cout << "shape1" << std::endl;
        for (auto &&p : proCoord1)
        {
            std::cout << p.m_x << " " << p.m_y << std::endl;
        }
        std::cout << "shape2" << std::endl;
        for (auto &&p : proCoord2)
        {
            std::cout << p.m_x << " " << p.m_y << std::endl;
        }

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
            std::cout << "A" << dist << std::endl;
        }
        else if (project2[0].second < project1[0].second && project1[0].second < project2[7].second &&
                 project2[7].second < project1[7].second)
        {

            dist = project2[7].second.getDistance(project2[7].second, project1[0].second);
            overlapResult[dist].push_back(std::make_pair(project1[0].first, project1[1].first));
            overlapResult[dist].push_back(std::make_pair(project2[6].first, project2[7].first));
            overlap = true;
            std::cout << "B" << dist << std::endl;
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
            std::cout << "C" << dist << std::endl;
        }
        else if (project2[7].second <= project1[0].second)
        {
            dist = project1[7].second.getDistance(project1[7].second, project2[0].second);
            nonoverlapResult[dist].push_back(std::make_pair(project1[0].first, project1[1].first));
            nonoverlapResult[dist].push_back(std::make_pair(project2[6].first, project2[7].first));
            overlap = false;
            std::cout << "D" << dist << std::endl;
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
                std::cout << "E" << dist1 << std::endl;
            }
            else
            {
                overlapResult[dist2].push_back(std::make_pair(project1[0].first, project1[1].first));
                overlapResult[dist2].push_back(std::make_pair(project2[6].first, project2[7].first));
                std::cout << "E" << dist2 << std::endl;
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
                std::cout << "F" << dist1 << std::endl;
            }
            else
            {
                overlapResult[dist2].push_back(std::make_pair(project1[6].first, project1[7].first));
                overlapResult[dist2].push_back(std::make_pair(project2[0].first, project2[1].first));
                std::cout << "F" << dist2 << std::endl;
            }
            overlap = true;
        }
        else
        {
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
        std::cout << "point: " << shape1[point[0].first] << ", " << shape1[point[0].second] << std::endl;
        std::cout << "relative point: " << reShape2[point[1].first] << ", " << reShape2[point[1].second] << std::endl;
        std::cout << "obj2 relative point: ";
        printPolygon(reShape2);
    }
    else
    {
        auto &&p = nonoverlapResult.rbegin();
        auto &&point = p->second;
        //equ = lineEquation(shape1[point[0].first], shape1[point[0].second], reShape2[point[1].first], reShape2[point[1].second]);
        equ = inequalityLineEquation(shape1[point[0].first], shape1[point[0].second], reShape2[point[1].first], reShape2[point[1].second], centerPos);
        std::cout << "point: " << shape1[point[0].first] << ", " << shape1[point[0].second] << std::endl;
        std::cout << "relative point: " << reShape2[point[1].first] << ", " << reShape2[point[1].second] << std::endl;
        std::cout << "obj2 relative point: ";
        printPolygon(reShape2);
    }
    return equ;
}

void Drc::traverseRTree()
{
    std::cout << "=========================" << std::endl;
    double MAX_DIST = 0.2;
    bg::model::point<double, 2, bg::cs::cartesian> point1;
    bg::model::point<double, 2, bg::cs::cartesian> point2;
    for (auto &&obj1 : m_objects)
    {
        /*auto &&obj = m_objects[437];
        if (obj != obj1)
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

points_2d Drc::projection(point_2d &center, points_2d &octagon, int degree)
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

void Drc::testProjection()
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
void Drc::checkClearance()
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

void Drc::printDrc()
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

void Drc::printObject()
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

void Drc::printObject(int &id)
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

std::vector<double> Drc::lineEquation(point_2d &p1, point_2d &p2)
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

std::vector<std::vector<double>> Drc::inequalityLineEquation(point_2d &p1, point_2d &p2, point_2d &r1, point_2d &r2, point_2d &center)
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

std::vector<std::vector<double>> Drc::lineEquation(point_2d &p1, point_2d &p2, point_2d &r1, point_2d &r2)
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

void Drc::printEquation(std::vector<double> &equ)
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
std::vector<double> Drc::getInequalityEquation(std::vector<double> &equ, point_2d &center)
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
void Drc::printInequalityEquation(std::vector<double> &equ, point_2d &center)
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

void Drc::printInequalityEquation(std::vector<double> &equ)
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

void Drc::printInequalityEquation(std::vector<std::vector<double>> &equs, point_2d &center)
{
    auto equ = equs[0];
    printInequalityEquation(equ, center);
    equ = equs[1];
    printInequalityEquation(equ, center);
}

void Drc::printEquation(std::vector<std::vector<double>> &equs)
{
    auto equ = equs[0];
    printEquation(equ);
    equ = equs[1];
    printEquation(equ);
}

void Drc::printPolygon(points_2d &coord)
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

void Drc::printPoint(point_2d &p)
{
    std::cout << "Point({" << p.m_x << "," << p.m_y << "})" << std::endl;
}

void Drc::printSegment(points_2d &line)
{
    std::cout << "Segment: (" << line[0].m_x << "," << line[0].m_y << ")"
              << "(" << line[1].m_x << "," << line[1].m_y << ")" << std::endl;
}

//////////////////////////////////////
//  equ[0] y + equ[1] x + equ[2] equ[3]
//  equ[3]:0 denotes >=
//  equ[3]:1 denotes <=
void Drc::writeLPfile(std::string &fileName)
{
    std::ofstream file;
    int slackWeight = 100000;
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

    file << "End" << std::endl;

    file.close();
}

void Drc::readLPSolution(std::string &fileName)
{
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
                objId = std::stoi(objNo.substr(4));
                updateValue(objId, "y", coor, ObjectType::PIN);
            }
            else if (objNo[1] == '_')
            {
                objId = std::stoi(objNo.substr(2));
                updateValue(objId, "y", coor, ObjectType::NONE);
            }
            else if (objNo[1] == 'i')
            {
            }
        }
        else if (objNo[0] == 'x')
        {
            if (objNo[1] == 'i')
            {
            }
            else if (objNo[1] == '_')
            {
                objId = std::stoi(objNo.substr(2));
                updateValue(objId, "x", coor, ObjectType::NONE);
            }
            else if (objNo[1] == 't' && objNo[2] == 'i')
            {
                objId = std::stoi(objNo.substr(4));
                updateValue(objId, "x", coor, ObjectType::PIN);
            }
        }

        //std::cout << objId << std::endl;
    }
}

void Drc::updateValue(int &objId, std::string type, double &coor, ObjectType otype)
{
    if (otype == ObjectType::PIN)
    {
        if (type == "x")
            m_instDiffPos[objId].m_x = coor;
        else if (type == "y")
            m_instDiffPos[objId].m_y = coor;
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

void Drc::updateDatabase()
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

    for (int i = 0; i < m_instDiffPos.size(); ++i)
    {
        auto &&diffPos = m_instDiffPos[i];
        auto &&inst = m_db.getInstance(i);

        auto instX = inst.getX(), instY = inst.getY();
        double x = instX + diffPos.m_x, y = instY + diffPos.m_y;
        inst.setX(x);
        inst.setY(y);
    }
}

void Drc::clearEquations()
{
    for (auto &&obj : m_objects)
    {
        obj.clearEquation();
    }
}

void Drc::updatePinsShapeAndPosition()
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

void Drc::maxLength()
{
    std::vector<net> nets = m_db.getNets();
    for (auto &&net : nets)
    {
        std::vector<Segment> segments = net.getSegments();
    }
}
