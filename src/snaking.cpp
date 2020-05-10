#include "snaking.h"

void Snaking::buildSnakingPattern()
{
    std::cout << "start: " << m_points[0] << ", end: " << m_points[1] << std::endl;
    std::cout << "bbox: " << m_bbox[0] << ", " << m_bbox[1] << std::endl;
    double cornerLength = 0.127;
    double width = getSegmentWidth() + 3 * getClearance();
    double bufferLength = width + 3 * cornerLength;
    double length = getBboxLength() - 2 * bufferLength;
    double cnt = length / width, intCnt, fracCnt;
    fracCnt = modf(cnt, &intCnt);
    std::cout << "vertical segment: " << cnt << " int Cnt: " << intCnt << std::endl;
    auto bbox = getBbox();
    //Segments Snaking;
    int id = 0, netId = 2, i = 0;
    points_2d positions, corPos1, corPos2;
    std::string layer = m_layer;
    point_2d intersectPt;

    bbox[0].m_x += bufferLength;
    bbox[1].m_x -= bufferLength;

    for (i = 0; i < intCnt - 1; ++i)
    {
        positions.clear();
        if (i == 0)
        {
            points_2d intersectCor;

            intersectCor.emplace_back(point_2d(bbox[0].m_x - width - 2 * cornerLength, m_bbox[0].m_y + cornerLength));
            intersectCor.emplace_back(point_2d(bbox[0].m_x - width - cornerLength, bbox[0].m_y));
            intersectPt = getIntersectionOfPointLine(m_points[0], intersectCor);

            points_2d startSegPos;
            startSegPos.emplace_back(point_2d(m_points[0].m_x, m_points[0].m_y));
            startSegPos.emplace_back(intersectPt);

            Segment seg(id++, netId, getSegmentWidth(), layer);
            seg.setPosition(startSegPos);
            m_snaking.push_back(seg);

            points_2d cor;
            cor.emplace_back(intersectPt);
            cor.emplace_back(point_2d(bbox[0].m_x - width - cornerLength, bbox[0].m_y));

            Segment corSeg(id++, netId, getSegmentWidth(), layer);
            corSeg.setPosition(cor);
            m_snaking.push_back(corSeg);

            //    positions.emplace_back(point_2d(bbox[0].m_x + bufferLength, m_points[0].m_y - cornerLength));
            //    positions.emplace_back(point_2d(bbox[0].m_x + bufferLength, bbox[0].m_y + cornerLength));

            /*positions.emplace_back(point_2d(m_points[0].m_x, m_points[0].m_y));
            positions.emplace_back(point_2d(bbox[0].m_x + bufferLength, bbox[0].m_y));

            bbox[0].m_x += bufferLength;
            bbox[1].m_x -= bufferLength;*/
        }
        else
        {
            //Vertical line
            positions.emplace_back(point_2d(bbox[0].m_x + (i - 1) * width, bbox[1].m_y - cornerLength));
            positions.emplace_back(point_2d(bbox[0].m_x + (i - 1) * width, bbox[0].m_y + cornerLength));
            Segment seg(id++, netId, getSegmentWidth(), layer);
            seg.setPosition(positions);
            m_snaking.push_back(seg);
            std::cout << "add segment " << std::endl;
        }

        positions.clear();
        corPos1.clear();
        corPos2.clear();
        if (i == intCnt - 2)
            break;
        if (i % 2 == 0 && i != 0)
        {
            // | |
            //  \ \ corner
            corPos1.emplace_back(point_2d(bbox[0].m_x + (i - 1) * width, bbox[0].m_y + cornerLength));
            corPos1.emplace_back(point_2d(bbox[0].m_x + (i - 1) * width + cornerLength, bbox[0].m_y));
            //horizontal
            positions.emplace_back(point_2d(bbox[0].m_x + (i - 1) * width + cornerLength, bbox[0].m_y));
            positions.emplace_back(point_2d(bbox[0].m_x + (i)*width - cornerLength, bbox[0].m_y));
            // | |
            // / /corner
            corPos2.emplace_back(point_2d(bbox[0].m_x + (i)*width - cornerLength, bbox[0].m_y));
            corPos2.emplace_back(point_2d(bbox[0].m_x + (i)*width, bbox[0].m_y + cornerLength));
        }
        else if (i == 0)
        {
            //horizontal
            positions.emplace_back(point_2d(bbox[0].m_x - width - cornerLength, bbox[0].m_y));
            positions.emplace_back(point_2d(bbox[0].m_x - cornerLength, bbox[0].m_y));
            // | |
            // / /corner
            corPos2.emplace_back(point_2d(bbox[0].m_x - cornerLength, bbox[0].m_y));
            corPos2.emplace_back(point_2d(bbox[0].m_x, bbox[0].m_y + cornerLength));
        }
        else
        {
            // / / corner
            // | |
            corPos1.emplace_back(point_2d(bbox[0].m_x + (i - 1) * width, bbox[1].m_y - cornerLength));
            corPos1.emplace_back(point_2d(bbox[0].m_x + (i - 1) * width + cornerLength, bbox[1].m_y));

            //horizontal
            positions.emplace_back(point_2d(bbox[0].m_x + (i - 1) * width + cornerLength, bbox[1].m_y));
            positions.emplace_back(point_2d(bbox[0].m_x + (i)*width - cornerLength, bbox[1].m_y));

            // \ \ corner
            // | |
            corPos2.emplace_back(point_2d(bbox[0].m_x + (i)*width - cornerLength, bbox[1].m_y));
            corPos2.emplace_back(point_2d(bbox[0].m_x + (i)*width, bbox[1].m_y - cornerLength));
        }
        if (i != 0)
        {
            Segment cor1(id++, netId, getSegmentWidth(), layer);
            cor1.setPosition(corPos1);
            m_snaking.push_back(cor1);
        }

        Segment segHor(id++, netId, getSegmentWidth(), layer);
        segHor.setPosition(positions);
        m_snaking.push_back(segHor);

        Segment cor2(id++, netId, getSegmentWidth(), layer);
        cor2.setPosition(corPos2);
        m_snaking.push_back(cor2);

        std::cout << "add segment " << std::endl;
    }
    positions.clear();
    corPos1.clear();
    corPos2.clear();
    //++i;
    std::cout << "i: " << i << std::endl;

    if (i % 2 == 0)
    {
        //if (abs(bbox[0].m_y - m_points[1].m_y) >= 0.1)

        // | |
        //  \ \ corner
        corPos1.emplace_back(point_2d(bbox[0].m_x + (i - 1) * width, bbox[0].m_y + cornerLength));
        corPos1.emplace_back(point_2d(bbox[0].m_x + (i - 1) * width + cornerLength, bbox[0].m_y));
        //horizontal
        positions.emplace_back(point_2d(bbox[0].m_x + (i - 1) * width + cornerLength, bbox[0].m_y));
        positions.emplace_back(point_2d(bbox[1].m_x - cornerLength, bbox[0].m_y));
        // | |
        // / /corner

        corPos2.emplace_back(point_2d(bbox[1].m_x - cornerLength, bbox[0].m_y));
        corPos2.emplace_back(point_2d(bbox[1].m_x, bbox[0].m_y + cornerLength));
        intersectPt = getIntersectionOfPointLine(m_points[1], corPos2);
        corPos2.clear();
        corPos2.emplace_back(point_2d(bbox[1].m_x - cornerLength, bbox[0].m_y));
        corPos2.emplace_back(intersectPt);
    }
    else
    {
        // / / corner
        // | |
        corPos1.emplace_back(point_2d(bbox[0].m_x + (i - 1) * width, bbox[1].m_y - cornerLength));
        corPos1.emplace_back(point_2d(bbox[0].m_x + (i - 1) * width + cornerLength, bbox[1].m_y));
        //horizontal
        positions.emplace_back(point_2d(bbox[0].m_x + (i - 1) * width + cornerLength, bbox[1].m_y));
        positions.emplace_back(point_2d(bbox[1].m_x - cornerLength, bbox[1].m_y));
        // \ \ corner
        // | |
        corPos2.emplace_back(point_2d(bbox[1].m_x - cornerLength, bbox[1].m_y));
        corPos2.emplace_back(point_2d(bbox[1].m_x, bbox[1].m_y - cornerLength));
        intersectPt = getIntersectionOfPointLine(m_points[1], corPos2);
        corPos2.clear();
        corPos2.emplace_back(point_2d(bbox[1].m_x - cornerLength, bbox[1].m_y));
        corPos2.emplace_back(intersectPt);
    }
    Segment cor1(id++, netId, getSegmentWidth(), layer);
    cor1.setPosition(corPos1);
    m_snaking.push_back(cor1);

    Segment segHor(id++, netId, getSegmentWidth(), layer);
    segHor.setPosition(positions);
    m_snaking.push_back(segHor);

    Segment cor2(id++, netId, getSegmentWidth(), layer);
    cor2.setPosition(corPos2);
    m_snaking.push_back(cor2);

    positions.clear();
    corPos1.clear();
    corPos2.clear();

    Segment seg(id++, netId, getSegmentWidth(), layer);
    if (i % 2 == 0)
    {
        positions.emplace_back(point_2d(bbox[1].m_x, bbox[0].m_y + cornerLength));
        positions.emplace_back(point_2d(bbox[1].m_x, m_points[1].m_y - cornerLength));

        corPos1.emplace_back(point_2d(bbox[1].m_x, m_points[1].m_y - cornerLength));
        corPos1.emplace_back(point_2d(bbox[1].m_x + cornerLength, m_points[1].m_y));
    }
    else
    {
        positions.emplace_back(point_2d(bbox[1].m_x, bbox[1].m_y - cornerLength));
        positions.emplace_back(point_2d(bbox[1].m_x, m_points[1].m_y + cornerLength));

        corPos1.emplace_back(point_2d(bbox[1].m_x, m_points[1].m_y + cornerLength));
        corPos1.emplace_back(point_2d(bbox[1].m_x + cornerLength, m_points[1].m_y));
    }
    seg.setPosition(positions);
    //m_Snaking.push_back(seg);
    std::cout << "add segment " << std::endl;

    Segment cor(id++, netId, getSegmentWidth(), layer);
    cor.setPosition(corPos1);
    // m_Snaking.push_back(cor);

    points_2d endSegPos;
    endSegPos.emplace_back(intersectPt);
    endSegPos.emplace_back(point_2d(m_points[1].m_x, m_points[1].m_y));

    Segment endSeg(id++, netId, getSegmentWidth(), layer);
    endSeg.setPosition(endSegPos);
    m_snaking.push_back(endSeg);
}

Segments Snaking::getSnakingPattern(int angle)
{
    if (angle != 0)
        snakingRotation(angle);
    return m_snaking;
}

void Snaking::snakingRotation(int angle)
{
    //double angle = 0;
    auto cords = points_2d{};
    auto rads = fmod(angle * -M_PI / 180, 2 * M_PI);
    auto s = sin(rads);
    auto c = cos(rads);
    for (auto &seg : m_snaking)
    {
        auto p = seg.getPos();
        for (int i = 0; i < 2; ++i)
        {
            auto x = double(c * p[i].m_x - s * p[i].m_y);
            auto y = double(s * p[i].m_x + c * p[i].m_y);
            cords.push_back(point_2d{x, y});
        }
        seg.setPosition(cords);
        cords.clear();
    }
}

point_2d Snaking::getIntersectionOfPointLine(point_2d &pt, points_2d &line)
{
    // Line1 must be horizontal line (y=b)
    double a1, a2, b1, b2; //slope and intercept

    //a1 = (line1[0].m_y - line1[1].m_y) / (line1[0].m_x - line1[1].m_x);
    a2 = (line[0].m_y - line[1].m_y) / (line[0].m_x - line[1].m_x);
    //b1 = line1[0].m_y - a1 * line1[0].m_x;
    b2 = line[0].m_y - a2 * line[0].m_x;

    point_2d intersect;
    intersect.m_y = pt.m_y;
    intersect.m_x = (pt.m_y - b2) / a2;
    return intersect;
}
