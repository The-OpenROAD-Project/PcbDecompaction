#include "snacking.h"

Segments Snacking::getSnackingPattern()
{
    double width = getSegmentWidth() + getClearance();
    double length = getBboxLength();
    double cnt = length / width;
    auto bbox = getBbox();
    Segments snacking;
    int id = 0, netId = 0, i = 0;
    points_2d positions;

    for (i = 0; i < cnt - 1; ++i)
    {
        positions.clear();
        if (i == 0)
        {
            positions.emplace_back(point_2d(bbox[0].m_x + i * width, m_points[0].m_y));
        }
        else
        {
            positions.emplace_back(point_2d(bbox[0].m_x + i * width, bbox[1].m_y));
        }
        positions.emplace_back(point_2d(bbox[0].m_x + i * width, bbox[0].m_y));
        Segment seg(id++, netId, getSegmentWidth());
        seg.setPosition(positions);
        snacking.push_back(seg);

        positions.clear();
        if (i % 2 == 0)
        {

            positions.emplace_back(point_2d(bbox[0].m_x + i * width, bbox[0].m_y));
            positions.emplace_back(point_2d(bbox[0].m_x + (i + 1) * width, bbox[0].m_y));
        }
        else
        {

            positions.emplace_back(point_2d(bbox[0].m_x + i * width, bbox[1].m_y));
            positions.emplace_back(point_2d(bbox[0].m_x + (i + 1) * width, bbox[1].m_y));
        }
        Segment segHor(id++, netId, getSegmentWidth());
        segHor.setPosition(positions);
        snacking.push_back(segHor);
    }

    positions.clear();
    Segment seg(id++, netId, getSegmentWidth());
    if (i % 2 == 0)
    {
        positions.emplace_back(point_2d(bbox[0].m_x + (i + 1) * width, bbox[0].m_y));
        positions.emplace_back(point_2d(bbox[0].m_x + (i + 1) * width, m_points[1].m_y));
    }
    else
    {
        positions.emplace_back(point_2d(bbox[0].m_x + (i + 1) * width, bbox[1].m_y));
        positions.emplace_back(point_2d(bbox[0].m_x + (i + 1) * width, m_points[1].m_y));
    }
    seg.setPosition(positions);
    snacking.push_back(seg);

    return snacking;
}