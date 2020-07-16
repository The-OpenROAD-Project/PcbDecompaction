#ifndef PCBDECOMPACTION_SNAKING_H
#define PCBDECOMPACTION_SNAKING_H
#include "segment.h"
#include "point.h"
#include <assert.h>
#include <stdlib.h>
#include <math.h>

class Snaking
{
public:
    Snaking(const points_2d &bbox, const double &clearance, const double &width, const points_2d &points, int &netId, std::string layer = "TOP") : m_bbox(bbox), m_clearance(clearance), m_segmentWidth(width), m_points(points), m_netId(netId), m_layer(layer)
    {
        buildSnakingPattern();
    };
    ~Snaking() {}
    void buildSnakingPattern();
    Segments getSnakingPattern(int angle = 0);
    double &getSegmentWidth() { return m_segmentWidth; }
    double &getClearance() { return m_clearance; }
    points_2d &getBbox()
    {
        assert(m_bbox.size() == 2);
        return m_bbox;
    }
    double getBboxLength()
    {
        if (m_bbox.size() != 2)
            return 0;
        double length = abs(m_bbox[0].m_x - m_bbox[1].m_x);
        return length;
    }
    double getBboxWidth()
    {
        if (m_bbox.size() != 2)
            return 0;
        double length = abs(m_bbox[0].m_y - m_bbox[1].m_y);
        return length;
    }

    void snakingRotation(int angle);
    point_2d getIntersectionOfPointLine(point_2d &pt, points_2d &line);

    //Refactor
    void oldbuildSnakingPattern();

private:
    points_2d m_bbox; //[(xmin, ymin) (xmax,ymax)]
    double m_clearance;
    double m_segmentWidth;
    points_2d m_points; //[start point, end point]
    Segments m_snaking;
    std::string m_layer;
    int m_netId;
};

#endif