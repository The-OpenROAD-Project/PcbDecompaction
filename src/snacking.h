#ifndef PCBDECOMPACTION_SNACKING_H
#define PCBDECOMPACTION_SNACKING_H
#include "segment.h"
#include "point.h"
#include <assert.h>
#include <stdlib.h>

class Snacking
{
public:
    Snacking(const points_2d &bbox, const double &clearance, const double &width, const points_2d &points) : m_bbox(bbox), m_clearance(clearance), m_segmentWidth(width), m_points(points){};
    ~Snacking() {}
    Segments getSnackingPattern();
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

private:
    points_2d m_bbox; //[(xmin, ymin) (xmax,ymax)]
    double m_clearance;
    double m_segmentWidth;
    points_2d m_points; //[start point, end point]
};

#endif