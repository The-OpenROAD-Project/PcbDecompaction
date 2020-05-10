#ifndef BOUNDINGBOX_H
#define BOUNDINGBOX_H
#include "point.h"

class BoundingBox
{
public:
    BoundingBox(std::pair<double, double> lrWidth, int angle, point_2d start, point_2d end) : m_leftRightWidth(lrWidth), m_angle(angle), m_start(start), m_end(end)
    {
        createBBox();
    };
    ~BoundingBox(){};
    void createBBox();
    points_2d getBBox(int angle = 0);
    point_2d getPoint(const point_2d &point, int angle = 0);
    void printBBox(int angle = 0);

private:
    std::pair<double, double> m_leftRightWidth; //(left + half segment width, right + half segment width)
    int m_angle;
    point_2d m_start;
    point_2d m_end;
    points_2d m_bbox; //[ll, lr, rt, lt]
};

#endif
