#include "boundingBox.h"

void BoundingBox::createBBox()
{
    // auto coord = points_2d{}; //[ll, rr]
    point_2d p1 = getPoint(m_start, m_angle), p2 = getPoint(m_end, m_angle);

    m_bbox.emplace_back(point_2d{p1.m_x, p1.m_y - m_leftRightWidth.second});
    m_bbox.emplace_back(point_2d{p2.m_x, p1.m_y - m_leftRightWidth.second});
    m_bbox.emplace_back(point_2d{p2.m_x, p2.m_y + m_leftRightWidth.first});
    m_bbox.emplace_back(point_2d{p1.m_x, p2.m_y + m_leftRightWidth.first});
}

point_2d BoundingBox::getPoint(const point_2d &p, int angle)
{
    auto coord = point_2d{};
    auto rads = fmod(angle * -M_PI / 180, 2 * M_PI);
    auto s = sin(rads);
    auto c = cos(rads);
    coord.m_x = double(c * p.m_x - s * p.m_y);
    coord.m_y = double(s * p.m_x + c * p.m_y);

    return coord;
}
points_2d BoundingBox::getBBox(int angle)
{
    if (angle == 0)
        return m_bbox;

    auto coords = points_2d{};
    for (const auto &p : m_bbox)
    {
        coords.emplace_back(getPoint(p, angle));
    }
    return coords;
}

void BoundingBox::printBBox(int angle)
{
    if (angle == 0)
        std::cout << "Polygon(" << m_bbox[0] << ", " << m_bbox[1] << ", " << m_bbox[2] << ", " << m_bbox[3] << ")" << std::endl;
    else
    {
        auto bbox = getBBox(angle);
        std::cout << "Polygon(" << bbox[0] << ", " << bbox[1] << ", " << bbox[2] << ", " << bbox[3] << ")" << std::endl;
    }
}