#ifndef PCBDRC_DRC_H
#define PCBDRC_DRC_H


#include "kicadPcbDataBase.h"
#include "net.h"
#include "module.h"
#include "segment.h"
#include "via.h"
#include "point.h"
#include "shape.h"
#include "object.h"


// to store queries results
#include <vector>

// just for output
#include <iostream>

/*
namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;
typedef bg::model::point<double, 2, bg::cs::cartesian> point;
typedef bg::model::box<point> box;
typedef std::pair<box, int> value;
typedef bg::model::polygon<point> polygon_t;
*/

//TODO: rtree compile-time parameter
//reference: https://www.boost.org/doc/libs/1_65_1/libs/geometry/doc/html/geometry/reference/spatial_indexes/boost__geometry__index__rtree.html


class Drc
{
    public:
        Drc(kicadPcbDataBase &db) : m_db(db){};
        ~Drc(){
         //   m_objects.clear();
          //  m_rtrees.clear();
        };
        void createRTree();
        bool checkIntersection();
        void testProjection();
        void traverseRTree();
        void printDrc();
        void printObject();
        void checkClearance();
        points_2d projection(point_2d &, points_2d &, int );
        std::vector<double> buildRelation(int &, const int &);
        std::vector<double> lineEquation(point_2d &, point_2d &);
        void printEquation(std::vector<double> &);

        void printPolygon(points_2d &coord);
        void printPoint(point_2d&);



    private:
        std::vector<Object> m_objects;
        std::vector<bgi::rtree< value, bgi::quadratic<16> >> m_rtrees;
        kicadPcbDataBase &m_db;
        //std::vector<bgi::rtree< value, bgi::quadratic<16> >> m_rtrees;   //std::pair<int, box> value -> <object id, object bbox>
};





#endif
