#ifndef PCBDRC_DRC_H
#define PCBDRC_DRC_H


#include "kicadPcbDataBase.h"
#include <boost/geometry.hpp>
#include <boost/mpl/string.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/adapted/boost_tuple.hpp> 


#include <boost/geometry/geometries/geometries.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <boost/geometry/index/indexable.hpp>


// to store queries results
#include <vector>

// just for output
#include <iostream>
#include <boost/foreach.hpp>

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

//TODO: rtree compile-time parameter
//reference: https://www.boost.org/doc/libs/1_65_1/libs/geometry/doc/html/geometry/reference/spatial_indexes/boost__geometry__index__rtree.html


typedef bg::model::point<double, 2, bg::cs::cartesian> point;
typedef bg::model::box<point> box;
typedef std::pair<int, box> value;

enum class ObjectType
{
    PIN,
    VIA,
    SEGMENT,
    NONE
};

class Object
{
    public:
        Object( const ObjectType type = ObjectType::NONE,
                const int dbId = -1,
                const int netId = -1,  
                const std::string compName = "",
                const std::string instName = "")
                : m_type(type), m_dbId(dbId), m_netId(netId), m_compName(compName), m_instName(instName){};
        ~Object(){};

    private:
        ObjectType m_type;  
        int m_dbId;         // id in kicad database
        int m_netId;
        std::string m_compName;
        std::string m_instName;
        std::vector< std::pair<int, int> > m_ids;  //< the ith rtree, id in rtree >
};

class Drc
{
    public:
        Drc(){};

        ~Drc(){
         //   m_objects.clear();
          //  m_rtrees.clear();
        };
        void createRTree();

        void printDrc();



    private:
        std::vector<Object> m_objects;
        bgi::rtree< value, bgi::quadratic<16> > _tree;
        //std::vector<bgi::rtree< value, bgi::quadratic<16> >> m_rtrees;   //std::pair<int, box> value -> <object id, object bbox>
};





#endif
