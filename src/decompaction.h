#ifndef PCBDECOMPACTION_DECOMPACTION_H
#define PCBDECOMPACTION_DECOMPACTION_H

#include "kicadPcbDataBase.h"
#include "net.h"
#include "module.h"
#include "segment.h"
#include "via.h"
#include "point.h"
#include "shape.h"
#include "object.h"
#include "snaking.h"
#include "boundingBox.h"
#include <vector>
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

class Decompaction
{
public:
    Decompaction(kicadPcbDataBase &db) : m_db(db){};
    ~Decompaction(){
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
    points_2d projection(point_2d &, points_2d &, int);
    std::vector<std::vector<double>> buildRelation(int &, const int &);
    std::vector<double> lineEquation(point_2d &, point_2d &);
    std::vector<std::vector<double>> lineEquation(point_2d &, point_2d &, point_2d &, point_2d &);
    std::vector<std::vector<double>> inequalityLineEquation(point_2d &, point_2d &, point_2d &, point_2d &, point_2d &);
    void printEquation(std::vector<double> &);
    void printEquation(std::vector<std::vector<double>> &);
    void printInequalityEquation(std::vector<double> &, point_2d &);
    void printInequalityEquation(std::vector<std::vector<double>> &, point_2d &);
    void printInequalityEquation(std::vector<double> &);
    void printSegment(points_2d &line);
    void printPolygon(points_2d &coord);
    void printPoint(point_2d &);
    std::vector<double> getInequalityEquation(std::vector<double> &, point_2d &);
    void writeLPfile(std::string &);
    void writeLPfileForBus(std::string &fileName);
    void readLPSolution(std::string &);
    void updateValue(int &, std::string, double &, ObjectType);
    void updateDatabase();
    void updatePinsShapeAndPosition();
    void updateExtraSpace(int &objId, std::string type, double &width);
    void printObject(int &);
    void clearEquations();
    void addWidthToBusSegmentEquation();
    double maxLength();
    void getSnaking();
    void getBoundingBox();
    void addSnakingPatterns();
    void testBBoxSnaking();
    double getNetLength(int &);
    void printAllNetLength();
    int checkDRC();
    vector<string> collectNonoverlapInstEqu();
    void writeLPInstFile(std::string &fileName);

private:
    std::vector<Object> m_objects;
    std::vector<bgi::rtree<value, bgi::quadratic<16>>> m_rtrees;
    std::vector<point_2d> m_instDiffPos; //inst's diff position for each iteration
    kicadPcbDataBase &m_db;
    int m_numLayer;
    bool m_verbose;

    //std::vector<bgi::rtree< value, bgi::quadratic<16> >> m_rtrees;   //std::pair<int, box> value -> <object id, object bbox>
};

#endif
