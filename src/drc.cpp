#include "drc.h"


void Drc::createRTree()
{
    m_rtrees.resize(m_db.getNumCopperLayers());
    std::vector<net> nets = m_db.getNets();
    int id = 0;
    for (auto &&net : nets)
    {
        std::vector<pin> pins = net.getPins();
        std::vector<Segment> segments = net.getSegments();
        std::vector<Via> vias = net.getVias();

        for (auto &&s : segments) 
        {
            points_2d line = s.getPos();
            auto width = s.getWidth();
            int layerId = m_db.getLayerId(s.getLayer());
            points_2d coord = segment_to_rect(line, width);
            polygon_t polygon;
            for(auto &&p : coord)
            {
                bg::append(polygon.outer(), point(p.m_x, p.m_y)); 
            }
            bg::append(polygon.outer(), point(coord[0].m_x, coord[0].m_y)); 

            box b = bg::return_envelope<box>(polygon);
            Object obj(ObjectType::SEGMENT, s.getId(), s.getNetId(), -1, -1);
            if (layerId == 0)
            {
                m_rtrees[0].insert(std::make_pair(b, id));
                obj.setRTreeId(std::make_pair(0,id));
                ++id;
                
            }
            else if (layerId == 31)
            {
                m_rtrees[1].insert(std::make_pair(b, id));
                obj.setRTreeId(std::make_pair(1,id));
                ++id;
            }
            m_objects.push_back(obj);
        }

        for (auto &&v : vias)
        {
            point_2d pos = v.getPos();
            auto size = v.getSize();
            points_2d coord = via_to_circle(pos, size);
            polygon_t polygon;
            for(auto &&p : coord)
            {
                bg::append(polygon.outer(), point(p.m_x, p.m_y)); 
            }
            bg::append(polygon.outer(), point(coord[0].m_x, coord[0].m_y)); 

            box b = bg::return_envelope<box>(polygon);
            Object obj(ObjectType::VIA, v.getId(), v.getNetId(), -1, -1);
        
            m_rtrees[0].insert(std::make_pair(b, id));
            obj.setRTreeId(std::make_pair(0,id));
            m_rtrees[1].insert(std::make_pair(b, id));
            obj.setRTreeId(std::make_pair(1,id));
            ++id;           
            m_objects.push_back(obj);
        }

        for (auto &&pin : pins)
        {
            int padId = pin.m_padstack_id;
            int compId = pin.m_comp_id;
            int instId = pin.m_inst_id;
            polygon_t polygon;
            component comp = m_db.getComponent(compId);
            instance inst = m_db.getInstance(instId);
            auto &pad = comp.getPadstack(padId);
            points_2d coord = shape_to_cords(pad.getSize(), pad.getPos(), padShape::RECT, inst.getAngle(), pad.getAngle(), 0);
            for (auto && p : coord)
            {
                p.m_x = p.m_x + inst.getX();
                p.m_y = p.m_y + inst.getY();
            }

            for(auto &&p : coord)
            {
                bg::append(polygon.outer(), point(p.m_x, p.m_y)); 
            }
            bg::append(polygon.outer(), point(coord[0].m_x, coord[0].m_y)); 
            std::vector<std::string> layers = pad.getLayers();
            box b = bg::return_envelope<box>(polygon);
            Object obj(ObjectType::PIN, pad.getId(), net.getId(), compId, instId);

            for (auto &&layer : layers)
            {
                int layerId = m_db.getLayerId(layer);
                if (layerId == 0)
                {
                    m_rtrees[0].insert(std::make_pair(b, id));
                    obj.setRTreeId(std::make_pair(0,id));
                }
                else if (layerId == 31)
                {
                    m_rtrees[1].insert(std::make_pair(b, id));
                    obj.setRTreeId(std::make_pair(1,id));
                }
            }
            m_objects.push_back(obj);
        }
    }
}

bool Drc::checkIntersection()
{
    

    return true;
}

void Drc::printDrc()
{


}

void Drc::printObject()
{
    std::cout << std::endl;
    std::cout << "#####################################" << std::endl;
    std::cout << "###                               ###" << std::endl;
    std::cout << "###             OBJ              ###" << std::endl;
    std::cout << "###                               ###" << std::endl;
    std::cout << "#####################################" << std::endl;
    for (auto &&obj : m_objects){
        std::vector< std::pair<int, int> > ids = obj.getId();
        for (auto &&id : ids)
        {
            std::cout << "(" << id.first << "," << id.second << ") ";
        }
        std::cout << std::endl;
    }
}
