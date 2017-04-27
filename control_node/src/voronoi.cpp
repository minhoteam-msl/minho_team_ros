#include "voronoi.h"

Voronoi::Voronoi(Fundamental *fund)
{
    fundamental = fund;

    initialize_ArtificialObstacles();
}

//
void Voronoi::initialize_ArtificialObstacles()
{
    float number_points;

    number_points = fundamental->getFieldLength()/fundamental->getRobotDiameter();
    float length_interval = fundamental->getRobotDiameter() +
            ((fundamental->getRobotDiameter()*(number_points-(int)number_points))/(int)number_points);

    number_points = fundamental->getFieldWidth()/fundamental->getRobotDiameter();
    float width_interval = fundamental->getRobotDiameter() +
            ((fundamental->getRobotDiameter()*(number_points-(int)number_points))/(int)number_points);

    for(float x=-fundamental->getHalfFieldLength(); x<=fundamental->getHalfFieldLength(); x+=length_interval)
    {
        artificial_obstacles.push_back(Site(x,-fundamental->getHalfFieldWidth()));
        artificial_obstacles.push_back(Site(x,fundamental->getHalfFieldWidth()));
    }

    for(float y=-fundamental->getHalfFieldWidth()+width_interval; y<=fundamental->getHalfFieldWidth()-width_interval; y+=width_interval)
    {
        artificial_obstacles.push_back(Site(-fundamental->getHalfFieldLength(),y));
        artificial_obstacles.push_back(Site(fundamental->getHalfFieldLength(),y));
    }
}

//insert all obstacles in Voronoi Diagram
void Voronoi::insert_Obstacles_VoronoiDiagram(robotInfo robot)
{
    for(unsigned int i=0; i<robot.obstacles.size(); i++)
    {
        if(fundamental->isInsideField(Point(robot.obstacles.at(i).x,robot.obstacles.at(i).y), 0.0))
            obstacles.push_back(Site(robot.obstacles.at(i).x,robot.obstacles.at(i).y));
    }

    obstacles.insert(obstacles.end(),artificial_obstacles.begin(),artificial_obstacles.end());

    voronoi.insert(obstacles.begin(),obstacles.end());
}

//
Voronoi_Diagram Voronoi::getVoronoi()
{
    return voronoi;
}

//
vector<Site> Voronoi::getObstacles()
{
    return obstacles;
}

//
vector<Site> Voronoi::getArtificialObstacles()
{
    return artificial_obstacles;
}

//
void Voronoi::cut_VoronoiDiagram_FieldLimits()
{
    segment seg;

    Iso_Rectangle bbox(-fundamental->getHalfFieldLength(),-fundamental->getHalfFieldWidth(),
                       fundamental->getHalfFieldLength(),fundamental->getHalfFieldWidth());

    Cropped_voronoi_from_delaunay vor(bbox);

    Delaunay_Triangulation delaunay_triangulation = voronoi.dual();

    delaunay_triangulation.draw_dual(vor);

    for(auto it = vor.m_cropped_vd.begin(); it != vor.m_cropped_vd.end(); it++)
    {
        if(!isnan(it->source().x()) && !isnan(it->source().y()) && !isnan(it->target().x()) && !isnan(it->target().y())) {
            seg.ini.x = it->source().x();
            seg.ini.y = it->source().y();
            seg.fini.x = it->target().x();
            seg.fini.y = it->target().y();
            insert_VoronoiSegments_Visualizer(seg);
        }
    }
}

//
void Voronoi::insert_VoronoiSegments_Visualizer(segment seg)
{
    voronoi_segments_visualizer.push_back(seg);
}

//
vector<segment> Voronoi::get_VoronoiSegments_Visualizer()
{
    return voronoi_segments_visualizer;
}

//
Locate_Result Voronoi::locatePoint(Point point)
{
    return voronoi.locate(point);
}

//
Ccb_Halfedge_Circulator Voronoi::getHalfedgeCirculator(Face_Handle face_handle)
{
    return voronoi.ccb_halfedges(face_handle);
}

//
void Voronoi::get_Vertices_of_VoronoiFace(Face_Handle face_handle, vector<Point>& vertices)
{
    Ccb_Halfedge_Circulator edge_circulator_begin = voronoi.ccb_halfedges(face_handle);
    Ccb_Halfedge_Circulator edge_circulator = edge_circulator_begin;

    do{
        if(edge_circulator->is_segment()) {

            Point p_source = edge_circulator->source()->point();
            Point p_target = edge_circulator->target()->point();

            bool has_p_source = false;
            bool has_p_target = false;

            for(unsigned int i=0; i<vertices.size(); i++) {
                if(p_source == vertices.at(i))
                    has_p_source = true;
                if(p_target == vertices.at(i))
                    has_p_target = true;
            }

            if(!has_p_source)
                vertices.push_back(p_source);
            if(!has_p_target)
                vertices.push_back(p_target);
        }
    }while(++edge_circulator != edge_circulator_begin);

    if(vertices.size() == 0) {
        if(edge_circulator_begin->has_source())
            vertices.push_back(edge_circulator_begin->source()->point());
        else if(edge_circulator_begin->has_target())
            vertices.push_back(edge_circulator_begin->target()->point());
    }
}

//
void Voronoi::clearVoronoi()
{
    voronoi.clear();
    obstacles.clear();
    voronoi_segments_visualizer.clear();
}
