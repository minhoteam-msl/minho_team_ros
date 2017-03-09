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
void Voronoi::clearVoronoi()
{
    voronoi.clear();
    obstacles.clear();
    voronoi_segments_visualizer.clear();
}



/*
void Voronoi::insertArtificialObstacles()
{
    voronoi.insert(artificial_obstacles.begin(),artificial_obstacles.end());
}*/


/*Site Voronoi::getFace1(Point point)
{
    Locate_Result locate_res = voronoi.locate(point);

    cout<<"loc: "<<locate_res.which()<<endl;

    if(Face_Handle *face_handle = boost::get<Face_Handle>(&locate_res)) {

        cout<<"ola"<<endl;
        return Site((*face_handle)->dual()->point());
    }
    else{
         cout<<"null"<<endl;
        return Site(NULL,NULL);

}
}*/

/*
//
void Voronoi::site_Obstacles_Circle(Point circle_center, float radius)  // !!!n
{
    vector<Site> obst;

    float perimeter = 2.0 * M_PI * radius * 2.5;

    int points_num = perimeter/fundamental->getRobotDiameter();

    float angle_step = (2.0 * M_PI)/points_num;

    for(int i=0; i<points_num; i++) {

        float angle = angle_step * i;
        float x = circle_center.x() + radius * 2.5 * cos(angle); //!!!
        float y = circle_center.y() + radius * 2.5 * sin(angle); //!!!

        obst.push_back(Site(x,y));
    }

    obstacles.insert(obstacles.end(), obst.begin(), obst.end());
}
*/





