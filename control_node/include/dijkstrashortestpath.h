/*
 * LAR - Laboratório de Automação e Robótica
 * MinhoTeam robotic soccer team
 * http://www.robotica.dei.uminho.pt/robocup2016/
 * DEI - University of Minho, Portugal
 *
 * dijkstrashortestpath.h
 *
 * Created: ,2016
 * Author: Tiago Maia
 *
 */

#ifndef DIJKSTRASHORTESTPATH_H
#define DIJKSTRASHORTESTPATH_H

//
#include "fundamental.h"
#include "voronoi.h"


// A functor used to compute the length of an edge.
class Edge_length_func
{
public:
    // Boost property type definitions:
    typedef boost::readable_property_map_tag category;
    typedef double value_type;
    typedef value_type reference;
    typedef Arrangement::Halfedge_handle key_type;

    double operator()(Arrangement::Halfedge_handle e) const
    {
        const double x1 = CGAL::to_double (e->source()->point().x());
        const double y1 = CGAL::to_double (e->source()->point().y());
        const double x2 = CGAL::to_double (e->target()->point().x());
        const double y2 = CGAL::to_double (e->target()->point().y());
        const double diff_x = x2 - x1;
        const double diff_y = y2 - y1;
        return sqrt(diff_x*diff_x + diff_y*diff_y);
    }
};

//
namespace boost {
template <typename Arrangement>
class Arr_vertex_index_map_boost :
        public CGAL::Arr_vertex_index_map<Arrangement>
{
public:
    typedef CGAL::Arr_vertex_index_map<Arrangement> Base;
    Arr_vertex_index_map_boost() : Base() {}
    Arr_vertex_index_map_boost(Base & other) :
    CGAL::Arr_vertex_index_map<Arrangement>(other){}
};

template<class Arrangement_>
unsigned int get(const boost::Arr_vertex_index_map_boost<Arrangement_> & index_map,
                 typename Arrangement_::Vertex_handle v)
{
    const CGAL::Arr_vertex_index_map<Arrangement_> & index_map_tmp =
    static_cast<const CGAL::Arr_vertex_index_map<Arrangement_> &>(index_map);
    return CGAL::get<Arrangement_>(index_map_tmp, v);
}
}

//Center and radius of the circle for obstacles
struct Obstacles_Circle {
    Point circle_center;
    float radius;
};


class DijkstraShortestPath
{
public:
    DijkstraShortestPath(Fundamental *fund, Voronoi *vor);
    vector<float> get_ObstaclesCircle_Visualizer();
    vector<segment> get_IntersectSegments_Visualizer();
    vector<position> get_DijkstraPath_Visualizer();
    vector<position> get_DijkstraPath_with_ObstaclesCircle_Visualizer();
    vector<position> get_SmoothPath_Visualizer();
    vector<position> get_SmoothPath_with_ObstaclesCircle_Visualizer();
    vector<position> get_Path_Visualizer();

    void Test1(robotInfo robot, Point target_point, vector<Point>& path);

    void clearDijkstraPath();

private:
    void calculate_Radius_of_Obstacles(robotInfo robot, Point target_point, bool& source_in_radius, bool& target_in_radius);
    void insert_ObstaclesCircle_Visualizer(float obst_circle);
    bool directPath_without_Obstacles(Point source_point, Point target_point, vector<Point>& direct_path);
    void insert_VoronoiSegments_at_Arrangement();
    void insert_ArrangementSegments(Arr_Segment arr_seg);
    void insert_IntersectSegments_Visualizer(segment seg);
    void insert_FaceIntersectionSegments_at_Arrangement(Point point, Face_Handle face_handle);
    void insert_FaceSegments_with_ObstaclesCircle_at_Arrangement(Point point, const vector<Point>& vertices);
    void insert_FaceSegments_with_SiteObstacle_at_Arrangement(Point point, Face_Handle face_handle, const vector<Point>& vertices);
    bool insert_Source_and_Target_at_Arrangement(Point source_point, Point target_point, bool source_in_radius, bool target_in_radius);
    void construct_Arrangement();
    bool calculate_DijkstraPath(Point source_point, Point target_point, vector<Point>& dijkstra_path);
    void insert_DijkstraPath_Visualizer(position pos);
    void dijkstraPath_with_ObstaclesCircle(const vector<Point>& path_points, vector<double>& dijkstra_path_obst_circle);
    void insert_DijkstraPath_with_ObstaclesCircle_Visualizer(position pos);
    void smoothPath(vector<double>& path_points, vector<Point>& curve_points);
    void insert_SmoothPath_Visualizer(position pos);
    void smoothPath_with_ObstaclesCircle(vector<Point>& curve_points);
    void insert_SmoothPath_with_ObstaclesCircle_Visualizer(position pos);
    void shortest_SmoothPath_with_ObstaclesCircle(const vector<Point>& curve_points, vector<Point>& shortest_path);
    void insert_Path_Visualizer(position pos);

    Fundamental *fundamental;
    Voronoi *voronoi;
    vector<Obstacles_Circle> obstacles_circle;
    vector<float> obstacles_circle_visualizer;
    vector<Arr_Segment> arr_segments;             //????list_vector??
    Arrangement arrangement;
    vector<segment> intersect_segments_visualizer;
    vector<position> dijkstra_path_visualizer;
    vector<position> dijkstra_path_obst_circle_visualizer;
    vector<position> smooth_path_visualizer;
    vector<position> smooth_path_obst_circle_visualizer;
    vector<position> path_visualizer;

};

#endif // DIJKSTRASHORTESTPATH_H
