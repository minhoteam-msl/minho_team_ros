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


class DijkstraShortestPath
{
public:
    DijkstraShortestPath(Fundamental *fund, Voronoi *vor);
    vector<position> getPath_SendVisualizer();
    void dijkstra_SmoothPath();
    vector<position> getSmoothPath_SendVisualizer();

    void Test1(Point point1, Point point2);

    void clearDijkstraPath();

    float nextpoint_x;
    float nextpoint_y;

    void teste_circle_intersection_line(Point la, Point lb, Point c);

private:
    void insert_Arrangement_Segments_Voronoi_Diagram();
    void insert_Arrangement_Segments(Arr_Segment arr_seg);
    void intersection_Face(Point point, Face_Handle face_handle);
    void construct_Arrangement();
    void calculate_Dijkstra_Shortest_Path(Point source_point, Point target_point);
    void insertPath_SendVisualizer(position pos);
    void insertSmoothPath_SendVisualizer(position pos);

    Fundamental *fundamental;
    Voronoi *voronoi;
    Arrangement arrangement;
    vector<Arr_Segment> arr_segments;  //????list_vector??
    vector<position> path_visualizer;
    vector<double> path_points;
    vector<position> smooth_path_visualizer;

    //vector<Segment> additional_segments;
};

#endif // DIJKSTRASHORTESTPATH_H
