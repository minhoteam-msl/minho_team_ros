/*
 * LAR - Laboratório de Automação e Robótica
 * MinhoTeam robotic soccer team
 * http://www.robotica.dei.uminho.pt/robocup2016/
 * DEI - University of Minho, Portugal
 *
 * fundamental.h
 *
 * Created: ,2016
 * Author: Tiago Maia
 *
 */

#ifndef FUNDAMENTAL_H
#define FUNDAMENTAL_H

//include
#include <QFile>
#include <QString>
#include <QTextStream>
#include <QElapsedTimer>
#include <iostream>
#include <sstream>
#include <pthread.h>
#include <iterator>
#include <math.h>
#include "Utils/types.h"
//include for signal
#include <sys/time.h>
#include <unistd.h>
#include <signal.h>
//include ROS
#include "ros/ros.h"
#include "minho_team_ros/controlInfo.h" // PUBLISH
#include "minho_team_ros/robotInfo.h" // SUBSCRIBE
#include "minho_team_ros/aiInfo.h" // SUBSCRIBE
#include "minho_team_ros/controlConfig.h" // SUBSCRIBE
#include "minho_team_ros/requestKick.h" // CALLING SERVICE
#include "minho_team_ros/segment.h" // CALLING SERVICE
#include "minho_team_ros/pathData.h" // CALLING SERVICE
#include "minho_team_ros/position.h"
#include "minho_team_ros/requestControlConfig.h"
//include CGAL for Voronoi Diagram
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Delaunay_triangulation_adaptation_traits_2.h>
#include <CGAL/Delaunay_triangulation_adaptation_policies_2.h>
#include <CGAL/Voronoi_diagram_2.h>
//include Adapting an arrangement (Voronoi Diagram) to a BGL graph
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Arrangement_2.h>
//#include <CGAL/Arr_non_caching_segment_basic_traits_2.h>
#include <CGAL/Arr_segment_traits_2.h>
#include <CGAL/graph_traits_Arrangement_2.h>
#include <CGAL/Arr_vertex_index_map.h>
#include <CGAL/property_map.h>
#include <CGAL/Arr_batched_point_location.h>
//include shortest path
#include <boost/graph/dijkstra_shortest_paths.hpp>       // !!!!
//#include <CGAL/boost/graph/dijkstra_shortest_paths.h>  // !!!!
/// \include CGAL Circular Geometry Kernel and Intersections
#include <CGAL/Exact_circular_kernel_2.h>
#include <CGAL/Circular_kernel_intersections.h>
#include <CGAL/intersections.h>
#include <CGAL/iterator.h>
#include <CGAL/Exact_rational.h>
//include SINTEF Spline Library for cubic B-splines
#include <sisl.h>


using namespace std;
using namespace ros;

using minho_team_ros::controlInfo;
using minho_team_ros::robotInfo;
using minho_team_ros::aiInfo;
using minho_team_ros::controlConfig;
using minho_team_ros::requestKick;
using minho_team_ros::position;
using minho_team_ros::requestControlConfig;
using minho_team_ros::segment;
using minho_team_ros::pathData;


//typedef CGAL
typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::Segment_2 Segment;
typedef Kernel::Iso_rectangle_2 Iso_Rectangle;
typedef Kernel::Ray_2 Ray;
typedef Kernel::Line_2 Line;
typedef CGAL::Delaunay_triangulation_2<Kernel> Delaunay_Triangulation;
typedef CGAL::Delaunay_triangulation_adaptation_traits_2<Delaunay_Triangulation> AdaptationTraits;
//Ver qual o melhor com ou sem caching*******************************************************************************
//typedef CGAL::Delaunay_triangulation_caching_degeneracy_removal_policy_2<Delaunay_Triangulation> AdaptationPolicy;
typedef CGAL::Delaunay_triangulation_degeneracy_removal_policy_2<Delaunay_Triangulation> AdaptationPolicy;
typedef CGAL::Voronoi_diagram_2<Delaunay_Triangulation, AdaptationTraits, AdaptationPolicy> Voronoi_Diagram;
typedef AdaptationTraits::Point_2 Point;
typedef AdaptationTraits::Site_2 Site;
typedef Voronoi_Diagram::Locate_result Locate_Result;
typedef Voronoi_Diagram::Face_handle Face_Handle;
typedef Voronoi_Diagram::Ccb_halfedge_circulator Ccb_Halfedge_Circulator;
typedef Voronoi_Diagram::Vertex_iterator Vertex_Iterator;
typedef Voronoi_Diagram::Edge_iterator Edge_Iterator;
//typedef Adapting an arrangement (Voronoi Diagram) to a BGL graph.
typedef CGAL::Exact_predicates_exact_constructions_kernel Arr_Kernel;
//typedef CGAL::Arr_non_caching_segment_basic_traits_2<Arr_Kernel> Arr_Traits;
typedef CGAL::Arr_segment_traits_2<Arr_Kernel> Arr_Traits;
typedef Arr_Traits::Point_2 Arr_Point;
typedef Arr_Traits::X_monotone_curve_2 Arr_Segment;
typedef CGAL::Arrangement_2<Arr_Traits> Arrangement;
typedef CGAL::Arr_point_location_result<Arrangement> Arr_Point_Location_Result;
typedef pair<Arr_Point, Arr_Point_Location_Result::Type> Arr_Query_Result;
typedef Arrangement::Vertex_const_handle Arr_Vertex_Const_Handle;
//typedef CGAL Circular Geometry Kernel and Intersections
typedef CGAL::Exact_circular_kernel_2 Circ_Kernel;
typedef Circ_Kernel::Point_2 Circ_Point;
typedef Circ_Kernel::Circle_2 Circle;
typedef Circ_Kernel::Line_2 Circ_Line;
typedef pair<CGAL::Circular_arc_point_2<Circ_Kernel>, unsigned> InterRes;
typedef CGAL::Dispatch_output_iterator<CGAL::cpp11::tuple<InterRes>,CGAL::cpp0x::tuple<back_insert_iterator<vector<InterRes>>>> Dispatcher;


class Fundamental
{
public:
    Fundamental();   
    float distance(float pointAx, float pointAy, float pointBx, float pointBy);
    float distance_without_sqrt(float pointAx, float pointAy, float pointBx, float pointBy);
    float cartesian2polar_angleRad(float pointAx, float pointAy, float pointBx, float pointBy);
    int cartesian2polar_angleDeg(float pointAx, float pointAy, float pointBx, float pointBy);
    float cartesian2polar_angleRadNormalize(float pointAx, float pointAy, float pointBx, float pointBy);
    int cartesian2polar_angleDegNormalize(float pointAx, float pointAy, float pointBx, float pointBy);
    int cartesian2polar_angleDeg_halfCircle(float pointAx, float pointAy, float pointBx, float pointBy);
    float getFieldLength();
    float getFieldWidth();
    float getTotalFieldLength();
    float getTotalFieldWidth();
    float getOutsideFieldLength();
    float getOutsideFieldWidth();
    float getHalfFieldLength();
    float getHalfFieldWidth();
    float getRobotDiameter();
    bool intersect_Segment_and_Circle(Segment seg, Point circle_center, float radius);
    bool intersect_Segment_and_Circle(Segment seg, Point circle_center, float radius, vector<Point>& intersection_points);
    bool intersect_Line_and_Circle(Point line_pointA, Point line_pointB, Point circle_center, float radius, vector<Point>& intersection_points);

private:
    bool initParameters();
    bool readField();

    QString field_file;
    /// \brief structure containing diverse information/dimensions from a field
    fieldDimensions field;
    //
    float total_field_length, total_field_width, field_length, field_width;
    float outside_field_length, outside_field_width;
    float half_field_length, half_field_width;
    float robot_diameter;

};

#endif // FUNDAMENTAL_H
