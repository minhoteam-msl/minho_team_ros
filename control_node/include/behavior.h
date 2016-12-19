#ifndef BEHAVIOR_H
#define BEHAVIOR_H

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
#include <QFile>
#include <QString>
#include <QTextStream>
#include <iostream>
#include <sstream>
#include <pthread.h>
#include <time.h>
//defines for signal
#include <sys/time.h>
#include <unistd.h>
#include <signal.h>
#include "types.h"

//include CGAL for Voronoi Diagram
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <iterator>

using namespace ros;
using namespace std;
using minho_team_ros::controlInfo;
using minho_team_ros::robotInfo;
using minho_team_ros::aiInfo;
using minho_team_ros::controlConfig;
using minho_team_ros::requestKick;
using minho_team_ros::position;
using minho_team_ros::requestControlConfig;
using minho_team_ros::segment;
using minho_team_ros::pathData;

//include CGAL for Voronoi Diagram
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Delaunay_triangulation_adaptation_traits_2.h>
#include <CGAL/Delaunay_triangulation_adaptation_policies_2.h>
#include <CGAL/Voronoi_diagram_2.h>
#include <iterator>


#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>

//typedef
typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::Point_2 Point_2;
typedef Kernel::Iso_rectangle_2 Iso_rectangle_2;
typedef Kernel::Segment_2 Segment_2;
typedef Kernel::Ray_2 Ray_2;
typedef Kernel::Line_2 Line_2;
typedef CGAL::Delaunay_triangulation_2<Kernel> Delaunay_Triangulation;
typedef CGAL::Delaunay_triangulation_adaptation_traits_2<Delaunay_Triangulation> AdaptationTraits;
typedef CGAL::Delaunay_triangulation_caching_degeneracy_removal_policy_2<Delaunay_Triangulation> AdaptationPolicy;
typedef CGAL::Voronoi_diagram_2<Delaunay_Triangulation, AdaptationTraits, AdaptationPolicy> Voronoi_Diagram;
//
typedef Kernel::Direction_2 Direction;
//
typedef Voronoi_Diagram::Site_2 SiteVD;

typedef Voronoi_Diagram::Vertex_iterator vertexIterator;
typedef Voronoi_Diagram::Halfedge_iterator halfedgeIterator;
typedef Voronoi_Diagram::Halfedge_handle halfedgeHandle;
typedef Voronoi_Diagram::Edge_iterator edgeIterator;
typedef Voronoi_Diagram::Halfedge_around_vertex_circulator halfedgeVertexCirculator;
typedef Voronoi_Diagram::Face_iterator faceIterator;
typedef Voronoi_Diagram::Site_iterator siteIterator;
typedef Voronoi_Diagram::Ccb_halfedge_circulator CcbHalfedgeCirculator;
typedef Voronoi_Diagram::Face_handle FaceHandle;
typedef Voronoi_Diagram::Locate_result LocateResult;


//property for adjacency_list
typedef boost::property<boost::edge_weight_t, float> EdgeWeightProperty;
typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, boost::no_property, EdgeWeightProperty> Graph;
typedef boost::graph_traits<Graph>::vertex_descriptor VertexDescriptor;

enum ROLE {GOALKEEPER = 0, DEFENSE, SUP_ATTACKER, ATTACKER };
enum ACTION {STOP = 0, GO_TO_POSITION, KICK_PASS, ENGAGE_BALL };

typedef struct Vertex{
    VertexDescriptor vertexId;
    Point_2 vertexXY;
}Vertex;

//Voronoi Diagram
typedef struct Cropped_voronoi_from_delaunay{
    vector<Segment_2> m_cropped_vd;
    Iso_rectangle_2 m_bbox;

    Cropped_voronoi_from_delaunay(const Iso_rectangle_2& bbox):m_bbox(bbox){}

    template <class RSL>
    void crop_and_extract_segment(const RSL& rsl){
      CGAL::Object obj = CGAL::intersection(rsl,m_bbox);
      const Segment_2* s=CGAL::object_cast<Segment_2>(&obj);
      if (s) m_cropped_vd.push_back(*s);
    }

    void operator<<(const Ray_2& ray)    { crop_and_extract_segment(ray); }
    void operator<<(const Line_2& line)  { crop_and_extract_segment(line); }
    void operator<<(const Segment_2& seg){ crop_and_extract_segment(seg); }
}Cropped_voronoi_from_delaunay;

class Behavior
{
public:
   Behavior(std::string topics_base_name, int rob_id, ros::NodeHandle *par);
   void robotInfoCallback(const robotInfo::ConstPtr &msg);
   void aiInfoCallback(const aiInfo::ConstPtr &msg);
   void controlConfigCallback(const controlConfig::ConstPtr &msg);
   void doWork();
   bool initParameters();
   bool readControlParameters();
   bool writeControlParameters();
   bool readField();
   bool controlConfService(requestControlConfig::Request &req, requestControlConfig::Response &res);
private:
   /// \brief pointer to parent ros node
   ros::NodeHandle *parent;
   /// \brief ROS subscriber for robotInfo
   ros::Subscriber robot_info_sub;
   /// \brief ROS subscriber for aiInfo
   ros::Subscriber ai_info_sub;
    /// \brief ROS subscriber for controlConfig
   ros::Subscriber cconfig_info_sub;
   /// \brief ROS publisher for controlInfo
   ros::Publisher control_info_pub;
   /// \brief ROS service client requestKick
   ros::ServiceClient kick_service;
   // \brief Service to relay the configuration
   ros::ServiceServer service_cconf;
   /// \brief ROS publisher for pathData
   ros::Publisher path_data_pub;

   // DATA STORAGE
   /// \brief struct to hold control commands (to be published)
   controlInfo control;
   /// \brief struct to hold most recent robotInfo message
   robotInfo robot_info;
   /// \brief struct to hold most recent aiInfo message
   aiInfo ai_info;
   /// \brief struct to hold most recent controlConfig message
   controlConfig cconfig;
   /// \brief structure containing diverse information/dimensions from a field
   fieldDimensions field;
   QString mainfile, controlparam_file, field_file;

   int robot_id;
   // PID VARIABLES
   float error, previous_error;
   float derivative, integral;
   double field_length, field_width;
   
   //VORONOI VARIABLES
   Delaunay_Triangulation delaunayTriangulation;
   Voronoi_Diagram VD;
   Graph graphVoronoi;
   QString sendStringPath;
   double distRobotToSiteAux;
   pathData path_data;  

   // MATH FUNCTIONS
   float Distance(float positionAX, float positionAY, float positionBX, float positionBY);
   int PsiTarget(double positionAX, double positionAY, double positionBX, double positionBY, bool normalize);
   int RotationRobot_for_Target_PID(int psi_target, int rotational_speed_max);
   int SpeedRobot_TargetDistance_Log(double distanceRobot_Target, int linear_speed_max);
   vector<double> ConstructVoronoiDiagram_WithRectangleLimits(robotInfo robot, aiInfo ai);

};

 
#endif // BEHAVIOR_H
