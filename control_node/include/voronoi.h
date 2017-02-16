/*
 * LAR - Laboratório de Automação e Robótica
 * MinhoTeam robotic soccer team
 * http://www.robotica.dei.uminho.pt/robocup2016/
 * DEI - University of Minho, Portugal
 *
 * voronoi.h
 *
 * Created: ,2016
 * Author: Tiago Maia
 *
 */

#ifndef VORONOI_H
#define VORONOI_H

//
#include "fundamental.h"


//cut voronoi diagram
struct Cropped_voronoi_from_delaunay {
    list<Segment> m_cropped_vd;
    Iso_Rectangle m_bbox;

    Cropped_voronoi_from_delaunay(const Iso_Rectangle& bbox):m_bbox(bbox){}

    template <class RSL>
    void crop_and_extract_segment(const RSL& rsl){
      CGAL::Object obj = CGAL::intersection(rsl,m_bbox);
      const Segment* s = CGAL::object_cast<Segment>(&obj);
      if(s) m_cropped_vd.push_back(*s);
    }

    void operator<<(const Ray& ray)    { crop_and_extract_segment(ray); }
    void operator<<(const Line& line)  { crop_and_extract_segment(line); }
    void operator<<(const Segment& seg){ crop_and_extract_segment(seg); }
};


class Voronoi
{
public:
    Voronoi(Fundamental *fund);
    void insertObstacles(robotInfo robot);
    Voronoi_Diagram getVoronoi();
    vector<Site> getObstacles();
    vector<Site> getArtificialObstacles();
    void cutVoronoiDiagram_FieldLimits();
    void insertSegments_SendVisualizer(segment seg);
    vector<segment> getSegments_SendVisualizer();
    Locate_Result locatePoint(Point point);
    Ccb_Halfedge_Circulator getHalfedgeCirculator(Face_Handle face_handle);
    void clearVoronoi();

private:
    void initializeArtificialObstacles();

    Fundamental *fundamental;
    Voronoi_Diagram voronoi;
    vector<Site> artificial_obstacles;
    vector<Site> obstacles;
    vector<segment> segments_visualizer;

};

#endif // VORONOI_H
