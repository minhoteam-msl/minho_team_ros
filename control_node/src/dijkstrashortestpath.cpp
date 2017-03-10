#include "dijkstrashortestpath.h"

DijkstraShortestPath::DijkstraShortestPath(Fundamental *fund, Voronoi *vor)
{
    voronoi = vor;
    fundamental = fund;
}

//
double get(Edge_length_func edge_length, Arrangement::Halfedge_handle e)
{
    return edge_length(e);
}

//
void DijkstraShortestPath::calculate_Radius_of_Obstacles(robotInfo robot)
{
    Obstacles_Circle obst_circle;

    for(unsigned int i=0; i<robot.obstacles.size(); i++)
    {
        obst_circle.circle_center = Point(robot.obstacles.at(i).x,robot.obstacles.at(i).y);
        obst_circle.radius = 0.52; //radius:50cm->0.5
        obstacles_circle.push_back(obst_circle);

        insert_ObstaclesCircle_Visualizer(obst_circle.circle_center.x());
        insert_ObstaclesCircle_Visualizer(obst_circle.circle_center.y());
        insert_ObstaclesCircle_Visualizer(obst_circle.radius);
    }
}

//
void DijkstraShortestPath::insert_ObstaclesCircle_Visualizer(float obst_circle)
{
    obstacles_circle_visualizer.push_back(obst_circle);
}

//
vector<float> DijkstraShortestPath::get_ObstaclesCircle_Visualizer()
{
    return obstacles_circle_visualizer;
}

//Check if direct path between source and target is possible without obstacles
bool DijkstraShortestPath::directPath_without_Obstacles(Point source_point, Point target_point, vector<Point>& direct_path)
{
    bool intersect = false;
    unsigned int i=0;
    position pos;

    while(i<obstacles_circle.size() && !intersect) {
        intersect = fundamental->intersect_Segment_and_Circle(Segment(source_point, target_point),
                                                              obstacles_circle.at(i).circle_center, obstacles_circle.at(i).radius);
        i++;
    }

    if(!intersect) {
        direct_path.push_back(source_point);
        direct_path.push_back(target_point);

        pos.x = source_point.x();
        pos.y = source_point.y();
        insert_Path_Visualizer(pos);

        pos.x = target_point.x();
        pos.y = target_point.y();
        insert_Path_Visualizer(pos);
    }

    return !intersect;
}

// only arrangement segments voronoi diagram
void DijkstraShortestPath::insert_VoronoiSegments_at_Arrangement()
{
    Voronoi_Diagram vd = voronoi->getVoronoi();
    bool intersect;
    unsigned int i;
    segment seg;

    for(Edge_Iterator eit = vd.edges_begin(); eit != vd.edges_end(); eit++) {

        if(eit->is_segment()) {

            intersect = false;
            i=0;

            while(i<obstacles_circle.size() && !intersect) {
                intersect = fundamental->intersect_Segment_and_Circle(Segment(eit->source()->point(), eit->target()->point()),
                                                                      obstacles_circle.at(i).circle_center, obstacles_circle.at(i).radius);
                i++;
            }

            if(!intersect)
                insert_ArrangementSegments(Arr_Segment(Arr_Point(eit->source()->point().x(), eit->source()->point().y()),
                                                        Arr_Point(eit->target()->point().x(), eit->target()->point().y())));
            else {
                seg.ini.x = eit->source()->point().x();
                seg.ini.y = eit->source()->point().y();
                seg.fini.x = eit->target()->point().x();
                seg.fini.y = eit->target()->point().y();
                insert_IntersectSegments_Visualizer(seg);
            }
        }
    }
}

// all arrangement segments
void DijkstraShortestPath::insert_ArrangementSegments(Arr_Segment arr_seg)
{
    arr_segments.push_back(arr_seg);
}

//
void DijkstraShortestPath::insert_IntersectSegments_Visualizer(segment seg)
{
    intersect_segments_visualizer.push_back(seg);
}

//
vector<segment> DijkstraShortestPath::get_IntersectSegments_Visualizer()
{
    return intersect_segments_visualizer;
}

//
void DijkstraShortestPath::insert_FaceIntersectionSegments_at_Arrangement(Point point, Face_Handle face_handle)
{
    bool intersect_segs_circle;
    bool intersect_segt_circle;
    bool intersect_segp_circle;
    unsigned int j;
    segment seg;

    Ccb_Halfedge_Circulator edge_circulator_begin = voronoi->getHalfedgeCirculator(face_handle);
    Ccb_Halfedge_Circulator edge_circulator = edge_circulator_begin;

    CGAL::cpp11::result_of<Kernel::Intersect_2(Ray, Segment)>::type intersect_result;

    for(int i=0; i<360; i=i+30) {
        float angle_rad = ((float)i*M_PI)/180.0;
        float xRay = cos(angle_rad)+point.x();
        float yRay = sin(angle_rad)+point.y();

        Ray ray(point, Point(xRay, yRay));

        do{
            Point p_source = edge_circulator->source()->point();
            Point p_target = edge_circulator->target()->point();

            intersect_result = CGAL::intersection(ray, Segment(p_source, p_target));

            if(intersect_result) {
                if(const Point *intersect_point = boost::get<Point>(&*intersect_result)) {

                    intersect_segs_circle = false;
                    j=0;
                    while(j<obstacles_circle.size() && !intersect_segs_circle) {
                        intersect_segs_circle = fundamental->intersect_Segment_and_Circle(Segment(p_source, *intersect_point),
                                                                              obstacles_circle.at(j).circle_center, obstacles_circle.at(j).radius);
                        j++;
                    }

                    intersect_segt_circle = false;
                    j=0;
                    while(j<obstacles_circle.size() && !intersect_segt_circle) {
                        intersect_segt_circle = fundamental->intersect_Segment_and_Circle(Segment(p_target, *intersect_point),
                                                                              obstacles_circle.at(j).circle_center, obstacles_circle.at(j).radius);
                        j++;
                    }

                    intersect_segp_circle = false;
                    j=0;
                    while(j<obstacles_circle.size() && !intersect_segp_circle) {
                        intersect_segp_circle = fundamental->intersect_Segment_and_Circle(Segment(point, *intersect_point),
                                                                              obstacles_circle.at(j).circle_center, obstacles_circle.at(j).radius);
                        j++;
                    }

                    if((!intersect_segs_circle || !intersect_segt_circle) && !intersect_segp_circle) {
                        if(!intersect_segs_circle)
                            insert_ArrangementSegments(Arr_Segment(Arr_Point(p_source.x(), p_source.y()),
                                                                    Arr_Point(intersect_point->x(), intersect_point->y())));
                        if(!intersect_segt_circle)
                            insert_ArrangementSegments(Arr_Segment(Arr_Point(intersect_point->x(), intersect_point->y()),
                                                                    Arr_Point(p_target.x(), p_target.y())));

                        insert_ArrangementSegments(Arr_Segment(Arr_Point(point.x(), point.y()),
                                                                Arr_Point(intersect_point->x(), intersect_point->y())));
                        seg.ini.x = point.x();
                        seg.ini.y = point.y();
                        seg.fini.x = intersect_point->x();
                        seg.fini.y = intersect_point->y();
                        voronoi->insert_VoronoiSegments_Visualizer(seg);
                    }
                }
            }
        }while(++edge_circulator != edge_circulator_begin);

        edge_circulator = edge_circulator_begin;
    }
}

//
void DijkstraShortestPath::insert_FaceSegments_at_Arrangement(Point point, Face_Handle face_handle)
{
    vector<Point> vertices;
    bool intersect;
    unsigned int j;
    segment seg;

    Ccb_Halfedge_Circulator edge_circulator_begin = voronoi->getHalfedgeCirculator(face_handle);
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

    for(unsigned int i=0; i<vertices.size(); i++) {

        intersect = false;
        j=0;

        while(j<obstacles_circle.size() && !intersect) {
            intersect = fundamental->intersect_Segment_and_Circle(Segment(point, vertices.at(i)),
                                                                  obstacles_circle.at(j).circle_center, obstacles_circle.at(j).radius);
            j++;
        }

        if(!intersect) {
            insert_ArrangementSegments(Arr_Segment(Arr_Point(point.x(), point.y()),
                                                    Arr_Point(vertices.at(i).x(), vertices.at(i).y())));
            seg.ini.x = point.x();
            seg.ini.y = point.y();
            seg.fini.x = vertices.at(i).x();
            seg.fini.y = vertices.at(i).y();
            voronoi->insert_VoronoiSegments_Visualizer(seg);
        }else {
            seg.ini.x = point.x();
            seg.ini.y = point.y();
            seg.fini.x = vertices.at(i).x();
            seg.fini.y = vertices.at(i).y();
            insert_IntersectSegments_Visualizer(seg);
        }
    }
}

// construct arrangement with aggregated insertion
void DijkstraShortestPath::construct_Arrangement()
{
    CGAL::insert(arrangement, arr_segments.begin(), arr_segments.end());
}

//
bool DijkstraShortestPath::calculate_DijkstraPath(Point source_point, Point target_point, vector<Point>& dijkstra_path)
{
    bool has_path;
    position pos;

    // Create a mapping of the arrangement vertices to indices.
    CGAL::Arr_vertex_index_map<Arrangement> index_map_tmp(arrangement);
    boost::Arr_vertex_index_map_boost<Arrangement> index_map(index_map_tmp);
    Edge_length_func edge_length;
    
    boost::vector_property_map<double, boost::Arr_vertex_index_map_boost<Arrangement>>
            distances(static_cast<unsigned int>(arrangement.number_of_vertices()), index_map);
    boost::vector_property_map<Arrangement::Vertex_handle, boost::Arr_vertex_index_map_boost<Arrangement>>
            parents(static_cast<unsigned int>(arrangement.number_of_vertices()), index_map);
            
    Arr_Point arr_s_point(source_point.x(), source_point.y());
    Arr_Point arr_t_point(target_point.x(), target_point.y());
    vector<Arr_Point> points;
    points.push_back(arr_s_point);
    points.push_back(arr_t_point);

    vector<Arr_Query_Result> results;
    CGAL::locate(arrangement, points.begin(), points.end(), back_inserter(results));
    vector<Arr_Query_Result>::const_iterator it;
    Arrangement::Vertex_handle source_vertex, target_vertex;

    for(it = results.begin(); it != results.end(); it++) {
        if(const Arr_Vertex_Const_Handle* arr_vertex = boost::get<Arr_Vertex_Const_Handle>(&(it->second))) {
            if((*arr_vertex)->point() == arr_s_point) {
                source_vertex = arrangement.non_const_handle(*arr_vertex);
            }
            else if((*arr_vertex)->point() == arr_t_point) {
                target_vertex = arrangement.non_const_handle(*arr_vertex);
            }
        }
    }
    

    boost::dijkstra_shortest_paths(arrangement, source_vertex, boost::vertex_index_map(index_map).weight_map(edge_length).
                                   distance_map(distances).predecessor_map(parents));                           

    if(target_vertex != parents[target_vertex]) {
        has_path = true;
        
        /*for(Arrangement::Vertex_iterator vit = arrangement.vertices_begin(); vit != arrangement.vertices_end(); vit++) {
            cout<<"Vertex point: "<<vit->point()<<" --> "<<"distance = "<<distances[vit]<<", ";
            cout<<"parent = "<<parents[vit]->point()<<endl;
        }*/
        
        for(Arrangement::Vertex_iterator path_vertex = target_vertex; path_vertex != parents[path_vertex]; path_vertex = parents[path_vertex]) {
            dijkstra_path.push_back(Point(CGAL::to_double(path_vertex->point().x()), CGAL::to_double(path_vertex->point().y())));
            pos.x = CGAL::to_double(path_vertex->point().x());
            pos.y = CGAL::to_double(path_vertex->point().y());
            insert_DijkstraPath_Visualizer(pos);
        }
        
        dijkstra_path.push_back(Point(CGAL::to_double(source_vertex->point().x()), CGAL::to_double(source_vertex->point().y())));
        pos.x = CGAL::to_double(source_vertex->point().x());
        pos.y = CGAL::to_double(source_vertex->point().y());
        insert_DijkstraPath_Visualizer(pos);

    }else {
        has_path = false;
    }

    return has_path;
}

//
void DijkstraShortestPath::insert_DijkstraPath_Visualizer(position pos)
{
    dijkstra_path_visualizer.push_back(pos);
}

//
vector<position> DijkstraShortestPath::get_DijkstraPath_Visualizer()
{
    return dijkstra_path_visualizer;
}

//
void DijkstraShortestPath::dijkstraPath_with_ObstaclesCircle(const vector<Point>& path_points, vector<double>& dijkstra_path_obst_circle)
{
    int i;
    unsigned int j;
    position pos;
    bool intersect;

    Point path_pointA(path_points.at(path_points.size()-1));
    Point path_pointB(path_points.at(path_points.size()-2));
    Point path_pointB_previous = path_pointB;

    dijkstra_path_obst_circle.push_back(path_pointA.x());
    dijkstra_path_obst_circle.push_back(path_pointA.y());
    pos.x = path_pointA.x();
    pos.y = path_pointA.y();
    insert_DijkstraPath_with_ObstaclesCircle_Visualizer(pos);

    i = path_points.size()-3;

    while(i>-1) {

        path_pointB = path_points.at(i);
        intersect = false;
        j=0;

        while(j<obstacles_circle.size() && !intersect) {
            intersect = fundamental->intersect_Segment_and_Circle(Segment(path_pointA, path_pointB),
                                                                  obstacles_circle.at(j).circle_center, obstacles_circle.at(j).radius);
            j++;
        }

        if(intersect) {
            path_pointA = path_pointB_previous;
            dijkstra_path_obst_circle.push_back(path_pointA.x());
            dijkstra_path_obst_circle.push_back(path_pointA.y());
            pos.x = path_pointA.x();
            pos.y = path_pointA.y();
            insert_DijkstraPath_with_ObstaclesCircle_Visualizer(pos);
        }

        path_pointB_previous = path_pointB;
        i--;
    }

    dijkstra_path_obst_circle.push_back(path_pointB.x());
    dijkstra_path_obst_circle.push_back(path_pointB.y());
    pos.x = path_pointB.x();
    pos.y = path_pointB.y();
    insert_DijkstraPath_with_ObstaclesCircle_Visualizer(pos);
}

//
void DijkstraShortestPath::insert_DijkstraPath_with_ObstaclesCircle_Visualizer(position pos)
{
    dijkstra_path_obst_circle_visualizer.push_back(pos);
}

//
vector<position> DijkstraShortestPath::get_DijkstraPath_with_ObstaclesCircle_Visualizer()
{
    return dijkstra_path_obst_circle_visualizer;
}

//
/*
 * number          - number of control points
 * order           - order of spline curve (degree + 1)
 * knots           - pointer to knot vector (parametrization)
 * parameter_value - The parameter value at which to compute position and derivatives
 * state           - If state  >0: warning | =0: ok | <0: error
 */
//
void DijkstraShortestPath::smoothPath(vector<double>& path_points, vector<Point>& curve_points)
{
    int number = path_points.size()/2;

    if(number > 1) {

        int order = 4;

        if(number < 4 && number > 1)  //number>1 ou number>2 não sei se tem logica fazer para 2 pontos apenas
            order = number;

        vector<double> knots;

        for(int i=0; i<number+order; i++) {
            if(i < order)
                knots.push_back(0);
            else if(i >= order && i <= number)
                knots.push_back(knots.at(i-1)+1);
            else
                knots.push_back(knots.at(i-1));
        }

        /*
        cout<<"number: "<<number<<endl; cout<<"order: "<<order<<endl; cout<<endl;
        for(int j=0; j<knots.size(); j++) {
            cout<<knots.at(j)<<endl;
        }
        cout<<endl;
        */

        SISLCurve* curve = newCurve(number, order, &knots[0], &path_points[0], 1, 2, 0);

        if(!curve) {
            throw runtime_error("Error occured while generating curve.");
        }else {
            int number_samples = 20;  // !!!!!
            int leftknot;
            vector<double> curve_samples(2*number_samples);
            int state;
            position pos;

            for(int i=0; i<number_samples; i++) {

                double parameter_value = curve->et[curve->ik-1]+(curve->et[curve->in]-curve->et[curve->ik-1])*i/((double)number_samples-1.0);
                //cout<<"parvalue: "<<parameter_value<<endl;

                s1227(curve, 0, parameter_value, &leftknot, &curve_samples[2*i], &state);

                if(state!=0)
                    cout<<"s1227 returned status: "<<state<<endl;
            }

            for(int i=0; i<curve_samples.size(); i+=2) {
                curve_points.push_back(Point(curve_samples.at(i),curve_samples.at(i+1)));
                pos.x = curve_samples.at(i);
                pos.y = curve_samples.at(i+1);
                insert_SmoothPath_Visualizer(pos);
            }

            freeCurve(curve);
        }
    }
}

//
void DijkstraShortestPath::insert_SmoothPath_Visualizer(position pos)
{
    smooth_path_visualizer.push_back(pos);
}

//
vector<position> DijkstraShortestPath::get_SmoothPath_Visualizer()
{
    return smooth_path_visualizer;
}

//
void DijkstraShortestPath::smoothPath_with_ObstaclesCircle(vector<Point>& curve_points)
{
    bool intersect;
    vector<Point> points;
    vector<Point> points_intersect;
    vector<Point> curve_points_aux;
    float angle_first, angle_second, angle_interval, angle_step;
    int points_number = 8;  //este valor pode ser calculado pelo perimetro
    position pos;

    if(curve_points.size() > 2) {

        for(int i=0; i<obstacles_circle.size(); i++) {

            for(int j=0; j<curve_points.size()-1; j++) {

                intersect = fundamental->intersect_Segment_and_Circle(Segment(curve_points.at(j),curve_points.at(j+1)),
                                                                      obstacles_circle.at(i).circle_center, obstacles_circle.at(i).radius, points);

                for(int k=0; k<points.size(); k++) {
                    points_intersect.push_back(points.at(k));
                }
                points.clear();

                if(points_intersect.size() > 0 && (!intersect || (j+1 == curve_points.size()-1))) {

                    if(points_intersect.size() == 1) {

                        float angle = fundamental->cartesian2polar_angleRad(obstacles_circle.at(i).circle_center.x(),
                                                                            obstacles_circle.at(i).circle_center.y(),
                                                                            points_intersect.at(0).x(), points_intersect.at(0).y());

                        float point_x = obstacles_circle.at(i).circle_center.x() + (obstacles_circle.at(i).radius + 0.1) * cos(angle);
                        float point_y = obstacles_circle.at(i).circle_center.y() + (obstacles_circle.at(i).radius + 0.1) * sin(angle);

                        curve_points_aux.push_back(Point(point_x,point_y));

                    }else if(points_intersect.size() == 2) {

                        float angle0 = fundamental->cartesian2polar_angleRad(obstacles_circle.at(i).circle_center.x(),
                                                                             obstacles_circle.at(i).circle_center.y(),
                                                                             points_intersect.at(0).x(), points_intersect.at(0).y());

                        float angle1 = fundamental->cartesian2polar_angleRad(obstacles_circle.at(i).circle_center.x(),
                                                                             obstacles_circle.at(i).circle_center.y(),
                                                                             points_intersect.at(1).x(), points_intersect.at(1).y());

                        int j_aux = j;
                        if(j+1 == curve_points.size()-1)
                            j_aux = j+1;

                        float dist0 = fundamental->distance_without_sqrt(points_intersect.at(0).x(), points_intersect.at(0).y(),
                                                                         curve_points.at(j_aux).x(), curve_points.at(j_aux).y());

                        float dist1 = fundamental->distance_without_sqrt(points_intersect.at(1).x(), points_intersect.at(1).y(),
                                                                         curve_points.at(j_aux).x(), curve_points.at(j_aux).y());

                        if(dist0 < dist1) {
                            angle_first = angle1;
                            angle_second = angle0;
                        }else {
                            angle_first = angle0;
                            angle_second = angle1;
                        }

                        angle_interval = angle_second - angle_first;

                        if(angle_interval > M_PI)
                            angle_interval -= (2.0 * M_PI);
                        else if(angle_interval < -M_PI)
                            angle_interval += (2.0 * M_PI);

                        angle_step = angle_interval/(float)points_number;

                        int points_number_aux = 0;
                        while(points_number_aux <= points_number) {

                            float point_x = obstacles_circle.at(i).circle_center.x() + (obstacles_circle.at(i).radius + 0.1) * cos(angle_first);
                            float point_y = obstacles_circle.at(i).circle_center.y() + (obstacles_circle.at(i).radius + 0.1) * sin(angle_first);

                            curve_points_aux.push_back(Point(point_x,point_y));

                            angle_first += angle_step;

                            if(angle_first > M_PI)
                                angle_first -= (2.0 * M_PI);
                            else if(angle_first < -M_PI)
                                angle_first += (2.0 * M_PI);

                            points_number_aux++;
                        }
                    }
                    points_intersect.clear();
                }

                if(!intersect)
                    curve_points_aux.push_back(curve_points.at(j));

                if(j+1 == curve_points.size()-1)
                    curve_points_aux.push_back(curve_points.at(j+1));
            }

            curve_points.clear();
            for(int p=0; p<curve_points_aux.size(); p++)
                curve_points.push_back(curve_points_aux.at(p));
            curve_points_aux.clear();
        }
    }

    for(int i=0; i<curve_points.size(); i++) {
        pos.x = curve_points.at(i).x();
        pos.y = curve_points.at(i).y();
        insert_SmoothPath_with_ObstaclesCircle_Visualizer(pos);
    }
}

//
void DijkstraShortestPath::insert_SmoothPath_with_ObstaclesCircle_Visualizer(position pos)
{
    smooth_path_obst_circle_visualizer.push_back(pos);
}

//
vector<position> DijkstraShortestPath::get_SmoothPath_with_ObstaclesCircle_Visualizer()
{
    return smooth_path_obst_circle_visualizer;
}

//
void DijkstraShortestPath::shortest_SmoothPath_with_ObstaclesCircle(const vector<Point>& curve_points, vector<Point>& shortest_path)
{
    int i;
    unsigned int j;
    position pos;
    bool intersect;

    Point path_pointA(curve_points.at(0));
    Point path_pointB(curve_points.at(1));
    Point path_pointB_previous = path_pointB;

    shortest_path.push_back(path_pointA);
    pos.x = path_pointA.x();
    pos.y = path_pointA.y();
    insert_Path_Visualizer(pos);

    i=2;
    while(i<curve_points.size()) {

        path_pointB = curve_points.at(i);
        intersect = false;
        j=0;

        while(j<obstacles_circle.size() && !intersect) {
            intersect = fundamental->intersect_Segment_and_Circle(Segment(path_pointA, path_pointB),
                                                                  obstacles_circle.at(j).circle_center, obstacles_circle.at(j).radius);
            j++;
        }

        if(intersect) {
            path_pointA = path_pointB_previous;
            shortest_path.push_back(path_pointA);
            pos.x = path_pointA.x();
            pos.y = path_pointA.y();
            insert_Path_Visualizer(pos);
        }

        path_pointB_previous = path_pointB;
        i++;
    }

    shortest_path.push_back(path_pointB);
    pos.x = path_pointB.x();
    pos.y = path_pointB.y();
    insert_Path_Visualizer(pos);
}

//end path
void DijkstraShortestPath::insert_Path_Visualizer(position pos)
{
    path_visualizer.push_back(pos);
}

//end path
vector<position> DijkstraShortestPath::get_Path_Visualizer()
{
    return path_visualizer;
}

//
void DijkstraShortestPath::Test1(robotInfo robot, Point target_point, vector<Point>& path)
{
    QElapsedTimer timer;
    double t;
    timer.start();
    vector<Point> dijkstra_path;
    vector<double> dijkstra_path_obst_circle;
    vector<Point> smooth_path;
    bool has_path, direct_path;

    Point source_point(robot.robot_pose.x, robot.robot_pose.y);
    calculate_Radius_of_Obstacles(robot);
    direct_path = directPath_without_Obstacles(source_point, target_point, path);

    if(!direct_path) {
        voronoi->insert_Obstacles_VoronoiDiagram(robot);
        insert_VoronoiSegments_at_Arrangement();
        Locate_Result locate_source = voronoi->locatePoint(source_point);
        Locate_Result locate_target = voronoi->locatePoint(target_point);

        if(locate_source.which()==0 && locate_target.which()==0) {
            Face_Handle *face_handle_source = boost::get<Face_Handle>(&locate_source);
            Face_Handle *face_handle_target = boost::get<Face_Handle>(&locate_target);

            insert_FaceSegments_at_Arrangement(source_point, *face_handle_source);
            insert_FaceSegments_at_Arrangement(target_point, *face_handle_target);
 
            construct_Arrangement();

            has_path = calculate_DijkstraPath(source_point, target_point, dijkstra_path);
            

            if(has_path) {
                dijkstraPath_with_ObstaclesCircle(dijkstra_path, dijkstra_path_obst_circle);
                smoothPath(dijkstra_path_obst_circle, smooth_path);
                smoothPath_with_ObstaclesCircle(smooth_path);
                shortest_SmoothPath_with_ObstaclesCircle(smooth_path, path);
            }
        }
    }

    t = timer.nsecsElapsed()/1000000.0; //cout<<"timer: "<<t<<endl;
}

//
void DijkstraShortestPath::clearDijkstraPath()
{
    obstacles_circle.clear();
    obstacles_circle_visualizer.clear();
    arr_segments.clear();
    arrangement.clear();
    intersect_segments_visualizer.clear();   
    dijkstra_path_visualizer.clear();
    dijkstra_path_obst_circle_visualizer.clear();
    smooth_path_visualizer.clear();
    smooth_path_obst_circle_visualizer.clear();
    path_visualizer.clear();
}


/*
//
void DijkstraShortestPath::Test1(robotInfo robot, Point pointA, Point pointB)
{
    QElapsedTimer timer;
    double t;

    voronoi->insertObstacles(robot);

    calculate_Radius_of_Obstacles(robot);

    insert_Arrangement_Segments_Voronoi_Diagram();

    Locate_Result locate_resA = voronoi->locatePoint(pointA);
    Locate_Result locate_resB = voronoi->locatePoint(pointB);

    if(locate_resA.which()==0 && locate_resB.which()==0) {
        Face_Handle *face_handleA = boost::get<Face_Handle>(&locate_resA);
        Face_Handle *face_handleB = boost::get<Face_Handle>(&locate_resB);
timer.start();
        if((*face_handleA)->is_unbounded())
            insert_FaceSegments_at_Arrangement(pointA, *face_handleA);
        else
            insert_FaceIntersectionSegments_at_Arrangement(pointA, *face_handleA);

        if((*face_handleB)->is_unbounded())
            insert_FaceSegments_at_Arrangement(pointB, *face_handleB);
        else
            insert_FaceIntersectionSegments_at_Arrangement(pointB, *face_handleB);
t = timer.nsecsElapsed()/1000000.0; cout<<"timer: "<<t<<endl;
        construct_Arrangement();
        calculate_Dijkstra_Shortest_Path(pointA, pointB);
        shortestPath_with_ObstaclesCircle();
        smoothPath();
    }
}
*/
