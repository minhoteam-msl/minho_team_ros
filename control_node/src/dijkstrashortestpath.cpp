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

// only arrangement segments voronoi diagram
void DijkstraShortestPath::insert_Arrangement_Segments_Voronoi_Diagram()
{
    Voronoi_Diagram vd = voronoi->getVoronoi();

    for(Edge_Iterator eit = vd.edges_begin(); eit != vd.edges_end(); eit++) {

        if(eit->is_segment()) {
            insert_Arrangement_Segments(Arr_Segment(Arr_Point(eit->source()->point().x(), eit->source()->point().y()),
                                                    Arr_Point(eit->target()->point().x(), eit->target()->point().y())));
        }
    }
}

// all arrangement segments
void DijkstraShortestPath::insert_Arrangement_Segments(Arr_Segment arr_seg)
{
    arr_segments.push_back(arr_seg);
}

//
void DijkstraShortestPath::intersection_Face(Point point, Face_Handle face_handle)
{
    segment seg;

    Ccb_Halfedge_Circulator edge_circulator_begin = voronoi->getHalfedgeCirculator(face_handle);
    Ccb_Halfedge_Circulator edge_circulator = edge_circulator_begin;

    CGAL::cpp11::result_of<Kernel::Intersect_2(Ray, Segment)>::type intersect_result;

    Point site_point(face_handle->dual()->point());

    float angle = fundamental->cartesian2polar_angleRad(point.x(), point.y(), site_point.x(), site_point.y());

    for(int i=1; i<4; i++) {
        float xRay = cos(angle+(i*M_PI_2))+point.x();
        float yRay = sin(angle+(i*M_PI_2))+point.y();

        Ray ray(point, Point(xRay, yRay));

        do{
            Point p_source = edge_circulator->source()->point();
            Point p_target = edge_circulator->target()->point();

            intersect_result = CGAL::intersection(ray, Segment(p_source, p_target));

            if(intersect_result) {
                if(const Point *intersect_point = boost::get<Point>(&*intersect_result)) {
                    seg.ini.x = point.x();
                    seg.ini.y = -point.y();
                    seg.fini.x = intersect_point->x();
                    seg.fini.y = -intersect_point->y();
                    voronoi->insertSegments_SendVisualizer(seg);

                    insert_Arrangement_Segments(Arr_Segment(Arr_Point(point.x(), point.y()),
                                                            Arr_Point(intersect_point->x(), intersect_point->y())));

                    insert_Arrangement_Segments(Arr_Segment(Arr_Point(p_source.x(), p_source.y()),
                                                            Arr_Point(intersect_point->x(), intersect_point->y())));

                    insert_Arrangement_Segments(Arr_Segment(Arr_Point(intersect_point->x(), intersect_point->y()),
                                                            Arr_Point(p_target.x(), p_target.y())));
                }
            }

        }while(++edge_circulator != edge_circulator_begin);

        edge_circulator = edge_circulator_begin;
    }
}

// construct arrangement with aggregated insertion
void DijkstraShortestPath::construct_Arrangement()
{
    //CGAL::insert_non_intersecting_curves(arrangement, arr_segments.begin(), arr_segments.end());
    CGAL::insert(arrangement, arr_segments.begin(), arr_segments.end());
}

//
void DijkstraShortestPath::calculate_Dijkstra_Shortest_Path(Point source_point, Point target_point)
{
    vector<Point> path_vertices;
    position pos;

    // Create a mapping of the arrangement vertices to indices.
    CGAL::Arr_vertex_index_map<Arrangement> index_map_tmp(arrangement);
    boost::Arr_vertex_index_map_boost<Arrangement> index_map(index_map_tmp);

    Edge_length_func edge_length;

    boost::vector_property_map<double, boost::Arr_vertex_index_map_boost<Arrangement>>
            distances(static_cast<unsigned int>(arrangement.number_of_vertices()), index_map);

    boost::vector_property_map<Arrangement::Vertex_handle, boost::Arr_vertex_index_map_boost<Arrangement>>  //Arrangement::Vertex_handle???
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

    /*for(Arrangement::Vertex_iterator vit = arrangement.vertices_begin(); vit != arrangement.vertices_end(); vit++) {
        cout<<"Vertex point: "<<vit->point()<<" --> "<<"distance = "<<distances[vit]<<", ";
        cout<<"parent = "<<parents[vit]->point()<<endl;
    }*/

    for(Arrangement::Vertex_iterator path_vertex = target_vertex; path_vertex != parents[path_vertex]; path_vertex = parents[path_vertex]) {
        path_points.push_back(CGAL::to_double(path_vertex->point().x()));
        path_points.push_back(CGAL::to_double(-path_vertex->point().y()));
        pos.x = CGAL::to_double(path_vertex->point().x());
        pos.y = CGAL::to_double(-path_vertex->point().y());
        insertPath_SendVisualizer(pos);
    }

    path_points.push_back(CGAL::to_double(source_vertex->point().x()));
    path_points.push_back(CGAL::to_double(-source_vertex->point().y()));
    pos.x = CGAL::to_double(source_vertex->point().x());
    pos.y = CGAL::to_double(-source_vertex->point().y());
    insertPath_SendVisualizer(pos);
}

//
void DijkstraShortestPath::insertPath_SendVisualizer(position pos)
{
    path_visualizer.push_back(pos);
}

//
vector<position> DijkstraShortestPath::getPath_SendVisualizer()
{
    return path_visualizer;
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
void DijkstraShortestPath::dijkstra_SmoothPath()
{
    int number = path_points.size()/2;

    if(number > 1) {

        int order = 5;

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

        SISLCurve* curve = newCurve(number, order, &knots[0], &path_points[0], 1, 2, 0);

        if(!curve) {  // ***temp****
            throw runtime_error("Error occured while generating curve.");
        }

        /*cout<<"number: "<<number<<endl; cout<<"order: "<<order<<endl; cout<<endl;
        for(int j=0; j<knots.size(); j++) {
            cout<<knots.at(j)<<endl;
        }
        cout<<endl;*/

        //100 este valor tem de ser através de uma formula
        int number_samples = 100;
        int leftknot;
        vector<double> curve_samples(2*number_samples);
        int state;
        position pos;

        for(int i=0; i<number_samples; i++) {

            double parameter_value = curve->et[curve->ik-1]+(curve->et[curve->in]-curve->et[curve->ik-1])*i/((double)number_samples-1.0);
            //cout<<"parvalue: "<<parameter_value<<endl;

            s1227(curve, 0, parameter_value, &leftknot, &curve_samples[2*i], &state);

            if(state!=0)  // ***temp****
                cout<<"s1227 returned status: "<<state<<endl;
        }

        nextpoint_x = curve_samples.at(number_samples-4);
        nextpoint_y = curve_samples.at(number_samples-3);
        cout<<curve_samples.at(number_samples-4)<<" "<<curve_samples.at(number_samples-3)<<endl;

        //agora vou fazer outro for() mas posso passar para o for() de cima
        for(int i=0; i<curve_samples.size(); i=i+2) {
            pos.x = curve_samples.at(i);
            pos.y = curve_samples.at(i+1);
            insertSmoothPath_SendVisualizer(pos);
        }

        freeCurve(curve);
    }
}

//
void DijkstraShortestPath::insertSmoothPath_SendVisualizer(position pos)
{
    smooth_path_visualizer.push_back(pos);
}

//
vector<position> DijkstraShortestPath::getSmoothPath_SendVisualizer()
{
    return smooth_path_visualizer;
}

//
void DijkstraShortestPath::Test1(Point point1, Point point2)
{
    insert_Arrangement_Segments_Voronoi_Diagram();

    Locate_Result locate_res1 = voronoi->locatePoint(point1);
    Locate_Result locate_res2 = voronoi->locatePoint(point2);

    if((locate_res1.which()==0) && (locate_res2.which()==0)) {
        Face_Handle *face_handle1 = boost::get<Face_Handle>(&locate_res1);
        Face_Handle *face_handle2 = boost::get<Face_Handle>(&locate_res2);

        if(!(*face_handle1)->is_unbounded() && !(*face_handle2)->is_unbounded()) {
            intersection_Face(point1, *face_handle1);
            intersection_Face(point2, *face_handle2);
            construct_Arrangement();
            calculate_Dijkstra_Shortest_Path(point1, point2);
        }
    }
}

//
void DijkstraShortestPath::clearDijkstraPath()
{
    arrangement.clear();
    arr_segments.clear();
    path_visualizer.clear();
    path_points.clear();
    smooth_path_visualizer.clear();
}



//
void DijkstraShortestPath::teste_circle_intersection_line(Point la, Point lb, Point c)
{
    Circ_Point cla(la.x(), la.y());
    Circ_Point clb(lb.x(), lb.y());

    Circ_Line cir_lin(cla,clb);

    CGAL::Exact_rational sqr_r1 = CGAL::Exact_rational(4.0);   //raio=2 --> 2²

    Circ_Point cc(c.x(), c.y());

    Circle cir1(cc,sqr_r1,CGAL::CLOCKWISE);
    Circ_Point cc2(-6.0,0.0);
    Circle cir2(cc2,sqr_r1,CGAL::CLOCKWISE);

    cout<<"PointRobot: "<<lb.x()<<" "<<lb.y()<<endl;
    cout<<"PointBall: "<<la.x()<<" "<<la.y()<<endl;

    double x_min=0.0;
    double x_max=0.0;
    double y_min=0.0;
    double y_max=0.0;

    if(la.x()<=lb.x()) {
        x_min=la.x()-2.0;
        x_max=lb.x()+2.0;
    }else{
        x_min=lb.x()-2.0;
        x_max=la.x()+2.0;
    }

    if(la.y()<=lb.y()) {
        y_min=la.y()-2.0;
        y_max=lb.y()+2.0;
    }else{
        y_min=lb.y()-2.0;
        y_max=la.y()+2.0;
    }

    if(c.x()>=x_min && c.x()<=x_max && c.y()>=y_min && c.y()<=y_max) {
        if(CGAL::do_intersect(cir1, cir_lin)) {
            cout<<"intersection1"<<endl;
        }
        else
            cout<<"not1"<<endl;
    }



    /*if(CGAL::do_intersect(cir2, cir_lin)) {
        cout<<"intersection2"<<endl;
    }
    else
        cout<<"not2"<<endl;*/

}
