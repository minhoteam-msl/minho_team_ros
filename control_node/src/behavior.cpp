#include "behavior.h"

pthread_mutex_t robot_info_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t ai_info_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t cconfig_mutex = PTHREAD_MUTEX_INITIALIZER;

Behavior::Behavior(std::string topics_base_name, int rob_id, ros::NodeHandle *par)
{
   parent = par;
   robot_id = rob_id;
   stringstream robot_topic_name; robot_topic_name << topics_base_name;
   stringstream control_topic_name; control_topic_name << topics_base_name;
   stringstream ai_topic_name; ai_topic_name << topics_base_name;
   stringstream kick_service_name; kick_service_name << topics_base_name;
   stringstream cconfig_topic_name; cconfig_topic_name << topics_base_name;   
   stringstream cconfig_sv_topic_name; cconfig_sv_topic_name << topics_base_name;
   stringstream path_topic_name; path_topic_name << topics_base_name;

   robot_topic_name << "/robotInfo";
   control_topic_name << "/controlInfo";
   ai_topic_name << "/aiInfo";
   kick_service_name << "/requestKick";
   cconfig_topic_name << "/controlConfig";
   cconfig_sv_topic_name << "/requestControlConfig";
   path_topic_name << "/pathData";
   
   //## Setup ROS Pubs and Subs ##
   //############################# 
   ROS_INFO("Getting configurations from Robot %d", robot_id);
   //Initialize controlInfo publisher
   control_info_pub = parent->advertise<controlInfo>(control_topic_name.str(), 1);
   path_data_pub = parent->advertise<pathData>(path_topic_name.str(), 1);
	//Initialize robotInfo subscriber
	robot_info_sub = par->subscribe(robot_topic_name.str(), 
                                       1, 
                                       &Behavior::robotInfoCallback,
                                       this);               
   ai_info_sub = par->subscribe(ai_topic_name.str(), 
                                       1, 
                                       &Behavior::aiInfoCallback,
                                       this);   
   cconfig_info_sub = par->subscribe(cconfig_topic_name.str(), 
                                       1, 
                                       &Behavior::controlConfigCallback,
                                       this);        

   kick_service = par->serviceClient<requestKick>(kick_service_name.str());       

   service_cconf = par->advertiseService(cconfig_sv_topic_name.str(),
                                  &Behavior::controlConfService,
                                  this);                                
   //############################# 
   
   error = previous_error = 0.0;
   derivative = integral = 0.0;
   cconfig.P = 0.45; cconfig.I = 0.00; cconfig.D = 0.0;
   cconfig.max_linear_velocity = 60;
   cconfig.max_angular_velocity = 60;
   if(!initParameters()) ROS_ERROR("Failed to initialize control parameters/field");
}

bool Behavior::initParameters()
{
   QString home = QString::fromStdString(getenv("HOME"));
   QString cfgDir = home+QString(CCONFIGFOLDERPATH);
   mainfile = cfgDir+QString(MAINFILENAME);
   
   QString fieldname = "";
   QFile file(mainfile);
   if(!file.open(QIODevice::ReadOnly)) {
      ROS_ERROR("Failed to read %s",MAINFILENAME);
      return false;
   }
   QTextStream in(&file);
   fieldname = in.readLine();
   file.close();

   ROS_INFO("Control Parameters for %s Field",fieldname.toStdString().c_str());
   controlparam_file = cfgDir+QString("Robot")+QString::number(robot_id)+"/"+QString(CONTROLFILENAME);
   field_file = cfgDir+QString(FIELDSFOLDERPATH)+fieldname+QString(".view");
   return (readControlParameters()&&readField());
}

bool Behavior::readControlParameters()
{
   QFile file(controlparam_file);
   if(!file.open(QIODevice::ReadOnly)) {
      ROS_ERROR("Failed to read %s.",CONTROLFILENAME);
      return false;
   }
   QTextStream in(&file);
   
   QString line = "";
   while(!in.atEnd()){
      line = in.readLine();
      if(line[0]=='#') continue;
      else {
         QString label_name = line.left(line.indexOf("="));
         QString value = line.right(line.size()-line.indexOf('=')-1);

         if(label_name=="P"){
            cconfig.P = value.toFloat();      
         } else if(label_name=="I"){
            cconfig.I = value.toFloat(); 
         } else if(label_name=="D"){
            cconfig.D = value.toFloat(); 
         } else if(label_name=="lin_max"){
            cconfig.max_linear_velocity = value.toFloat(); 
         } else if(label_name=="ang_max"){
            cconfig.max_angular_velocity = value.toFloat(); 
         } else { ROS_ERROR("Bad Configuration (2) in %s",CONTROLFILENAME); return false; }
      }
    }

   file.close();
   return true;
}

bool Behavior::writeControlParameters()
{
   QFile file(controlparam_file);
   if(!file.open(QIODevice::WriteOnly)){
     ROS_ERROR("Error writing to %s.",CONTROLFILENAME);
     return false;
   }
   QTextStream in(&file);

   in << "P="<< cconfig.P << "\n";
   in << "I="<< cconfig.I << "\n";
   in << "D="<< cconfig.D << "\n";
   in << "lin_max="<< cconfig.max_linear_velocity << "\n";
   in << "ang_max="<< cconfig.max_angular_velocity << "\n"; 

   QString message = QString("#DONT CHANGE THE ORDER OF THE CONFIGURATIONS");
   in << message;                   
   file.close();
   return true;
}

bool Behavior::readField()
{
   // Create view of current Field
   QFile file(field_file);
   if(!file.open(QIODevice::ReadOnly)){
      ROS_ERROR("Error reading field.view.");
      return false;
   }
   QTextStream in(&file);

   QString value; 
   int counter = 0;
   while(!in.atEnd()){
      value = in.readLine();
      field.dimensions[counter] = value.right(value.size()-value.indexOf('=')-1).toInt();
      counter++;
   }
   
   field_length = (double)field.fieldDims.LENGTH/2000.0;
   field_width = (double)field.fieldDims.WIDTH/2000.0;
   return true;
}
bool Behavior::controlConfService(requestControlConfig::Request &req, requestControlConfig::Response &res)
{
   res.config = cconfig;
}

void Behavior::robotInfoCallback(const robotInfo::ConstPtr &msg)
{
   pthread_mutex_lock(&robot_info_mutex); //Lock mutex
   robot_info = *msg;
   pthread_mutex_unlock(&robot_info_mutex); //Unlock mutex
}

void Behavior::aiInfoCallback(const aiInfo::ConstPtr &msg)
{
   pthread_mutex_lock(&ai_info_mutex); //Lock mutex
   ai_info = *msg;
   pthread_mutex_unlock(&ai_info_mutex); //Unlock mutex
}

void Behavior::controlConfigCallback(const controlConfig::ConstPtr &msg)
{
   pthread_mutex_lock(&cconfig_mutex); //Lock mutex
   cconfig = *msg;
   writeControlParameters();
   pthread_mutex_unlock(&cconfig_mutex); //Unlock mutex
}

void Behavior::doWork()
{
   pthread_mutex_lock(&robot_info_mutex); //Lock mutex   
   minho_team_ros::robotInfo robot = robot_info; // Make a copy to hold the mutex for less time
   pthread_mutex_unlock(&robot_info_mutex); //Unlock mutex
   
   pthread_mutex_lock(&ai_info_mutex); //Lock mutex
   minho_team_ros::aiInfo ai = ai_info; // Make a copy to hold the mutex for less time
   pthread_mutex_unlock(&ai_info_mutex); //Unlock mutex
   
   ConstructVoronoiDiagram_WithRectangleLimits(robot,ai);
   /// \brief implementation of Control Sequence
   /// \brief Roles : 0 - GK | 1 - DEFENSE | 2 - ATTACKER | 3 - SUPPORT ATTACKER
   /// \brief Actions : 0 - PARAR | 1 - IR P/ POSIÇÃO | 2 - ATRAS DA BOLA | 3 - CHUTAR/PASSAR
   
   if(ai.action == STOP) control.linear_velocity = control.angular_velocity = 0;
   else { // Implement behaviour based on ai info
  
   }
   control_info_pub.publish(control);
}

// MATH FUNCTIONS
float Behavior::Distance(float positionAX, float positionAY, float positionBX, float positionBY)
{
   float difX = positionBX - positionAX;
   float difY = positionBY - positionAY;

   return sqrt((difX*difX) + (difY*difY));
}

//Calcular ângulo do alvo em graus relativamente ao robô
//return ângulo
int Behavior::PsiTarget(double positionAX, double positionAY, double positionBX, double positionBY, bool normalize)
{
    int psiTargetAngleDegrees=0;

    psiTargetAngleDegrees = (int)((atan2((positionBY-positionAY),(positionBX-positionAX))*180.0)/M_PI);

    if(normalize){
        if(psiTargetAngleDegrees<0)
            psiTargetAngleDegrees +=360;

        if(psiTargetAngleDegrees>=0 && psiTargetAngleDegrees<90)
            psiTargetAngleDegrees = 360+psiTargetAngleDegrees-90;
        else
            psiTargetAngleDegrees -=90;
    }
    
    return psiTargetAngleDegrees;
}

//
int Behavior::RotationRobot_for_Target_PID(int psi_target, int rotational_speed_max)
{
    int robot_angle = robot_info.robot_pose.z;
    int rotational_speed=0;

    if(robot_angle>90 && robot_angle<360)
        robot_angle -=270;
    else
        robot_angle +=90;

    error = (double)(robot_angle - psi_target);

    if(error>180.0)
        error += -360.0;
    else if(error<-180.0)
        error += 360.0;

    derivative = (error-previous_error)/0.03333;
    
    pthread_mutex_lock(&cconfig_mutex); //Lock mutex
    rotational_speed = (int)(cconfig.P*error + cconfig.I*integral + cconfig.D*derivative);
    pthread_mutex_unlock(&cconfig_mutex); //Unlock mutex

    if((error*previous_error)<=0) integral=0.0;
    
    pthread_mutex_lock(&cconfig_mutex); //Lock mutex
    if(rotational_speed>cconfig.max_angular_velocity)
        rotational_speed=cconfig.max_angular_velocity;
    else if(rotational_speed<-cconfig.max_angular_velocity)
        rotational_speed=-cconfig.max_angular_velocity;
    else
        integral += error*0.03333;
    pthread_mutex_unlock(&cconfig_mutex); //Unlock mutex

    previous_error = error;
    
    return rotational_speed;
}

int Behavior::SpeedRobot_TargetDistance_Log(double distanceRobot_Target, int linear_speed_max)
{
    int linear_speed=0;

    pthread_mutex_lock(&cconfig_mutex); //Lock mutex
    linear_speed = (int)(cconfig.max_angular_velocity*log(distanceRobot_Target+1.08));

    if(linear_speed>cconfig.max_linear_velocity)
        linear_speed = cconfig.max_linear_velocity;
    else if(linear_speed<0)
        linear_speed = 0;
    pthread_mutex_unlock(&cconfig_mutex); //Unlock mutex

    return linear_speed;
}

vector<double> Behavior::ConstructVoronoiDiagram_WithRectangleLimits(robotInfo robot, aiInfo ai)
{
    path_data.voronoi.clear();
    vector<double> vectPointsReturn;
    vector<Point_2> pointsAux;
    vector<Point_2> PointsVD;
    int j=0;
    vector<SiteVD> siteVD;
  
    for(unsigned int i=0; i<robot.obstacles.size(); i++){
        siteVD.push_back(SiteVD(robot.obstacles.at(i).x,-robot.obstacles.at(i).y));
    }

    for(double i=-9; i<9.5; i+=0.5){
        siteVD.push_back(SiteVD(i,-6));
    }
    for(double i=-9; i<9.5; i+=0.5){
        siteVD.push_back(SiteVD(i, 6));
    }
    for(double i=-6; i<6.5; i+=0.5){
        siteVD.push_back(SiteVD(-9, i));
    }
    for(double i=-6; i<6.5; i+=0.5){
        siteVD.push_back(SiteVD(9,i));
    }

    VD.insert(siteVD.begin(),siteVD.end());

    Iso_rectangle_2 bbox(-field_length,-field_width,field_length,field_width);
    Cropped_voronoi_from_delaunay vor(bbox);

    delaunayTriangulation = VD.dual();
    delaunayTriangulation.draw_dual(vor);


    for(unsigned int i=0; i<vor.m_cropped_vd.size(); i++){
        //cout<<vor.m_cropped_vd.at(i)<<"\n";
        pointsAux.push_back(vor.m_cropped_vd.at(i).source());
        pointsAux.push_back(vor.m_cropped_vd.at(i).target());

        if(!isnan(pointsAux.at(j).hx()) && !isnan(pointsAux.at(j).hy()) && !isnan(pointsAux.at(j+1).hx()) && !isnan(pointsAux.at(j+1).hy())){
            /*vectPointsReturn.push_back(pointsAux.at(j).hx());
            vectPointsReturn.push_back(-pointsAux.at(j).hy());
            vectPointsReturn.push_back(pointsAux.at(j+1).hx());
            vectPointsReturn.push_back(-pointsAux.at(j+1).hy());*/   
            
            segment temp;
            if(cconfig.send_voronoi){
               temp.ini.x = pointsAux.at(j).hx(); temp.ini.y = -pointsAux.at(j).hy();
               temp.fini.x = pointsAux.at(j+1).hx(); temp.fini.y = -pointsAux.at(j+1).hy();
               path_data.voronoi.push_back(temp);   
            }
        }
        j=j+2;
    }



   if(VD.number_of_faces()>3){
       vector<Vertex> vectVertex;
       Vertex varVertex;
       VertexDescriptor vertexSource;

       //Site mais próximo de robot
       Point_2 sitePoint;
       Point_2 sitePointAux;
       double distRobotToSite;
       SiteVD siteToRobot;
       distRobotToSiteAux=20;

       for(siteIterator sit = VD.sites_begin(); sit != VD.sites_end(); ++sit){
           sitePoint = *sit;
           //cout<<"point: "<<sitePoint<<endl;
           distRobotToSite = Distance(robot.robot_pose.x,robot.robot_pose.y, sitePoint.hx(), -sitePoint.hy()); //cout<<"dist: "<<distRobotToSite<<endl;

           if(distRobotToSite<distRobotToSiteAux){
               siteToRobot = *sit;
               sitePointAux = *sit;
               distRobotToSiteAux = distRobotToSite;

               //Segment_2 segmentRobot_Site(Point_2(robot.robot_pose.x,robot.robot_pose.y), Point_2(sitePointAux.hx(),-sitePointAux.hy()));
           }
       }
       //cout<<"siteToRobot: "<<siteToRobot<<endl;


       int angle = PsiTarget(robot.robot_pose.x, robot.robot_pose.y, sitePointAux.hx(), -sitePointAux.hy(), false);

       vector<Ray_2> rayRobot;
       double xRay,yRay;

       for(int i=1; i<4; i++){

           xRay = cos((double)((angle+(i*90))*(M_PI/180)));
           yRay = sin((double)((angle+(i*90))*(M_PI/180)));

           xRay = xRay + robot.robot_pose.x;
           yRay = yRay + robot.robot_pose.y;

           Ray_2 ray(Point_2(robot.robot_pose.x, robot.robot_pose.y), Point_2(xRay, yRay));
           rayRobot.push_back(ray);
       }


       LocateResult lr = VD.locate(sitePointAux);

       FaceHandle *f = boost::get<FaceHandle>(&lr);

       CcbHalfedgeCirculator ccbHalfedgeCirculator = VD.ccb_halfedges(*f);

       CcbHalfedgeCirculator ccbhc = ccbHalfedgeCirculator;


       CGAL::cpp11::result_of<Kernel::Intersect_2(Ray_2, Segment_2)>::type result;

       Point_2 ps,pt;

       vector<Segment_2> segSourceRobot;


       for(unsigned int i=0; i<rayRobot.size(); i++){

           do{
               if(ccbhc->has_source() && ccbhc->has_target()){

                   ps = ccbhc->source()->point();
                   pt = ccbhc->target()->point();

                   Segment_2 seg(Point_2(ps.hx(), -ps.hy()), Point_2(pt.hx(), -pt.hy()));

                   result = CGAL::intersection(rayRobot.at(i),seg);

                   if(result){
                       if(const Point_2 *p = boost::get<Point_2 >(&*result)){
                           Segment_2 segSourceRobotAux(Point_2(robot.robot_pose.x, -robot.robot_pose.y), Point_2(p->hx(), -p->hy()));

                           /*vectPointsReturn.push_back(robot.robot_pose.x);
                           vectPointsReturn.push_back(robot.robot_pose.y);
                           vectPointsReturn.push_back(p->hx());
                           vectPointsReturn.push_back(p->hy());*/
                           
                           segment temp2;
                           if(cconfig.send_voronoi){
                              temp2.ini.x = robot.robot_pose.x; temp2.ini.y = robot.robot_pose.y;
                              temp2.fini.x = p->hx(); temp2.fini.y = p->hy();
                              path_data.voronoi.push_back(temp2);   
                           }

                           segSourceRobot.push_back(segSourceRobotAux);

                           Segment_2 seg1(Point_2(ps.hx(), ps.hy()), Point_2(p->hx(), -p->hy()));
                           Segment_2 seg2(Point_2(p->hx(), -p->hy()), Point_2(pt.hx(), pt.hy()));
                           segSourceRobot.push_back(seg1);
                           segSourceRobot.push_back(seg2);

                           varVertex.vertexId = boost::add_vertex(graphVoronoi);
                           varVertex.vertexXY = Point_2(p->hx(), -p->hy());

                           vectVertex.push_back(varVertex);
                       }
                   }
               }

           }while(++ccbhc != ccbHalfedgeCirculator);

           ccbhc = ccbHalfedgeCirculator;
       }

       //source
       varVertex.vertexId = boost::add_vertex(graphVoronoi);
       varVertex.vertexXY = Point_2(robot.robot_pose.x, -robot.robot_pose.y);

       vertexSource = varVertex.vertexId;

       vectVertex.push_back(varVertex);

       //Site mais próximo de robot
       Point_2 sitePoint1;
       Point_2 sitePointAux1;
       double distRobotToSite1;
       SiteVD siteToRobot1;
       distRobotToSiteAux=20;

       for(siteIterator sit = VD.sites_begin(); sit != VD.sites_end(); ++sit){
           sitePoint1 = *sit;
           //cout<<"point: "<<sitePoint<<endl;
           distRobotToSite1 = Distance(ai.target_pose.x,ai.target_pose.y, sitePoint1.hx(), -sitePoint1.hy()); //cout<<"dist: "<<distRobotToSite<<endl;

           if(distRobotToSite1<distRobotToSiteAux){
               siteToRobot1 = *sit;
               sitePointAux1 = *sit;
               distRobotToSiteAux = distRobotToSite1;

               //Segment_2 segmentRobot_Site(Point_2(robot.robot_pose.x,robot.robot_pose.y), Point_2(sitePointAux.hx(),-sitePointAux.hy()));
           }
       }

       int angle1 = PsiTarget(ai.target_pose.x, ai.target_pose.y, sitePointAux1.hx(), -sitePointAux1.hy(), false);

       vector<Ray_2> rayRobot1;
       double xRay1,yRay1;

       for(int i=1; i<4; i++){

           xRay1 = cos((double)((angle1+(i*90))*(M_PI/180)));
           yRay1 = sin((double)((angle1+(i*90))*(M_PI/180)));

           xRay1 = xRay1 + ai.target_pose.x;
           yRay1 = yRay1 + ai.target_pose.y;

           Ray_2 ray1(Point_2(ai.target_pose.x, ai.target_pose.y), Point_2(xRay1, yRay1));
           rayRobot1.push_back(ray1);
       }


       LocateResult lr1 = VD.locate(sitePointAux1);

       FaceHandle *f1 = boost::get<FaceHandle>(&lr1);

       CcbHalfedgeCirculator ccbHalfedgeCirculator1 = VD.ccb_halfedges(*f1);

       CcbHalfedgeCirculator ccbhc1 = ccbHalfedgeCirculator1;


       CGAL::cpp11::result_of<Kernel::Intersect_2(Ray_2, Segment_2)>::type result1;

       Point_2 ps1,pt1;

       vector<Segment_2> segSourceRobot1;


       for(unsigned int i=0; i<rayRobot1.size(); i++){

           do{
               if(ccbhc1->has_source() && ccbhc1->has_target()){

                   ps1 = ccbhc1->source()->point();
                   pt1 = ccbhc1->target()->point();

                   Segment_2 seg1(Point_2(ps1.hx(), -ps1.hy()), Point_2(pt1.hx(), -pt1.hy()));

                   result1 = CGAL::intersection(rayRobot1.at(i),seg1);

                   if(result1){
                       if(const Point_2 *p1 = boost::get<Point_2 >(&*result1)){
                           Segment_2 segSourceRobotAux1(Point_2(ai.target_pose.x, -ai.target_pose.y), Point_2(p1->hx(), -p1->hy()));

                           /*vectPointsReturn.push_back(ai.target_pose.x);
                           vectPointsReturn.push_back(ai.target_pose.y);
                           vectPointsReturn.push_back(p1->hx());
                           vectPointsReturn.push_back(p1->hy());*/
                           
                           segment temp3;
                           if(cconfig.send_voronoi){
                              temp3.ini.x = ai.target_pose.x; temp3.ini.y = ai.target_pose.y;
                              temp3.fini.x = p1->hx(); temp3.fini.y = p1->hy();
                              path_data.voronoi.push_back(temp3);   
                           }

                           segSourceRobot1.push_back(segSourceRobotAux1);

                           Segment_2 seg11(Point_2(ps1.hx(), ps1.hy()), Point_2(p1->hx(), -p1->hy()));
                           Segment_2 seg21(Point_2(p1->hx(), -p1->hy()), Point_2(pt1.hx(), pt1.hy()));
                           segSourceRobot1.push_back(seg11);
                           segSourceRobot1.push_back(seg21);

                           varVertex.vertexId = boost::add_vertex(graphVoronoi);
                           varVertex.vertexXY = Point_2(p1->hx(), -p1->hy());

                           vectVertex.push_back(varVertex);
                       }
                   }
               }

           }while(++ccbhc1 != ccbHalfedgeCirculator1);

           ccbhc1 = ccbHalfedgeCirculator1;
       }



       //target
       VertexDescriptor vertexTarget;
       varVertex.vertexId = boost::add_vertex(graphVoronoi);
       varVertex.vertexXY = Point_2(ai.target_pose.x, -ai.target_pose.y);
       vertexTarget = varVertex.vertexId;
       vectVertex.push_back(varVertex);

       for(vertexIterator vit = VD.vertices_begin(); vit != VD.vertices_end(); ++vit){

           varVertex.vertexId = boost::add_vertex(graphVoronoi);
           varVertex.vertexXY = vit->point();

           vectVertex.push_back(varVertex);
       }

       //para o source
       Segment_2 aux1;
       VertexDescriptor vertex00, vertex11;
       EdgeWeightProperty edgeWeight1;
       Point_2 pointSource1, pointTarget1;

       for(unsigned int j=0; j<segSourceRobot.size(); j++){
           aux1 = segSourceRobot.at(j);

           for(unsigned int i=0; i<vectVertex.size(); i++){

               if(aux1.source() == vectVertex.at(i).vertexXY){
                   vertex00 = vectVertex.at(i).vertexId;
                   pointSource1 = vectVertex.at(i).vertexXY;
               }
               if(aux1.target() == vectVertex.at(i).vertexXY){
                   vertex11 = vectVertex.at(i).vertexId;
                   pointTarget1 = vectVertex.at(i).vertexXY;
               }
           }

           edgeWeight1 = (float)Distance(pointSource1.hx(), pointSource1.hy(), pointTarget1.hx(), pointTarget1.hy());

           boost::add_edge(vertex00, vertex11, edgeWeight1, graphVoronoi);

       }


       //para o target
       Segment_2 aux11;
       VertexDescriptor vertex001, vertex111;
       EdgeWeightProperty edgeWeight11;
       Point_2 pointSource11, pointTarget11;

       for(unsigned int j=0; j<segSourceRobot1.size(); j++){
           aux11 = segSourceRobot1.at(j);

           for(unsigned int i=0; i<vectVertex.size(); i++){

               if(aux11.source() == vectVertex.at(i).vertexXY){
                   vertex001 = vectVertex.at(i).vertexId;
                   pointSource11 = vectVertex.at(i).vertexXY;
               }
               if(aux11.target() == vectVertex.at(i).vertexXY){
                   vertex111 = vectVertex.at(i).vertexId;
                   pointTarget11 = vectVertex.at(i).vertexXY;
               }
           }

           edgeWeight11 = (float)Distance(pointSource11.hx(), pointSource11.hy(), pointTarget11.hx(), pointTarget11.hy());

           boost::add_edge(vertex001, vertex111, edgeWeight11, graphVoronoi);

       }

       VertexDescriptor vertex0, vertex1;
       EdgeWeightProperty edgeWeight;
       Point_2 pointSource, pointTarget;


       for(edgeIterator vit = VD.edges_begin(); vit != VD.edges_end(); ++vit){

           if(vit->has_source() && vit->has_target()){

               for(unsigned int i=0; i<vectVertex.size(); i++){

                   if(vit->source()->point() == vectVertex.at(i).vertexXY){
                       vertex0 = vectVertex.at(i).vertexId;
                       pointSource = vectVertex.at(i).vertexXY;
                   }
                   if(vit->target()->point() == vectVertex.at(i).vertexXY){
                       vertex1 = vectVertex.at(i).vertexId;
                       pointTarget = vectVertex.at(i).vertexXY;
                   }
               }

               edgeWeight = (float)Distance(pointSource.hx(), pointSource.hy(), pointTarget.hx(), pointTarget.hy());

               boost::add_edge(vertex0, vertex1, edgeWeight, graphVoronoi);
           }
       }


       vector<VertexDescriptor> parents(boost::num_vertices(graphVoronoi));
       vector<float> distances(boost::num_vertices(graphVoronoi));


       boost::dijkstra_shortest_paths(graphVoronoi, vertexSource, boost::predecessor_map(&parents[0]).distance_map(&distances[0]));
       boost::graph_traits<Graph>::vertex_iterator VertexIterator, Vend;


       for(boost::tie(VertexIterator, Vend) = boost::vertices(graphVoronoi); VertexIterator != Vend; ++VertexIterator)
       {
           /*cout<< "distance(" << *VertexIterator << ") = " << distances[*VertexIterator] << ", ";
           cout<< "parent(" << *VertexIterator << ") = " << parents[*VertexIterator] <<endl;*/

       }
       //cout<<endl;


       //vector<double> vectPathPoint;
       path_data.path.clear();
       if(cconfig.send_path){
          position temp4;
          for(VertexDescriptor path_point = vertexTarget; path_point != parents[path_point]; path_point = parents[path_point]){
              /*vectPathPoint.push_back(vectVertex.at(path_point).vertexXY.hx());
              vectPathPoint.push_back(-vectVertex.at(path_point).vertexXY.hy());*/
              temp4.x = vectVertex.at(path_point).vertexXY.hx(); temp4.y = -vectVertex.at(path_point).vertexXY.hy();
              path_data.path.push_back(temp4);  
          }

          /*vectPathPoint.push_back(vectVertex.at(vertexSource).vertexXY.hx());
          vectPathPoint.push_back(-vectVertex.at(vertexSource).vertexXY.hy());*/
          temp4.x = vectVertex.at(vertexSource).vertexXY.hx(); temp4.y = -vectVertex.at(vertexSource).vertexXY.hy();
          path_data.path.push_back(temp4); 
         
          /*sendStringPath.clear();
          for(unsigned int i=0; i<vectPathPoint.size(); i++){
              sendStringPath += ";"+QString::number(vectPathPoint.at(i));
          }*/
       }

   }
   
   if(cconfig.send_voronoi || cconfig.send_path) {
      path_data_pub.publish(path_data);
   }

   graphVoronoi.clear();
   delaunayTriangulation.clear();
   VD.clear();
   vor.m_cropped_vd.clear();  //???
   return vectPointsReturn;  

}

