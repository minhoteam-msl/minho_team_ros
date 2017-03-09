#include "fundamental.h"

Fundamental::Fundamental()
{
    total_field_length = 19.8; total_field_width = 14.8;
    field_length = 18.0; field_width = 12.0;
    robot_diameter = 0.5;
    if(!initParameters()) ROS_ERROR("Failed to initialize field dimensions");

    outside_field_length = (total_field_length - field_length)/2.0;
    outside_field_width = (total_field_width - field_width)/2.0;
    half_field_length = field_length/2.0;
    half_field_width = field_width/2.0;


}

//
bool Fundamental::initParameters()
{
    QString home = QString::fromStdString(getenv("HOME"));
    QString commonDir = home+QString(COMMON_PATH);
    QString cfgDir = commonDir+QString(CTRL_CFG_PATH);
    QString fieldsDir = commonDir+QString(FIELDS_PATH);
    QString mainFile = commonDir+QString(MAINFILENAME);

    QString fieldname = "";
    QFile file(mainFile);
    if(!file.open(QIODevice::ReadOnly)) {
      ROS_ERROR("Failed to read %s",MAINFILENAME);
      return false;
    }
    QTextStream in(&file);
    fieldname = in.readLine();
    file.close();

    ROS_INFO("Dimensions for %s Field",fieldname.toStdString().c_str());
    field_file = fieldsDir+fieldname+QString(".view");

    return readField();
}

//
bool Fundamental::readField()
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

    total_field_length = (float)field.fieldDims.TOTAL_LENGTH/1000.0;
    total_field_width = (float)field.fieldDims.TOTAL_WIDTH/1000.0;
    field_length = (float)field.fieldDims.LENGTH/1000.0;
    field_width = (float)field.fieldDims.WIDTH/1000.0;
    robot_diameter = (float)field.fieldDims.ROBOT_DIAMETER/1000.0;

    return true;
}



//Field axis for MinhoTeam and robot angle
/*
 *       180
 *        *                  |
 *                           |
 * 90 *       * 270     -----+-----X
 *                           |
 *        *                  |
 *        0                   Y
 *                        ______
 *                       |RefBox|
 */



//
float Fundamental::distance(float pointAx, float pointAy, float pointBx, float pointBy)
{
    return sqrt((pointBx-pointAx)*(pointBx-pointAx) + (pointBy-pointAy)*(pointBy-pointAy));
}

//
float Fundamental::distance_without_sqrt(float pointAx, float pointAy, float pointBx, float pointBy)
{
    return (pointBx-pointAx)*(pointBx-pointAx) + (pointBy-pointAy)*(pointBy-pointAy);
}

//
float Fundamental::cartesian2polar_angleRad(float pointAx, float pointAy, float pointBx, float pointBy)
{
    return atan2((pointBy-pointAy),(pointBx-pointAx));
}

//
int Fundamental::cartesian2polar_angleDeg(float pointAx, float pointAy, float pointBx, float pointBy)
{
    return (int)((atan2((pointBy-pointAy),(pointBx-pointAx))*180.0)/M_PI);
}

//
float Fundamental::cartesian2polar_angleRadNormalize(float pointAx, float pointAy, float pointBx, float pointBy)
{
    float angleRad;

    angleRad = atan2((pointBy-pointAy),(pointBx-pointAx));

    if(angleRad<0.0)
        angleRad += 2.0*M_PI;
    if(angleRad>=0.0 && angleRad<M_PI_2)
        angleRad = 2.0*M_PI+angleRad-M_PI_2;
    else
        angleRad -= M_PI_2;

    return angleRad;
}

//
int Fundamental::cartesian2polar_angleDegNormalize(float pointAx, float pointAy, float pointBx, float pointBy)
{
    int angleDegrees;

    angleDegrees = (int)((atan2((pointBy-pointAy),(pointBx-pointAx))*180.0)/M_PI);

    if(angleDegrees<0)
        angleDegrees += 360;
    if(angleDegrees>=0 && angleDegrees<90)
        angleDegrees = 360+angleDegrees-90;
    else
        angleDegrees -= 90;

    return angleDegrees;
}

//Field axis for MinhoTeam and return.....
/*
 *       180
 *        *                  |
 *                           |
 * 90 *       * -90     -----+-----X
 *                           |
 *        *                  |
 *        0                   Y
 *                        ______
 *                       |RefBox|
 */
//
int Fundamental::cartesian2polar_angleDeg_halfCircle(float pointAx, float pointAy, float pointBx, float pointBy)
{
    return (int)((atan2((pointAx-pointBx),(pointBy-pointAy))*180.0)/M_PI);
}

//
float Fundamental::getFieldLength()
{
    return field_length;
}
//
float Fundamental::getFieldWidth()
{
    return field_width;
}
//
float Fundamental::getTotalFieldLength()
{
    return total_field_length;
}
//
float Fundamental::getTotalFieldWidth()
{
    return total_field_width;
}
//
float Fundamental::getOutsideFieldLength()
{
    return outside_field_length;
}
//
float Fundamental::getOutsideFieldWidth()
{
    return outside_field_width;
}
//
float Fundamental::getHalfFieldLength()
{
    return half_field_length;
}
//
float Fundamental::getHalfFieldWidth()
{
    return half_field_width;
}
//
float Fundamental::getRobotDiameter()
{
    return robot_diameter;
}

//
/*
 * seg           -
 * circle_center -
 * radius        -
 *
 */
//
bool Fundamental::intersect_Segment_and_Circle(Segment seg, Point circle_center, float radius)
{
    float x_min, x_max, y_min, y_max;
    float x_min_seg, x_max_seg, y_min_seg, y_max_seg;
    bool intersect;
    float seg_point_dist;
    Point intersect_point;

    if(seg.source().x() == seg.target().x()) {
        x_min = (float)seg.source().x() - radius;
        x_max = (float)seg.source().x() + radius;

        if(seg.source().y() < seg.target().y()) {
            y_min = (float)seg.source().y() - radius;
            y_max = (float)seg.target().y() + radius;
            y_min_seg = (float)seg.source().y();
            y_max_seg = (float)seg.target().y();
        }else {
            y_min = (float)seg.target().y() - radius;
            y_max = (float)seg.source().y() + radius;
            y_min_seg = (float)seg.target().y();
            y_max_seg = (float)seg.source().y();
        }

        if(circle_center.x() >= x_min && circle_center.x() <= x_max && circle_center.y() >= y_min && circle_center.y() <= y_max) {

            if(circle_center.y() >= y_min_seg && circle_center.y() <= y_max_seg) {
                intersect = true;
            }else if(circle_center.y() < y_min_seg) {
                seg_point_dist = distance((float)seg.source().x(), y_min_seg, (float)circle_center.x(), (float)circle_center.y());
                if(seg_point_dist <= radius)
                    intersect = true;
                else
                    intersect = false;
            }else if(circle_center.y() > y_max_seg) {
                seg_point_dist = distance((float)seg.source().x(), y_max_seg, (float)circle_center.x(), (float)circle_center.y());
                if(seg_point_dist <= radius)
                    intersect = true;
                else
                    intersect = false;
            }
        }else
            intersect = false;

    }else if(seg.source().y() == seg.target().y()) {

        y_min = (float)seg.source().y() - radius;
        y_max = (float)seg.source().y() + radius;

        if(seg.source().x() < seg.target().x()) {
            x_min = (float)seg.source().x() - radius;
            x_max = (float)seg.target().x() + radius;
            x_min_seg = (float)seg.source().x();
            x_max_seg = (float)seg.target().x();
        }else {
            x_min = (float)seg.target().x() - radius;
            x_max = (float)seg.source().x() + radius;
            x_min_seg = (float)seg.target().x();
            x_max_seg = (float)seg.source().x();
        }

        if(circle_center.x() >= x_min && circle_center.x() <= x_max && circle_center.y() >= y_min && circle_center.y() <= y_max) {

            if(circle_center.x() >= x_min_seg && circle_center.x() <= x_max_seg) {
                intersect = true;
            }else if(circle_center.x() < x_min_seg) {
                seg_point_dist = distance(x_min_seg, (float)seg.source().y(), (float)circle_center.x(), (float)circle_center.y());
                if(seg_point_dist <= radius)
                    intersect = true;
                else
                    intersect = false;
            }else if(circle_center.x() > x_max_seg) {
                seg_point_dist = distance(x_max_seg, (float)seg.source().y(), (float)circle_center.x(), (float)circle_center.y());
                if(seg_point_dist <= radius)
                    intersect = true;
                else
                    intersect = false;
            }
        }else
            intersect = false;

    }else {
        if(seg.source().x() < seg.target().x()) {
            x_min = (float)seg.source().x() - radius;
            x_max = (float)seg.target().x() + radius;
            x_min_seg = (float)seg.source().x();
            x_max_seg = (float)seg.target().x();
        }else {
            x_min = (float)seg.target().x() - radius;
            x_max = (float)seg.source().x() + radius;
            x_min_seg = (float)seg.target().x();
            x_max_seg = (float)seg.source().x();
        }

        if(seg.source().y() < seg.target().y()) {
            y_min = (float)seg.source().y() - radius;
            y_max = (float)seg.target().y() + radius;
            y_min_seg = (float)seg.source().y();
            y_max_seg = (float)seg.target().y();
        }else {
            y_min = (float)seg.target().y() - radius;
            y_max = (float)seg.source().y() + radius;
            y_min_seg = (float)seg.target().y();
            y_max_seg = (float)seg.source().y();
        }

        if(circle_center.x() >= x_min && circle_center.x() <= x_max && circle_center.y() >= y_min && circle_center.y() <= y_max) {

            float seg_source_dist = distance((float)seg.source().x(), (float)seg.source().y(), (float)circle_center.x(), (float)circle_center.y());
            float seg_target_dist = distance((float)seg.target().x(), (float)seg.target().y(), (float)circle_center.x(), (float)circle_center.y());

            if(seg_source_dist < radius && seg_target_dist < radius)
                intersect = true;
            else {
                Circle circle(Circ_Point(circle_center.x(), circle_center.y()), CGAL::Exact_rational(radius * radius), CGAL::CLOCKWISE);
                Circ_Line circ_line(Circ_Point(seg.source().x(), seg.source().y()), Circ_Point(seg.target().x(), seg.target().y()));

                vector<InterRes> output;
                Dispatcher disp = CGAL::dispatch_output<InterRes>(back_inserter(output));

                CGAL::intersection(circ_line, circle, disp);

                if(output.size() > 0) {
                    intersect_point = Point(CGAL::to_double(output[0].first.x()), CGAL::to_double(output[0].first.y()));
                    if(intersect_point.x() >= x_min_seg && intersect_point.x() <= x_max_seg &&
                            intersect_point.y() >= y_min_seg && intersect_point.y() <= y_max_seg)
                        intersect = true;
                    else if(output.size() > 1) {
                        intersect_point = Point(CGAL::to_double(output[1].first.x()), CGAL::to_double(output[1].first.y()));
                        if(intersect_point.x() >= x_min_seg && intersect_point.x() <= x_max_seg &&
                                intersect_point.y() >= y_min_seg && intersect_point.y() <= y_max_seg)
                            intersect = true;
                        else
                            intersect = false;
                    }else
                        intersect = false;
                }else
                    intersect = false;
            }
        }else
            intersect = false;
    }

    return intersect;
}

//
/*
 * seg                 -
 * circle_center       -
 * radius              -
 * intersection_points -
 */
//
bool Fundamental::intersect_Segment_and_Circle(Segment seg, Point circle_center, float radius, vector<Point>& intersection_points)
{
    float x_min, x_max, y_min, y_max;
    float x_min_seg, x_max_seg, y_min_seg, y_max_seg;
    bool intersect, intersect0, intersect1;
    Point intersect_point;

    if(seg.source().x() <= seg.target().x()) {
        x_min = (float)seg.source().x() - radius;
        x_max = (float)seg.target().x() + radius;
        x_min_seg = (float)seg.source().x();
        x_max_seg = (float)seg.target().x();
    }else {
        x_min = (float)seg.target().x() - radius;
        x_max = (float)seg.source().x() + radius;
        x_min_seg = (float)seg.target().x();
        x_max_seg = (float)seg.source().x();
    }

    if(seg.source().y() <= seg.target().y()) {
        y_min = (float)seg.source().y() - radius;
        y_max = (float)seg.target().y() + radius;
        y_min_seg = (float)seg.source().y();
        y_max_seg = (float)seg.target().y();
    }else {
        y_min = (float)seg.target().y() - radius;
        y_max = (float)seg.source().y() + radius;
        y_min_seg = (float)seg.target().y();
        y_max_seg = (float)seg.source().y();
    }

    if(circle_center.x() >= x_min && circle_center.x() <= x_max && circle_center.y() >= y_min && circle_center.y() <= y_max) {

        float seg_source_dist = distance((float)seg.source().x(), (float)seg.source().y(), (float)circle_center.x(), (float)circle_center.y());
        float seg_target_dist = distance((float)seg.target().x(), (float)seg.target().y(), (float)circle_center.x(), (float)circle_center.y());

        if(seg_source_dist < radius && seg_target_dist < radius)
            intersect = true;
        else {
            Circle circle(Circ_Point(circle_center.x(), circle_center.y()), CGAL::Exact_rational(radius * radius), CGAL::CLOCKWISE);
            Circ_Line circ_line(Circ_Point(seg.source().x(), seg.source().y()), Circ_Point(seg.target().x(), seg.target().y()));

            vector<InterRes> output;
            Dispatcher disp = CGAL::dispatch_output<InterRes>(back_inserter(output));

            CGAL::intersection(circ_line, circle, disp);

            if(output.size() > 0) {
                intersect_point = Point(CGAL::to_double(output[0].first.x()), CGAL::to_double(output[0].first.y()));

                if(intersect_point.x() >= x_min_seg && intersect_point.x() <= x_max_seg &&
                        intersect_point.y() >= y_min_seg && intersect_point.y() <= y_max_seg) {
                    intersect0 = true;
                    intersection_points.push_back(intersect_point);
                }else
                    intersect0 = false;
            }else
                intersect0 = false;

            if(output.size() > 1) {
                intersect_point = Point(CGAL::to_double(output[1].first.x()), CGAL::to_double(output[1].first.y()));

                if(intersect_point.x() >= x_min_seg && intersect_point.x() <= x_max_seg &&
                        intersect_point.y() >= y_min_seg && intersect_point.y() <= y_max_seg) {
                    intersect1 = true;
                    intersection_points.push_back(intersect_point);
                }else
                    intersect1 = false;
            }else
                intersect1 = false;

            if(intersect0 || intersect1)
                intersect = true;
            else
                intersect = false;
        }
    }else
        intersect = false;

    return intersect;
}

