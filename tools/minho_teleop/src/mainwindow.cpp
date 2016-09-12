#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(int robot_id, QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    teleop_activated_ = false;
    robot_id_ = robot_id;
    _update_ = new QTimer();
    dribblers_state_ = false;
    kick_request_ = is_pass_ = false;
    thrust_.resize(7);
    thrust_activation_.resize(7);
    connect(_update_,SIGNAL(timeout()),this,SLOT(onUpdate()));
    connect(_update_,SIGNAL(timeout()),this,SLOT(updateThrusts()));
    ui->hs_lin->setValue(50);
    ui->hs_ang->setValue(50);
    ui->lb_robot_name->setStyleSheet("QLabel { color : red; }");
    ui->centralWidget->setFocusPolicy(Qt::StrongFocus);
    ui->lb_robot_name->setText(QString("Robot ")+QString::number(robot_id_));
    
    // Setup ROS Node and pusblishers/subscribers
    QString asd = "Teleop";
    asd.append(QString::number(robot_id));
    std::stringstream control_topic;
    control_topic << "minho_gazebo_robot" << std::to_string(robot_id) << "/controlInfo";
    std::stringstream teleop_topic;
    teleop_topic << "minho_gazebo_robot" << std::to_string(robot_id) << "/teleop";
    std::stringstream robot_topic;
    robot_topic << "minho_gazebo_robot" << std::to_string(robot_id) << "/robotInfo";
    
    //Initialize ROS
    int argc = 0;
	ros::init(argc, NULL, asd.toStdString().c_str(),ros::init_options::NoSigintHandler);
    _node_ = new ros::NodeHandle();
    control_pub_ = _node_->advertise<controlInfo>(control_topic.str().c_str(),100);
    teleop_pub_ = _node_->advertise<teleop>(teleop_topic.str().c_str(),100);
	//Initialize controlInfo subscriber
	robot_sub_ = _node_->subscribe(robot_topic.str().c_str(), 1000, &MainWindow::robotInfoCallback, this);
    spinner = new ros::AsyncSpinner(2);
    spinner->start();
    
    _update_->start(30); // update teleop data to robot
}

MainWindow::~MainWindow()
{
    on_pushButton_2_clicked();
    ros::shutdown();    
    delete ui;
}

void MainWindow::on_pushButton_clicked()
{
    teleop_activated_ = true;
    ui->lb_robot_name->setStyleSheet("QLabel { color : green; }");
    kick_request_ = false;
    //Notify teleop state change
    minho_team_ros::teleop msg;
    msg.set_teleop = true;
    if(teleop_pub_) teleop_pub_.publish(msg);
}

void MainWindow::on_pushButton_2_clicked()
{
    teleop_activated_ = false;
    ui->lb_robot_name->setStyleSheet("QLabel { color : red; }");
    //Notify teleop state change
    minho_team_ros::teleop msg;
    msg.set_teleop = false;
    if(teleop_pub_) teleop_pub_.publish(msg);
}

void MainWindow::on_hs_lin_valueChanged(int value)
{
    ui->lb_lin->setText(QString::number(value));
    max_lin_ = value;
}

void MainWindow::on_hs_ang_valueChanged(int value)
{
    ui->lb_ang->setText(QString::number(value));
    max_ang_ = value;
}

void MainWindow::onUpdate()
{
    if(teleop_activated_){
        minho_team_ros::controlInfo msg;
        // publish data over ROS
        double sumY = thrust_[0]-thrust_[2];
        double sumX = thrust_[3]-thrust_[1];
        int velocity = sqrt(sumX*sumX+sumY*sumY);
        if(velocity>max_lin_) velocity = max_lin_;
        double direction = atan2(sumY,sumX)*(180.0/M_PI)-90;
        while(direction>360) direction-=360;
        while(direction<0) direction+=360;
        double angSum = thrust_[5]-thrust_[6];

        if(kick_request_){
            // send kick
            msg.kick_strength = thrust_[4];
            if(is_pass_) msg.kick_is_pass = is_pass_;
            thrust_[4] = 0;
            kick_request_ = is_pass_ = false;
        }
        
        // publish stuff
        msg.linear_velocity = velocity;
        msg.movement_direction = (int)direction;
        msg.angular_velocity = (int)angSum;
        msg.is_teleop = true;
        msg.dribbler_on = dribblers_state_;
        if(control_pub_) control_pub_.publish(msg);
    }
}

void MainWindow::updateThrusts()
{
    if(teleop_activated_){
        if(thrust_activation_[0]) { thrust_[0] += 3; if(thrust_[0]>max_lin_) thrust_[0]=max_lin_; }
        else { thrust_[0] -= 5; if(thrust_[0]<0) thrust_[0]=0; }
        if(thrust_activation_[1]) { thrust_[1] += 3; if(thrust_[1]>max_lin_) thrust_[1]=max_lin_; }
        else { thrust_[1] -= 5; if(thrust_[1]<0) thrust_[1]=0; }
        if(thrust_activation_[2]) { thrust_[2] += 3; if(thrust_[2]>max_lin_) thrust_[2]=max_lin_; }
        else { thrust_[2] -= 5; if(thrust_[2]<0) thrust_[2]=0; }
        if(thrust_activation_[3]) { thrust_[3] += 3; if(thrust_[3]>max_lin_) thrust_[3]=max_lin_; }
        else { thrust_[3] -= 5; if(thrust_[3]<0) thrust_[3]=0; }
        if(thrust_activation_[4]) { thrust_[4] += 5; if(thrust_[4]>100) thrust_[4]=100; }
        if(thrust_activation_[5]) { thrust_[5] += 1; if(thrust_[5]>max_ang_) thrust_[5]=max_ang_; }
        else { thrust_[5] -= 5; if(thrust_[5]<0) thrust_[5]=0; }
        if(thrust_activation_[6]) { thrust_[6] += 1; if(thrust_[6]>max_ang_) thrust_[6]=max_ang_; }
        else { thrust_[6] -= 5; if(thrust_[6]<0) thrust_[6]=0; }
    }
}

void MainWindow::keyPressEvent(QKeyEvent *event)
{
    if(!event->isAutoRepeat()){
        switch(event->key()){
            case 'W':{  //front
                thrust_activation_[0] = true;
                break;
            }
            case 'A':{  //left
                thrust_activation_[1] = true;
                break;
            }
            case 'S':{  //back
                thrust_activation_[2] = true;
                break;
            }
            case 'D':{  //right
                thrust_activation_[3] = true;
                break;
            }
            case 'Q':{  //turn left
                thrust_activation_[5] = true;
                break;
            }
            case 'E':{  //turn right
                thrust_activation_[6] = true;
                break;
            }
            case 'P':{  //pass
                thrust_activation_[4] = true;
                is_pass_ = true;
                break;
            }
            case 'K':{  //kick
                thrust_activation_[4] = true;
                is_pass_ = false;
                break;
            }
            case 'L':{  //dribblers on/off
                dribblers_state_ = !dribblers_state_;
                break;
            }
            case ' ':{  //dribblers on/off
                for(int i = 0;i<7;i++) { thrust_activation_[i] = false; thrust_[i] = 0; }
                break;
            }
            case 'T':{  //turn on/off
                if(teleop_activated_){
                    on_pushButton_2_clicked();
                } else {
                    on_pushButton_clicked();
                }
                break;
            }
        }
    }
}

void MainWindow::keyReleaseEvent(QKeyEvent *event)
{
    if(!event->isAutoRepeat()){
        switch(event->key()){
            case 'W':{  //front
                thrust_activation_[0] = false;
                break;
            }
            case 'A':{  //left
                thrust_activation_[1] = false;
                break;
            }
            case 'S':{  //back
                thrust_activation_[2] = false;
                break;
            }
            case 'D':{  //right
                thrust_activation_[3] = false;
                break;
            }
            case 'Q':{  //turn left
                thrust_activation_[5] = false;
                break;
            }
            case 'E':{  //turn right
                thrust_activation_[6] = false;
                break;
            }
            case 'P':{  //pass
                thrust_activation_[4] = false;
                kick_request_ = true;
                break;
            }
            case 'K':{  //kick
                thrust_activation_[4] = false;
                kick_request_ = true;
                break;
            }
        }
    }
}

void MainWindow::robotInfoCallback(const minho_team_ros::robotInfo::ConstPtr& msg)
{
    QString info = QString("[")+QString::number(msg->robot_pose.x,'f',2) + 
                   QString(",")+QString::number(msg->robot_pose.y,'f',2) +
                   QString(",")+QString::number(msg->imu_value) +
                   QString("] : Has Ball -> ") + QString::number(msg->has_ball);
                   
   ui->lb_pose->setText(info);
}
