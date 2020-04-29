#include <stdio.h>

#include <QPainter>
#include <QLineEdit>
#include <QComboBox>
#include <QPushButton>
#include <QString>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>
#include <QDebug>
#include <QWidget>
#include "teleop_pad.h"
#include <geometry_msgs/Twist.h>
#include <rfans_driver/RfansCommand.h>
#include <ros/ros.h>

namespace rviz_teleop_commander
{


TeleopPanel::TeleopPanel( QWidget* parent )
  : rviz::Panel( parent )
  , linear_velocity_( 0 )
  , angular_velocity_( 0 )
{


  QVBoxLayout* topic_layout = new QVBoxLayout;


  //create a combox of scan speed.
  topic_layout->addWidget( new QLabel( "Scan Speed:" ));
  scan_speed = new QComboBox;
  scan_speed->clear();
  scan_speed->addItem("5",0);
  scan_speed->addItem("10",1);
  scan_speed->addItem("20",2);
  int rps =0;
  ros::param::get("/rfans_driver/rps",rps);
  if(rps == 5){
      scan_speed->setCurrentIndex(0);
  } else if(rps == 10){
      scan_speed->setCurrentIndex(1);
  } else if(rps == 20){
      scan_speed->setCurrentIndex(2);
  } else {
      scan_speed->setCurrentIndex(1);
  }
  topic_layout->addWidget( scan_speed );

  topic_layout->addWidget( new QLabel( "Return Type:" ));
  return_type = new QComboBox;
  return_type->clear();
  return_type->addItem(tr("Strongest return"),0);
  return_type->addItem(tr("Dual return"),1);
  bool double_echo =false;
  ros::param::get("/rfans_driver/use_double_echo",double_echo);
  if(double_echo){
      return_type->setCurrentIndex(1);
  } else {
      return_type->setCurrentIndex(0);
  }
  topic_layout->addWidget( return_type );
  button_ok = new QPushButton;
  button_ok->setText("OK");
  button_ok->setEnabled(true);
  topic_layout->addWidget(button_ok);

  QHBoxLayout* layout = new QHBoxLayout;
  layout->addLayout( topic_layout );
  setLayout( layout );


  connect(button_ok, SIGNAL(clicked()), this, SLOT(button_clicked()));


}


void TeleopPanel::button_clicked(){
    ros::ServiceClient client = nh_.serviceClient<rfans_driver::RfansCommand>("rfans_driver/rfans_control");
    rfans_driver::RfansCommand srv;
    srv.request.cmd = 1;
    srv.request.speed = scan_speed->currentText().toInt();
    if(return_type->currentIndex() ==0){
        srv.request.use_double_echo = false;
    } else {
        srv.request.use_double_echo = true;
    }
    button_ok->setEnabled(false);
    if(client.call(srv)){
        if(srv.response.status == 1){
            button_ok->setEnabled(true);
        }
    }
}

void TeleopPanel::update_Linear_Velocity()
{

    QString temp_string = output_topic_editor_1->text();
    float lin = temp_string.toFloat();  
    linear_velocity_ = lin;
}

void TeleopPanel::update_Angular_Velocity()
{
    QString temp_string = output_topic_editor_2->text();
    float ang = temp_string.toFloat() ;  
    angular_velocity_ = ang;
}

void TeleopPanel::updateTopic()
{
  setTopic( output_topic_editor_->text() );
}


void TeleopPanel::setTopic( const QString& new_topic )
{

  if( new_topic != output_topic_ )
  {
    output_topic_ = new_topic;
    if( output_topic_ == "" )
    {
      velocity_publisher_.shutdown();
    }
    else
    {
      velocity_publisher_ = nh_.advertise<geometry_msgs::Twist>( output_topic_.toStdString(), 1 );
    }
    Q_EMIT configChanged();
  }
}


void TeleopPanel::sendVel()
{
  if( ros::ok() && velocity_publisher_ )
  {
    geometry_msgs::Twist msg;
    msg.linear.x = linear_velocity_;
    msg.linear.y = 0;
    msg.linear.z = 0;
    msg.angular.x = 0;
    msg.angular.y = 0;
    msg.angular.z = angular_velocity_;
    velocity_publisher_.publish( msg );
  }
}

void TeleopPanel::save( rviz::Config config ) const
{
  rviz::Panel::save( config );
  config.mapSetValue( "Topic", output_topic_ );
}

void TeleopPanel::load( const rviz::Config& config )
{
  rviz::Panel::load( config );
  QString topic;
  if( config.mapGetString( "Topic", &topic ))
  {
    output_topic_editor_->setText( topic );
    updateTopic();
  }
}

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_teleop_commander::TeleopPanel,rviz::Panel )
// END_TUTORIAL
