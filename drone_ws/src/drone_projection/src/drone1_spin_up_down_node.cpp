#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "mavros_msgs/State.h"
#include <vector>
#include <cmath>

// Variables globales
mavros_msgs::State current_state;

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

struct Waypoint {
    double x, y, z, yaw;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "drone1_spin_up_down_node");
    ros::NodeHandle nh;

    // Suscriptor al estado del dron
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/drone1/mavros/state", 10, state_cb);

    // Publicador de setpoints
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/drone1/mavros/setpoint_position/local", 10);

    ros::Rate rate(20.0);

    // Esperar a que el FCU se conecte
    ROS_INFO("Esperando conexión con FCU...");
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("FCU conectado");

    // Definir trayectoria (subidas y bajadas con giros)
    std::vector<Waypoint> waypoints;
    Waypoint wp;
    double base_alt = 5.0;

    wp.x = 0; wp.y = 0; wp.z = base_alt; wp.yaw = 0; waypoints.push_back(wp);
    wp.z = 7; wp.yaw = M_PI/2; waypoints.push_back(wp);
    wp.z = 3; wp.yaw = M_PI; waypoints.push_back(wp);
    wp.z = 6; wp.yaw = 3*M_PI/2; waypoints.push_back(wp);
    wp.z = 5; wp.yaw = 0; waypoints.push_back(wp);

    int wp_index = 0;

    // Publicar algunos setpoints antes de entrar en modo OFFBOARD
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = waypoints[0].x;
    pose.pose.position.y = waypoints[0].y;
    pose.pose.position.z = waypoints[0].z;
    pose.pose.orientation.w = 1.0;

    for(int i=0; i<100 && ros::ok(); i++){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("Iniciando trayectoria...");

    // Loop principal
    while(ros::ok()){
        ros::spinOnce();

        // Actualizar pose objetivo
        pose.pose.position.x = waypoints[wp_index].x;
        pose.pose.position.y = waypoints[wp_index].y;
        pose.pose.position.z = waypoints[wp_index].z;

        // Convertir yaw a quaternion
        double cy = cos(waypoints[wp_index].yaw * 0.5);
        double sy = sin(waypoints[wp_index].yaw * 0.5);
        pose.pose.orientation.w = cy;
        pose.pose.orientation.x = 0;
        pose.pose.orientation.y = 0;
        pose.pose.orientation.z = sy;

        local_pos_pub.publish(pose);

        // Cambio de waypoint cada ~5s
        static int counter = 0;
        counter++;
        if(counter > 100){
            counter = 0;
            wp_index = (wp_index + 1) % waypoints.size();
        }

        rate.sleep();
    }

    return 0;
}
