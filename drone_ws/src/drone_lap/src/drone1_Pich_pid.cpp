#include "ros/ros.h"
#include "geometry_msgs/TwistStamped.h" 
#include "geometry_msgs/PointStamped.h" 
#include "mavros_msgs/State.h"
#include <cmath>
#include <algorithm> 

// -----------------------------------------
// Variables Globales
// -----------------------------------------
mavros_msgs::State current_state;
double current_vertical_error = 0.0; // Error en Y (e_y)
const double PIXEL_SENTINEL = -999.0; 

// --- TUNING DEL PID (PITCH) ---
// Si el dron 2 está arriba, el error Y es positivo.
// Necesitamos Pitch positivo (nariz arriba) para corregir.
const double Kp_pitch = 0.15;   // Un poco menos agresivo
const double Ki_pitch = 0.015;  // Mantiene la corrección de error constante
const double Kd_pitch = 0.019;  // REDUCIDO: Menos sensible al ruido
const double MAX_PITCH_RATE = 0.5; // rad/s

// Clase PID (La misma de siempre)
class PIDController {
public:
    double Kp, Ki, Kd;
    double integral_error;
    double prev_error;
    ros::Time last_time;

    PIDController(double kp, double ki, double kd) : Kp(kp), Ki(ki), Kd(kd), integral_error(0.0), prev_error(0.0) {}

    double compute(double current_error) {
        ros::Time current_time = ros::Time::now();
        if (last_time.isZero()) { last_time = current_time; return 0.0; }
        
        double dt = (current_time - last_time).toSec();
        if (dt < 0.001) return 0.0; 

        double proportional = Kp * current_error;
        integral_error += current_error * dt;
        double integral = Ki * integral_error;
        double derivative = Kd * (current_error - prev_error) / dt;

        if (std::abs(current_error) < 1.0) integral_error = 0.0;

        prev_error = current_error;
        last_time = current_time;

        return proportional + integral + derivative;
    }
};

// -----------------------------------------
// Callbacks
// -----------------------------------------

void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}

void error_cb(const geometry_msgs::PointStamped::ConstPtr& msg) {
    // LEEMOS EL ERROR 'Y' (Vertical)
    // El nodo de visión debe estar publicando error_y en point.y
    current_vertical_error = msg->point.y; 
}

// -----------------------------------------
// Main
// -----------------------------------------

int main(int argc, char** argv)
{
    ros::init(argc, argv, "drone1_pitch_pid_node");
    ros::NodeHandle nh;

    PIDController pitch_pid(Kp_pitch, Ki_pitch, Kd_pitch);

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/drone1/mavros/state", 10, state_cb);
    ros::Subscriber error_sub = nh.subscribe<geometry_msgs::PointStamped>("/drone1/vision_error", 10, error_cb); 
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::TwistStamped>("/drone1/mavros/setpoint_velocity/cmd_vel", 10);
    
    ros::Rate rate(20.0);

    // Esperar conexión...
    while (ros::ok() && !current_state.connected) {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("FCU Conectado. Iniciando control de Pitch.");

    geometry_msgs::TwistStamped twist_msg; 
    
    while (ros::ok()) {
        
        double cmd_pitch = 0.0;
        
        // Usamos el error Y. Si el nodo de visión pone 0.0 cuando pierde, 
        // necesitamos saber si es "0 real" o "0 perdido".
        // Asumiremos que tu nodo de visión sigue enviando -999 en X si se pierde.
        // Pero aquí simplificamos: Si Y es exactamente 0.0 y X es -999, no hacemos nada.
        // Mejor: Tu nodo de visión debe poner Y = -999 si se pierde.
        
        if (current_vertical_error > PIXEL_SENTINEL + 1.0) 
        {
            cmd_pitch = pitch_pid.compute(current_vertical_error);
            cmd_pitch = std::min(std::max(cmd_pitch, -MAX_PITCH_RATE), MAX_PITCH_RATE);
        } else {
            cmd_pitch = 0.0; 
        }
        
        twist_msg.header.stamp = ros::Time::now(); 
        twist_msg.header.frame_id = "base_link"; 
        
        twist_msg.twist.linear.x = 0.0;
        twist_msg.twist.linear.y = 0.0;
        twist_msg.twist.linear.z = 0.0; 

        twist_msg.twist.angular.x = 0.0; // Roll
        twist_msg.twist.angular.y = cmd_pitch; // <--- AQUÍ ESTÁ EL CONTROL (PITCH)
        twist_msg.twist.angular.z = 0.0; // Yaw

        vel_pub.publish(twist_msg); 

        ROS_INFO_THROTTLE(0.5, "Err Y: %.1f px -> Pitch: %.3f rad/s", current_vertical_error, cmd_pitch);

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}