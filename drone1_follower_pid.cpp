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
ros::Publisher vel_pub; // Hacemos el publicador global para usarlo en el callback

// Parámetros PID
const double Kp_yaw = 0.015;  
const double Ki_yaw = 0.0002;
const double Kd_yaw = 0.002;
const double MAX_ANGULAR_VEL = 1.0; 
const double PIXEL_SENTINEL = -999.0;

// -----------------------------------------
// Clase PID (Optimizada)
// -----------------------------------------
class PIDController {
public:
    double Kp, Ki, Kd;
    double integral_error;
    double prev_error;
    ros::Time last_time;

    PIDController(double kp, double ki, double kd) : Kp(kp), Ki(ki), Kd(kd), integral_error(0.0), prev_error(0.0) {}

    double compute(double current_error) {
        ros::Time current_time = ros::Time::now();
        
        if (last_time.isZero()) {
            last_time = current_time;
            return 0.0; 
        }
        
        double dt = (current_time - last_time).toSec();

        // Protección contra dt muy pequeños (si llega una ráfaga de mensajes)
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

// Instancia Global del PID
PIDController yaw_pid(Kp_yaw, Ki_yaw, Kd_yaw);

// -----------------------------------------
// Callbacks
// -----------------------------------------

void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}

void error_cb(const geometry_msgs::PointStamped::ConstPtr& msg) {
    double current_error = msg->point.x;
    
    // Failsafe: Si el dron no está listo, no hacemos nada
    if (!current_state.connected || !current_state.armed || (current_state.mode != "GUIDED" && current_state.mode != "POSITION")) {
        return;
    }

    double commanded_omega_z = 0.0;
    
    // 1. Lógica de Control
    if (current_error > PIXEL_SENTINEL) {
        // Objetivo Visible -> Calcular PID
        commanded_omega_z = yaw_pid.compute(current_error);
        commanded_omega_z = std::min(std::max(commanded_omega_z, -MAX_ANGULAR_VEL), MAX_ANGULAR_VEL);
    } else {
        commanded_omega_z = 0.0; 
    }

    // 2. Publicar INMEDIATAMENTE
    geometry_msgs::TwistStamped twist_msg; 
    twist_msg.header.stamp = ros::Time::now(); 
    twist_msg.header.frame_id = "base_link"; 
    
    twist_msg.twist.linear.x = 0.0; // Solo giro
    twist_msg.twist.angular.z = commanded_omega_z; 

    vel_pub.publish(twist_msg); 

    // Log (opcional, throttled para no saturar consola)
    ROS_INFO_THROTTLE(0.5, "REACTIVO -> Error x : %.1f | Cmd: %.3f", current_error, commanded_omega_z);
}

// -----------------------------------------
// Main
// -----------------------------------------

int main(int argc, char** argv)
{
    ros::init(argc, argv, "drone1_follower_pid_node");
    ros::NodeHandle nh;

    // Inicializamos el publicador globalmente
    vel_pub = nh.advertise<geometry_msgs::TwistStamped>("/drone1/mavros/setpoint_velocity/cmd_vel", 10);

    // Suscriptores
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/drone1/mavros/state", 10, state_cb);
    
    // IMPORTANTE: queue_size=1 para procesar solo el dato más reciente posible
    // transport_hints().tcpNoDelay() reduce la latencia de red
    ros::Subscriber error_sub = nh.subscribe("/drone1/vision_error", 1, error_cb, ros::TransportHints().tcpNoDelay()); 

    ROS_INFO("Nodo PID Reactivo Iniciado. Esperando datos...");

    // ros::spin() mantiene el programa vivo y ejecuta los callbacks en cuanto llegan datos
    ros::spin();

    return 0;
}