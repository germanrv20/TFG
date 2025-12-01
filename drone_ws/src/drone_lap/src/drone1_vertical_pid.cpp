#include "ros/ros.h"
#include "geometry_msgs/TwistStamped.h" 
#include "geometry_msgs/PointStamped.h" 
#include "mavros_msgs/State.h"
#include <cmath>
#include <algorithm> 

// Variables Globales
mavros_msgs::State current_state;
ros::Publisher vel_pub;



const double Kp_vert = 0.15;   // Un poco menos agresivo
const double Ki_vert = 0.015;  // Mantiene la corrección de error constante
const double Kd_vert = 0.019;  // REDUCIDO: Menos sensible al ruido
const double MAX_VERT_VEL = 1.5; // Aumentamos ligeramente el límite de velocidad (1.5 m/s)
const double PIXEL_SENTINEL = -999.0;

// Clase PID (Reutilizada)
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
        if (dt < 0.001) return 0.0; 

        double proportional = Kp * current_error;
        integral_error += current_error * dt;
        double integral = Ki * integral_error;
        double derivative = Kd * (current_error - prev_error) / dt;

        // Anti-windup para altura
        if (std::abs(current_error) < 2.0) integral_error = 0.0;

        prev_error = current_error;
        last_time = current_time;

        return proportional + integral + derivative;
    }
};

PIDController vert_pid(Kp_vert, Ki_vert, Kd_vert);

void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}

// CALLBACK DE ERROR
void error_cb(const geometry_msgs::PointStamped::ConstPtr& msg) {
    // --- CAMBIO CLAVE: Usamos msg->point.y (Error Vertical) ---
    double current_error_y = msg->point.y; 
    double current_error_x = msg->point.x; // Podríamos usarlo para Yaw a la vez, pero nos centraremos en Z.
    
    if (!current_state.connected || !current_state.armed || (current_state.mode != "GUIDED" && current_state.mode != "POSITION")) return;

    double commanded_vel_z = 0.0;
    
    // Verificamos si el objetivo es visible (usando el centinela en X, que es la bandera principal)
    // Asumimos que si X es válido, Y también lo es.
    if (current_error_x > PIXEL_SENTINEL) {
        
        // --- LÓGICA DE SIGNOS ---
        // En imagen: Y crece hacia abajo. 
        // Si Drone2 sube -> 'v' disminuye -> error_y (cy - v) se vuelve POSITIVO.
        // En ROS: Velocidad Z positiva es SUBIR.
        // Por tanto: Error Positivo (+Y) implica comando Positivo (+Z). ¡Los signos coinciden!
        
        commanded_vel_z = vert_pid.compute(current_error_y);
        
        // Limitador
        commanded_vel_z = std::min(std::max(commanded_vel_z, -MAX_VERT_VEL), MAX_VERT_VEL);

    } else {
        commanded_vel_z = 0.0; // Si se pierde, mantener altura (o aterrizar suavemente)
    }

    // Publicar
    geometry_msgs::TwistStamped twist_msg; 
    twist_msg.header.stamp = ros::Time::now(); 
    twist_msg.header.frame_id = "base_link"; 
    
    twist_msg.twist.linear.x = 0.0; 
    twist_msg.twist.linear.y = 0.0;
    twist_msg.twist.linear.z = commanded_vel_z; // <--- CONTROL DE ALTURA
    twist_msg.twist.angular.z = 0.0; 

    vel_pub.publish(twist_msg); 

    ROS_INFO_THROTTLE(0.5, "PID VERTICAL -> Err Y: %.1f | Cmd Z: %.2f m/s", current_error_y, commanded_vel_z);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "drone1_vertical_pid_node");
    ros::NodeHandle nh;

    vel_pub = nh.advertise<geometry_msgs::TwistStamped>("/drone1/mavros/setpoint_velocity/cmd_vel", 10);

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/drone1/mavros/state", 10, state_cb);
    
    // Suscribirse al error de visión
    ros::Subscriber error_sub = nh.subscribe("/drone1/vision_error", 1, error_cb, ros::TransportHints().tcpNoDelay()); 

    ROS_INFO("Nodo PID Vertical (Z) Iniciado.");
    ros::spin();

    return 0;
}