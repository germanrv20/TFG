#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "mavros_msgs/State.h"
#include "gazebo_msgs/ModelStates.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <vector>
#include <string>
#include <cmath> // Para M_PI

using namespace Eigen;

// ==========================
// ESTRUCTURAS Y VARIABLES
// ==========================
struct Pose {
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
};

// ==========================
// VARIABLES GLOBALES
// ==========================

mavros_msgs::State current_state;
Pose initial_global_pose; 
bool initial_global_pose_received = false;

// --- VARIABLES DINÁMICAS ---
geometry_msgs::Pose P_G_current_msg; 
geometry_msgs::PoseStamped P_L_current_msg; 
bool current_poses_ready = false; 

// ==========================
// CALLBACKS
// ==========================

void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}

// CALLBACK GLOBAL (Actualización Continua para Anclaje Dinámico)
void gazebo_cb(const gazebo_msgs::ModelStates::ConstPtr& msg) {
    for (size_t i = 0; i < msg->name.size(); i++) {
        if (msg->name[i] == "drone2") {
            // --- 1. Actualización Continua del Ancla Global ---
            P_G_current_msg = msg->pose[i]; 
            
            // --- 2. Captura Única de 'base_pos' (para inicialización) ---
            if (!initial_global_pose_received && (P_G_current_msg.position.x != 0 || P_G_current_msg.position.y != 0 || P_G_current_msg.position.z != 0)) {
                initial_global_pose.position = Eigen::Vector3d(P_G_current_msg.position.x, P_G_current_msg.position.y, P_G_current_msg.position.z);
                initial_global_pose.orientation = Eigen::Quaterniond(P_G_current_msg.orientation.w, P_G_current_msg.orientation.x, P_G_current_msg.orientation.y, P_G_current_msg.orientation.z);
                initial_global_pose_received = true;
                ROS_INFO("Posición GLOBAL inicial capturada (Base).");
            }
            return;
        }
    }
}

// CALLBACK LOCAL (Actualización Continua del Ancla Local)
void local_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    P_L_current_msg = *msg;
    if (!current_poses_ready) {
        current_poses_ready = true;
        ROS_INFO("Posición LOCAL (MAVROS) recibida.");
    }
}


//------------------------- OPERACIONES MATRICES ------------------------------

Eigen::Matrix4d pose_to_transform(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation) {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3,3>(0,0) = orientation.toRotationMatrix();
    T.block<3,1>(0,3) = position;
    return T;
}

/**
 * @brief Convierte una pose del sistema global al local usando la matriz de transformación.
 */
Pose global_to_local_pose(const Pose& global_ref, const Pose& local_ref, const Pose& global_input) {
    
    // 1. Crear las matrices de anclaje (P_G_current y P_L_current)
    Eigen::Matrix4d T_global_ref = pose_to_transform(global_ref.position, global_ref.orientation);
    Eigen::Matrix4d T_local_ref = pose_to_transform(local_ref.position, local_ref.orientation);

    // 2. Crear la matriz del punto objetivo
    Eigen::Matrix4d T_global_input = pose_to_transform(global_input.position, global_input.orientation);

    // 3. Calcular la matriz de transformación T_L_from_G = T_L_ref * inv(T_G_ref)
    Eigen::Matrix4d T_L_from_G = T_local_ref * T_global_ref.inverse();

    // 4. Aplicar la transformación T_local_output = T_L_from_G * T_global_input
    Eigen::Matrix4d T_local_output = T_L_from_G * T_global_input;

    // Extraer y devolver
    Pose result;
    result.position = T_local_output.block<3,1>(0,3);
    result.orientation = Eigen::Quaterniond(T_local_output.block<3,3>(0,0));
    return result;
}

// -------------------------------------------------------------------------


int main(int argc, char** argv) 
{
    ros::init(argc, argv, "drone2_dynamic_lag_test_node"); 
    ros::NodeHandle nh;

    // Suscriptores y Publicadores
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/drone2/mavros/state", 10, state_cb);
    ros::Subscriber gazebo_sub = nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 10, gazebo_cb);
    ros::Subscriber local_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/drone2/mavros/local_position/pose", 10, local_pose_cb);
    
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/drone2/mavros/setpoint_position/local", 10);
    
    ros::Rate rate(20.0); // Bucle principal a 20Hz

    // 1. Esperar conexión con FCU
    ROS_INFO("Esperando conexión con FCU de drone2...");
    while (ros::ok() && !current_state.connected) {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("FCU drone2 conectado");

    // 2. Esperar las poses iniciales y dinámicas
    ROS_INFO("Esperando datos base (Global) y datos continuos (Local)...");
    while (ros::ok() && (!initial_global_pose_received || !current_poses_ready)) {
        // Para que el EKF se inicie, a veces necesita ver setpoints
        geometry_msgs::PoseStamped dummy_pose;
        local_pos_pub.publish(dummy_pose);
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("¡Datos base recibidos! Iniciando prueba de lag dinámica.");

    // ===================================================
    // 3. Definición de Waypoints (Octágono Lento)
    // ===================================================
    int num_points = 8; 
    double test_radius = 3.0;
    double test_z = 7.0; 
    Eigen::Vector3d polygon_center(0.0, 0.0, test_z);

    std::vector<Eigen::Vector3d> global_target_points;

    ROS_INFO("Calculando 8 puntos del polígono para prueba de lag...");
    for (int i = 0; i < num_points; ++i) {
        double angle = 2.0 * M_PI * i / num_points;
        
        Eigen::Vector3d wp_pos(
            polygon_center.x() + test_radius * cos(angle),
            polygon_center.y() + test_radius * sin(angle),
            polygon_center.z()
        );
        global_target_points.push_back(wp_pos);
    }
    global_target_points.push_back(global_target_points[0]); 

    // Los local_waypoints se calcularán dinámicamente en el bucle principal.
    // Solo necesitamos las posiciones globales.
    
    // 6. Enviar stream de setpoints iniciales (para OFFBOARD)
    // Usamos el primer punto del octágono para inicializar
    geometry_msgs::PoseStamped pose_msg;
    Pose initial_global_input;
    initial_global_input.position = global_target_points[0];
    initial_global_input.orientation = initial_global_pose.orientation;
    
    // Usamos las anclas dinámicas actuales para calcular el primer setpoint