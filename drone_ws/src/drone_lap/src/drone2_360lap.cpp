#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "mavros_msgs/State.h"
#include "gazebo_msgs/ModelStates.h"
#include <Eigen/Dense>
#include <vector>
#include <string>

using namespace Eigen;
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Geometry>

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

// Pose Inicial (Solo para orientación y base del círculo, se captura una vez)
Pose initial_global_pose; 
bool initial_global_pose_received = false;

// Variables para el anclaje dinámico (lectura continua)
geometry_msgs::Pose P_G_current_msg; 
geometry_msgs::PoseStamped P_L_current_msg; 
bool current_poses_ready = false;


// ==========================
// CALLBACKS
// ==========================

void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}

// CALLBACK GLOBAL (Continuo)
void gazebo_cb(const gazebo_msgs::ModelStates::ConstPtr& msg) {
    for (size_t i = 0; i < msg->name.size(); i++) {
        if (msg->name[i] == "drone2") {
            // --- 1. Lectura Continua ---
            P_G_current_msg = msg->pose[i]; 
            
            // --- 2. Captura Inicial (Base) ---
            if (!initial_global_pose_received && (P_G_current_msg.position.x != 0 || P_G_current_msg.position.y != 0 || P_G_current_msg.position.z != 0)) {
                initial_global_pose.position = Eigen::Vector3d(P_G_current_msg.position.x, P_G_current_msg.position.y, P_G_current_msg.position.z);
                initial_global_pose.orientation = Eigen::Quaterniond(P_G_current_msg.orientation.w, P_G_current_msg.orientation.x, P_G_current_msg.orientation.y, P_G_current_msg.orientation.z);
                initial_global_pose_received = true;
                ROS_INFO("Posición GLOBAL (Gazebo) inicial capturada.");
            }
            return;
        }
    }
}

// CALLBACK LOCAL (Continuo)
void local_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    P_L_current_msg = *msg;
    // La bandera se pone en true al recibir el primer dato
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

Pose global_to_local_pose(const Pose& global_ref, const Pose& local_ref, const Pose& global_input) {
    
    Eigen::Matrix4d T_global_ref = pose_to_transform(global_ref.position, global_ref.orientation);
    Eigen::Matrix4d T_local_ref = pose_to_transform(local_ref.position, local_ref.orientation);
    Eigen::Matrix4d T_global_input = pose_to_transform(global_input.position, global_input.orientation);

    // T_L_from_G = T_L_ref * inv(T_G_ref)
    Eigen::Matrix4d T_L_from_G = T_local_ref * T_global_ref.inverse();

    // T_L_output = T_L_from_G * T_G_input
    Eigen::Matrix4d T_local_output = T_L_from_G * T_global_input;

    // Extraer posición y rotación
    Eigen::Vector3d pos_local = T_local_output.block<3,1>(0,3);
    Eigen::Matrix3d rot_matrix_local = T_local_output.block<3,3>(0,0);
    Eigen::Quaterniond rot_local(rot_matrix_local);

    Pose result;
    result.position = pos_local;
    result.orientation = rot_local;
    return result;
}

// -------------------------------------------------------------------------


int main(int argc, char** argv) 
{
    ros::init(argc, argv, "drone2_dynamic_control_node");
    ros::NodeHandle nh;

    // Suscriptores y Publicadores
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/drone2/mavros/state", 10, state_cb);
    ros::Subscriber gazebo_sub = nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 10, gazebo_cb);
    // --- SUSCRIPTOR LOCAL ---
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

    // 2. Esperar todas las poses iniciales
    ROS_INFO("Esperando datos iniciales de Global (Gazebo) y Local (MAVROS)...");
    while (ros::ok() && (!initial_global_pose_received || !current_poses_ready)) {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("¡Datos de anclaje iniciales recibidos! Iniciando control dinámico.");


    // ----------------------------------------------------------------------
    // 3. Parámetros de la Trayectoria Circular (Mismo plan que antes)
    // ----------------------------------------------------------------------
    double radius = 3.0;
    double speed = 0.5;
    double z_global = 7;
    double omega = speed / radius;
    double t = 0.0;
    double dt = 1.0 / 20.0;
    Eigen::Vector3d circle_center_global(0.0, 0.0, z_global);

    ROS_INFO("Iniciando movimiento circular dinámico ");

    // El resto de la inicialización de pose_msg es igual...
    geometry_msgs::PoseStamped pose_msg;

    // ===================================================
    // 7. Bucle principal de control (Generación continua con anclaje dinámico)
    // ===================================================
    while (ros::ok()) {

        // ANCLAS DINÁMICAS 

        Pose P_G_current_eigen; 
        P_G_current_eigen.position = Eigen::Vector3d(P_G_current_msg.position.x, P_G_current_msg.position.y, P_G_current_msg.position.z);
        P_G_current_eigen.orientation = Eigen::Quaterniond(P_G_current_msg.orientation.w, P_G_current_msg.orientation.x, P_G_current_msg.orientation.y, P_G_current_msg.orientation.z);

        Pose P_L_current_eigen;
        P_L_current_eigen.position = Eigen::Vector3d(P_L_current_msg.pose.position.x, P_L_current_msg.pose.position.y, P_L_current_msg.pose.position.z);
        P_L_current_eigen.orientation = Eigen::Quaterniond(P_L_current_msg.pose.orientation.w, P_L_current_msg.pose.orientation.x, P_L_current_msg.pose.orientation.y, P_L_current_msg.pose.orientation.z);


        // 2. DEFINIR poses actuales
        Pose pose_global_ref = P_G_current_eigen;
        Pose pose_local_ref  = P_L_current_eigen;

        // 3. Incremento y Cálculo de la Trayectoria
        t += dt;

        // Calcular nueva pose GLOBAL deseada
        Pose global_input;
        global_input.position = Eigen::Vector3d(
            circle_center_global.x() + radius * cos(omega * t),
            circle_center_global.y() + radius * sin(omega * t),
            circle_center_global.z()
        );
        // La orientación sigue siendo fija, determinada por la orientación inicial
        global_input.orientation = initial_global_pose.orientation; 

        // 4. Transformar a pose LOCAL (con la transformación dinámica)
        Pose local_target = global_to_local_pose(pose_global_ref, pose_local_ref, global_input);

        // 5. Rellenar y Publicar
        pose_msg.pose.position.x = local_target.position.x();
        pose_msg.pose.position.y = local_target.position.y();
        pose_msg.pose.position.z = local_target.position.z();
        
        pose_msg.pose.orientation.w = local_target.orientation.w();
        pose_msg.pose.orientation.x = local_target.orientation.x();
        pose_msg.pose.orientation.y = local_target.orientation.y();
        pose_msg.pose.orientation.z = local_target.orientation.z();

        ROS_INFO_THROTTLE(1.0, 
            "Target Local: [x: %.2f, y: %.2f, z: %.2f]",
            local_target.position.x(),
            local_target.position.y(),
            local_target.position.z());

        local_pos_pub.publish(pose_msg);

        ros::spinOnce();
        rate.sleep();
    }
return 0;
}