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
Pose initial_global_pose; // Para 'base_pos'
bool initial_global_pose_received = false;


// ==========================
// CALLBACKS
// ==========================

void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}

// Callback de Gazebo 
void gazebo_cb(const gazebo_msgs::ModelStates::ConstPtr& msg) {
    // Si ya la hemos capturado, no hacemos nada más
    if (initial_global_pose_received) return;

    for (size_t i = 0; i < msg->name.size(); i++) {
        if (msg->name[i] == "drone2") {
            const geometry_msgs::Pose& pose = msg->pose[i];

            // --- Captura única para 'base_pos' ---
            if (pose.position.x != 0 || pose.position.y != 0 || pose.position.z != 0) {
                initial_global_pose.position = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
                initial_global_pose.orientation = Eigen::Quaterniond(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
                initial_global_pose_received = true;
                ROS_INFO("Posición GLOBAL (Gazebo) inicial capturada (para 'base_pos').");
            }
            // ------------------------------------
            return; // Encontramos drone2, salimos del bucle
        }
    }
}



//------------------------- OPERACIONES MATRICES ------------------------------

/**
 * @brief Convierte posición + cuaternión a matriz de transformación 4x4.
 * Idéntico a la función de Python.
 */
Eigen::Matrix4d pose_to_transform(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation) {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3,3>(0,0) = orientation.toRotationMatrix();
    T.block<3,1>(0,3) = position;
    return T;
}

/**
 * @brief Convierte una pose (posición + orientación) del sistema global al local.
 * Utiliza la lógica matemática corregida.
 */
Pose global_to_local_pose(const Pose& global_ref, const Pose& local_ref, const Pose& global_input) {
    
    // T_global_ref = pose_to_transform(...)
    Eigen::Matrix4d T_global_ref = pose_to_transform(global_ref.position, global_ref.orientation);
    
    // T_local_ref = pose_to_transform(...)
    Eigen::Matrix4d T_local_ref = pose_to_transform(local_ref.position, local_ref.orientation);

    // T_global_input = pose_to_transform(...)
    Eigen::Matrix4d T_global_input = pose_to_transform(global_input.position, global_input.orientation);

    // --- INICIO DE LA CORRECCIÓN ---

    // 1. Calcular la transformación del frame Global (G) al frame Local (L)
    // T_L_from_G = T_L_ref * inv(T_G_ref)
    Eigen::Matrix4d T_L_from_G = T_local_ref * T_global_ref.inverse();

    // 2. Aplicar esa transformación a la pose de entrada global
    // T_L_output = T_L_from_G * T_G_input
    Eigen::Matrix4d T_local_output = T_L_from_G * T_global_input;

    // --- FIN DE LA CORRECCIÓN ---

    // Extraer posición y rotación
    Eigen::Vector3d pos_local = T_local_output.block<3,1>(0,3);
    Eigen::Matrix3d rot_matrix_local = T_local_output.block<3,3>(0,0);
    Eigen::Quaterniond rot_local(rot_matrix_local);

    // Devolver la pose resultante
    Pose result;
    result.position = pos_local;
    result.orientation = rot_local;
    return result;
}

// -------------------------------------------------------------------------


int main(int argc, char** argv) 
{
    ros::init(argc, argv, "drone2_vertical_square_node");
    ros::NodeHandle nh;

    // Suscriptores y Publicadores
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/drone2/mavros/state", 10, state_cb);
    ros::Subscriber gazebo_sub = nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 10, gazebo_cb);
    
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/drone2/mavros/setpoint_position/local", 10);
    
    ros::Rate rate(20.0); // Bucle principal a 20Hz

    // 1. Esperar conexión con FCU
    ROS_INFO("Esperando conexión con FCU de drone2...");
    while (ros::ok() && !current_state.connected) {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("FCU drone2 conectado");

    // 2. Esperar solo la pose GLOBAL inicial (para la 'base_pos')
    ROS_INFO("Esperando pose GLOBAL inicial (para 'base_pos')...");
    while (ros::ok() && !initial_global_pose_received) {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("¡Pose global recibida! Calculando waypoints...");

    // ===================================================
    // 3. Definir las poses de referencia (Anclas FIJAS)
    // ===================================================
    ROS_WARN("Usando anclas de transformación FIJAS (hardcodeadas).");
    Pose pose_global_ref;
    pose_global_ref.position = Eigen::Vector3d(21.398502945035325, 12.194162540459388, 5.261217341198849);
    pose_global_ref.orientation = Eigen::Quaterniond(0.99996020734504,     // w
                                                     -8.289961205614862e-05,  // x
                                                     -0.00014614435009022207, // y
                                                     0.0089193887653896);    // z

    Pose pose_local_ref;
    pose_local_ref.position = Eigen::Vector3d(-12.205732345581055, 11.376830101013184, 5.561599254608154);
    pose_local_ref.orientation = Eigen::Quaterniond(-0.7073207304381668,   // w
                                                    -0.0028507966048823092, // x
                                                    0.00039973697517648105,  // y
                                                    -0.706886905708039);   // z


    // ===================================================
    // 4. Definir Waypoints GLOBALES (Lógica corregida)
    // ===================================================
    std::vector<Eigen::Vector3d> global_target_points;
    Eigen::Vector3d base_pos = initial_global_pose.position; // Base es la pos global
    double square_size = 2.0; // Lado de 2 metros

    // Definimos los movimientos según tus instrucciones
    double move_y_right = square_size;      // Derecha 2m
    double move_z_up = square_size * 2;     // Arriba 4m
    double move_y_left = square_size * 2;   // Izquierda 4m
    double move_z_down = square_size * 2;   // Abajo 4m

    // --- Calculamos los waypoints absolutos (relativos al mundo) ---
    
    // 1. Punto inicial
    ROS_INFO("Calculando Waypoints Globales...");
    global_target_points.push_back(base_pos); 
    ROS_INFO("  WP 1 (Global): [x: %.2f, y: %.2f, z: %.2f]", base_pos.x(), base_pos.y(), base_pos.z());

    // 2. Punto 2 (Derecha 5m)
    Eigen::Vector3d wp2(base_pos.x(), base_pos.y() + move_y_right, base_pos.z());
    global_target_points.push_back(wp2);
    ROS_INFO("  WP 2 (Global): [x: %.2f, y: %.2f, z: %.2f]", wp2.x(), wp2.y(), wp2.z());

    // 3. Punto 3 (Desde WP2, Arriba 10m)
    Eigen::Vector3d wp3(wp2.x(), wp2.y(), wp2.z() + move_z_up);
    global_target_points.push_back(wp3);
    ROS_INFO("  WP 3 (Global): [x: %.2f, y: %.2f, z: %.2f]", wp3.x(), wp3.y(), wp3.z());

    // 4. Punto 4 (Desde WP3, Izquierda 10m)
    Eigen::Vector3d wp4(wp3.x(), wp3.y() - move_y_left, wp3.z());
    global_target_points.push_back(wp4);
    ROS_INFO("  WP 4 (Global): [x: %.2f, y: %.2f, z: %.2f]", wp4.x(), wp4.y(), wp4.z());

    // 5. Punto 5 (Desde WP4, Abajo 10m)
    Eigen::Vector3d wp5(wp4.x(), wp4.y(), wp4.z() - move_z_down);
    global_target_points.push_back(wp5);
    ROS_INFO("  WP 5 (Global): [x: %.2f, y: %.2f, z: %.2f]", wp5.x(), wp5.y(), wp5.z());

    // 6. Vuelta al inicio
    global_target_points.push_back(base_pos); 
    ROS_INFO("  WP 6 (Global): [x: %.2f, y: %.2f, z: %.2f]", base_pos.x(), base_pos.y(), base_pos.z());
    // ===================================================

    // 5. Convertir todos los Waypoints GLOBALES a Waypoints LOCALES
    std::vector<Pose> local_waypoints;
    for (const auto& gp : global_target_points) {
        Pose global_input;
        global_input.position = gp;
        // Usamos la *misma orientación global inicial* para todos, como pediste
        global_input.orientation = initial_global_pose.orientation; 
        
        // Esta función AHORA SÍ funciona correctamente
        local_waypoints.push_back(global_to_local_pose(pose_global_ref, pose_local_ref, global_input));
    }

    ROS_INFO("%lu waypoints locales han sido calculados:", local_waypoints.size());
    for(size_t i = 0; i < local_waypoints.size(); ++i) {
        ROS_INFO("  WP %zu (Local): Pos [x: %.2f, y: %.2f, z: %.2f] | Quat [x: %.3f, y: %.3f, z: %.3f, w: %.3f]",
                 i,
                 local_waypoints[i].position.x(),
                 local_waypoints[i].position.y(),
                 local_waypoints[i].position.z(),
                 local_waypoints[i].orientation.x(),
                 local_waypoints[i].orientation.y(),
                 local_waypoints[i].orientation.z(),
                 local_waypoints[i].orientation.w());
    }
    
    // 6. Enviar stream de setpoints iniciales (necesario para OFFBOARD)
    geometry_msgs::PoseStamped pose_msg;
    // Empezamos con el waypoint 0, que AHORA SÍ es la pos/orientación local correcta
    pose_msg.pose.position.x = local_waypoints[0].position.x();
    pose_msg.pose.position.y = local_waypoints[0].position.y();
    pose_msg.pose.position.z = local_waypoints[0].position.z();
    pose_msg.pose.orientation.w = local_waypoints[0].orientation.w();
    pose_msg.pose.orientation.x = local_waypoints[0].orientation.x();
    pose_msg.pose.orientation.y = local_waypoints[0].orientation.y();
    pose_msg.pose.orientation.z = local_waypoints[0].orientation.z();
    
    ROS_INFO("Enviando setpoints iniciales para habilitar OFFBOARD...");
    ROS_WARN("¡ASEGÚRATE DE ARMAR Y CAMBIAR A MODO OFFBOARD MANUALMENTE!");
    
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose_msg);
        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("Iniciando movimiento...");

    int wp_index = 0;
    int counter = 0;
    int waypoint_duration =250; // 50 / 20Hz = 2.5 segundos por waypoint

    // --- NUEVO: Contador para imprimir poses ---
    int print_counter = 0;
    int print_rate = 20; // Imprimir cada 20 ciclos (aprox 1 vez por segundo)

    // 7. Bucle principal de control
    while (ros::ok()) {
        
        // Asignamos el waypoint local actual
        pose_msg.pose.position.x = local_waypoints[wp_index].position.x();
        pose_msg.pose.position.y = local_waypoints[wp_index].position.y();
        pose_msg.pose.position.z = local_waypoints[wp_index].position.z();
        
        // Asignamos la orientación local calculada
        pose_msg.pose.orientation.w = local_waypoints[wp_index].orientation.w();
        pose_msg.pose.orientation.x = local_waypoints[wp_index].orientation.x();
        pose_msg.pose.orientation.y = local_waypoints[wp_index].orientation.y();
        pose_msg.pose.orientation.z = local_waypoints[wp_index].orientation.z();

        local_pos_pub.publish(pose_msg);

        // Lógica simple para cambiar de waypoint
        counter++;
        if (counter > waypoint_duration) {
            counter = 0;
            wp_index = (wp_index + 1) % local_waypoints.size();
            ROS_INFO("Cambiando al waypoint local #%d", wp_index);

            ROS_INFO("  -> Enviando Coords Locales:");
            ROS_INFO("     Pos: [x: %.2f, y: %.2f, z: %.2f]",
                     local_waypoints[wp_index].position.x(),
                     local_waypoints[wp_index].position.y(),
                     local_waypoints[wp_index].position.z());
            ROS_INFO("     Quat: [x: %.3f, y: %.3f, z: %.3f, w: %.3f]",
                     local_waypoints[wp_index].orientation.x(),
                     local_waypoints[wp_index].orientation.y(),
                     local_waypoints[wp_index].orientation.z(),
                     local_waypoints[wp_index].orientation.w());
        }

        ros::spinOnce();
        rate.sleep();
    }
return 0;
}