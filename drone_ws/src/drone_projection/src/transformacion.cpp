#include <iostream>
#include <iomanip> // Para std::setprecision
#include <Eigen/Dense>
#include <Eigen/Geometry>

/**
 * @brief Estructura para almacenar una pose, similar al diccionario de Python.
 */
struct Pose {
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
};

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


int main() {
    // === POSES DEL DRON 2 ===
    // NOTA: Los datos de Python están en formato de cuaternión [x, y, z, w]
    // El constructor de Eigen::Quaterniond es (w, x, y, z)
    // Los valores se han reordenado al inicializar.

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

    // Pose en coordenadas globales que quieres transformar (ejemplo)
    Pose pose_global_input;
    pose_global_input.position = Eigen::Vector3d(11, 2, 5);
    pose_global_input.orientation = Eigen::Quaterniond(0.9999988965975715,     // w
                                                       -0.00017603257808170982, // x
                                                       4.083252849570985e-05,  // y
                                                       0.0014744995339891606); // z

    // Calcular la transformación
    Pose pose_local_output = global_to_local_pose(pose_global_ref, pose_local_ref, pose_global_input);

    // --- Imprimir Resultado ---
    // Configura la precisión de la salida para que coincida con la de numpy
    std::cout << std::fixed << std::setprecision(17);

    std::cout << "=== RESULTADO DRON 2 (Corregido) ===" << std::endl;
    
    // Eigen imprime los vectores verticalmente por defecto. 
    std::cout << "Posición local:\n" << pose_local_output.position << std::endl;

    // Imprimir el cuaternión en el formato (x, y, z, w) para que coincida con la salida de Python
    std::cout << "Orientación local (x,y,z,w): [" 
              << pose_local_output.orientation.x() << ", "
              << pose_local_output.orientation.y() << ", "
              << pose_local_output.orientation.z() << ", "
              << pose_local_output.orientation.w() << "]" << std::endl;
              
    return 0;
}