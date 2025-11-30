#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Geometry>

// Estructura para almacenar una pose (reemplaza al diccionario de Python)
struct Pose {
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
};

/**
 * @brief Convierte posición + cuaternión a matriz de transformación 4x4.
 * * @param position Vector 3D (x, y, z)
 * @param orientation Cuaternión (w, x, y, z)
 * @return Eigen::Matrix4d Matriz de transformación homogénea 4x4
 */
Eigen::Matrix4d pose_to_transform(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation)
{
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3,3>(0,0) = orientation.toRotationMatrix();
    T.block<3,1>(0,3) = position;
    return T;
}

/**
 * @brief Convierte una pose (posición + orientación) del sistema global al local.
 * * @param global_ref La pose de referencia en el sistema Global
 * @param local_ref La *misma* pose de referencia, en el sistema Local
 * @param global_input La nueva pose en el sistema Global que se quiere transformar
 * @return Pose La nueva pose expresada en el sistema Local
 */
Pose global_to_local_pose(const Pose& global_ref, const Pose& local_ref, const Pose& global_input)
{
    // T_global_ref = pose_to_transform(global_pose["position"], global_pose["orientation"])
    Eigen::Matrix4d T_global_ref = pose_to_transform(global_ref.position, global_ref.orientation);
    
    // T_local_ref = pose_to_transform(local_pose["position"], local_pose["orientation"])
    Eigen::Matrix4d T_local_ref = pose_to_transform(local_ref.position, local_ref.orientation);

    // T_rel = np.linalg.inv(T_local_ref) @ T_global_ref
    Eigen::Matrix4d T_rel = T_local_ref.inverse() * T_global_ref;

    // T_global_input = pose_to_transform(pose_global_input["position"], pose_global_input["orientation"])
    Eigen::Matrix4d T_global_input = pose_to_transform(global_input.position, global_input.orientation);

    // T_local_output = np.linalg.inv(T_rel) @ T_global_input
    Eigen::Matrix4d T_local_output = T_rel.inverse() * T_global_input;

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


int main() 
{
    // === POSES DEL DRON 2 ===
    // NOTA: El constructor de Eigen::Quaterniond es (w, x, y, z)
    // Tu script de Python usa (x, y, z, w)

    Pose pose_global_ref;
    pose_global_ref.position = Eigen::Vector3d(9.97836130443617, 0.08709615540091323, 8.392727350765263);
    pose_global_ref.orientation = Eigen::Quaterniond(0.999950107922036,  // w
                                                     0.0008360154604794934, // x
                                                     0.0004571450876097062, // y
                                                     0.009943528711044192); // z

    Pose pose_local_ref;
    pose_local_ref.position = Eigen::Vector3d(-0.1644459068775177, 0.08524183928966522, 9.005721092224121);
    pose_local_ref.orientation = Eigen::Quaterniond(-0.7014915068249182,  // w
                                                    -0.00016604578024292696, // x
                                                    -0.002040850711551727,  // y
                                                    -0.7126748720206693); // z

    // Pose en coordenadas globales que quieres transformar (ejemplo)
    Pose pose_global_input;
    pose_global_input.position = Eigen::Vector3d(12, 0.2, 7.0);
    pose_global_input.orientation = Eigen::Quaterniond(0.999950107922036,  // w
                                                       0.0008360154604794934, // x
                                                       0.0004571450876097062, // y
                                                       0.009943528711044192); // z

    Pose pose_local_output = global_to_local_pose(pose_global_ref, pose_local_ref, pose_global_input);

    // Configura la precisión de la salida para que coincida con la de numpy
    std::cout.precision(17);

    std::cout << "=== RESULTADO DRON 2 ===" << std::endl;
    
    // Eigen imprime los vectores verticalmente por defecto. .transpose() los imprime horizontalmente.
    // O puedes imprimirlos así para que sea más claro:
    std::cout << "Posición local:\n" << pose_local_output.position << std::endl;

    // Imprimir el cuaternión en el formato (x, y, z, w) para que coincida con la salida de Python
    std::cout << "Orientación local (x,y,z,w): [" 
              << pose_local_output.orientation.x() << ", "
              << pose_local_output.orientation.y() << ", "
              << pose_local_output.orientation.z() << ", "
              << pose_local_output.orientation.w() << "]" << std::endl;
              
    return 0;
}