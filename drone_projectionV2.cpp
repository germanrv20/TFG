#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "gazebo_msgs/ModelStates.h"
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PointStamped.h> 

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>    
#include <opencv2/core/eigen.hpp> 
#include <image_transport/image_transport.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <vector>

// ... (Variables globales y Callbacks se mantienen igual) ...
// -----------------------------------------
// Variables globales
// -----------------------------------------
geometry_msgs::Pose pose_d1_gz;
geometry_msgs::Pose pose_d2_gz;
bool d1_gz_received = false;
bool d2_gz_received = false;

sensor_msgs::Image latest_image;
bool image_received = false;

// -----------------------------------------
// Callbacks
// -----------------------------------------
void imageCallback(const sensor_msgs::Image::ConstPtr& msg) {
    latest_image = *msg;
    image_received = true;
}

void modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& msg) {
    for (size_t i = 0; i < msg->name.size(); i++) {
        if (msg->name[i] == "drone1") {
            pose_d1_gz = msg->pose[i];
            d1_gz_received = true;
        }
        if (msg->name[i] == "drone2") {
            pose_d2_gz = msg->pose[i];
            d2_gz_received = true;
        }
    }
}

// --- Funciones de transformación (Se mantienen igual) ---
Eigen::Matrix4d poseGazeboToTransform(const geometry_msgs::Pose& pose_msg) {
    Eigen::Quaterniond q(pose_msg.orientation.w, pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z);
    Eigen::Matrix3d R = q.toRotationMatrix();
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3,3>(0,0) = R;
    T.block<3,1>(0,3) << pose_msg.position.x, pose_msg.position.y, pose_msg.position.z;
    return T;
}

Eigen::Matrix4d invertTransform(const Eigen::Matrix4d &T) {
    Eigen::Matrix3d R = T.block<3,3>(0,0);
    Eigen::Vector3d t = T.block<3,1>(0,3);
    Eigen::Matrix4d T_inv = Eigen::Matrix4d::Identity();
    T_inv.block<3,3>(0,0) = R.transpose();
    T_inv.block<3,1>(0,3) = -R.transpose() * t;
    return T_inv;
}

Eigen::Matrix4d getCameraTransform() {
    Eigen::Matrix4d T_c_d1 = Eigen::Matrix4d::Identity();
    T_c_d1(0,3) = 0.0;
    T_c_d1(1,3) = 0.0;
    T_c_d1(2,3) = 0.13;
    return T_c_d1; 
}

// -----------------------------------------
// Main
// -----------------------------------------
int main(int argc, char** argv)
{
    ros::init(argc, argv, "drone_projection_opencv_node");
    ros::NodeHandle nh;

    ros::Publisher error_pub = nh.advertise<geometry_msgs::PointStamped>("/drone1/vision_error", 1);

    ros::Subscriber model_states_sub = nh.subscribe("/gazebo/model_states", 10, modelStatesCallback);
    ros::Subscriber image_sub = nh.subscribe("/webcam/image_raw", 10, imageCallback);

    ros::Rate rate(20); 

    // --- CONFIGURACIÓN CÁMARA ---
    double width = 640.0;
    double height = 480.0;
    double h_fov = 1.2;
    double fx = width / (2.0 * tan(h_fov / 2.0));
    double fy = fx;
    double cx = width / 2.0;
    double cy = height / 2.0;

    cv::Mat camera_matrix = (cv::Mat_<double>(3,3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
    cv::Mat dist_coeffs = cv::Mat::zeros(5, 1, CV_64F); 
    

    geometry_msgs::PointStamped error_msg;
    double error_distance = -1.0;
    double error_x = 0.0;
    double error_y = 0.0;

    // ROS FLU -> OpenCV Optical
    Eigen::Matrix3d R_flu2cv;
    R_flu2cv << 0, -1, 0, 0, 0, -1, 1, 0, 0;

    ROS_INFO("Nodo iniciado. Usando cv::projectPoints y cv::undistortPoints.");

    while (ros::ok())
    {
        ros::spinOnce();

        // Cálculos de Transformación (igual que antes)
        Eigen::Matrix4d Tm_d1 = Eigen::Matrix4d::Identity();
        Eigen::Matrix4d Tm_d2 = Eigen::Matrix4d::Identity();

        if (d1_gz_received) Tm_d1 = poseGazeboToTransform(pose_d1_gz);
        if (d2_gz_received) Tm_d2 = poseGazeboToTransform(pose_d2_gz);

        Eigen::Matrix4d Td1_m = invertTransform(Tm_d1); 
        Eigen::Matrix4d Tc_d1 = getCameraTransform();   
        Eigen::Matrix4d Tc_d2_flu = Tc_d1.inverse() * Td1_m * Tm_d2;

        // 1. Punto Matemático (Origen 0,0,0 del Drone 2)
        Eigen::Vector4d P_math_4d(0, 0, 0, 1); 
        Eigen::Vector4d P_math_flu = Tc_d2_flu * P_math_4d;
        Eigen::Vector3d P_math_cv = R_flu2cv * P_math_flu.head<3>();

        // 2. Punto Visual (Centro 0,0,0.13 del Drone 2)
        Eigen::Vector4d P_vis_4d(0, 0, 0.13, 1); 
        Eigen::Vector4d P_vis_flu = Tc_d2_flu * P_vis_4d;
        Eigen::Vector3d P_vis_cv = R_flu2cv * P_vis_flu.head<3>();

        error_distance = -1.0;
        error_x = -999.0;
        error_y = -999.0;
        error_msg.header.stamp = ros::Time::now();
        error_msg.header.frame_id = "drone1_camera"; 

        if (image_received) {
            cv_bridge::CvImagePtr cv_ptr;
            try {
                cv_ptr = cv_bridge::toCvCopy(latest_image, "bgr8");
            } catch (cv_bridge::Exception& e) {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                continue;
            }

            cv::Point2d center_pt(cx, cy);
            cv::drawMarker(cv_ptr->image, center_pt, cv::Scalar(0, 255, 0), cv::MARKER_CROSS, 20, 2);

            double Z_depth = P_vis_cv.z(); 

            if (Z_depth > 0) { 
                
                // --- A. PROYECCIÓN DISTORSIONADA (Lo que ve la cámara real) ---
                std::vector<cv::Point3f> pts_3d;
                pts_3d.push_back(cv::Point3f(P_math_cv.x(), P_math_cv.y(), P_math_cv.z())); // [0] PID
                pts_3d.push_back(cv::Point3f(P_vis_cv.x(), P_vis_cv.y(), P_vis_cv.z()));    // [1] Dibujo

                std::vector<cv::Point2f> pts_distorted;
                cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64F);
                cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64F);
                
                cv::projectPoints(pts_3d, rvec, tvec, camera_matrix, dist_coeffs, pts_distorted);
                
                // --- B. QUITAR DISTORSIÓN (UNDISTORT) ---
                // Aquí convertimos los puntos distorsionados a puntos lineales ideales
                std::vector<cv::Point2f> pts_undistorted;
                
                // cv::undistortPoints toma puntos, matriz K, coeffs D.
                // IMPORTANTE: Los últimos dos parámetros (R, P) son clave.
                // Si P = camera_matrix, devuelve coordenadas de píxel. Si P está vacío, devuelve normalizadas.
                cv::undistortPoints(pts_distorted, pts_undistorted, camera_matrix, dist_coeffs, cv::noArray(), camera_matrix);

                // Ahora tenemos:
                // pts_distorted[0]   -> Punto REAL en imagen (curvado)
                // pts_undistorted[0] -> Punto IDEAL lineal (como si la lente fuera perfecta)

                cv::Point2d pt_math_dist = pts_distorted[0];     // Usamos el distorsionado para pintar sobre la imagen real
                cv::Point2d pt_math_undist = pts_undistorted[0]; // Usamos este si quieres calcular error lineal puro
                cv::Point2d pt_vis_dist = pts_distorted[1];

                // Comprobamos límites
                if (pt_vis_dist.x >= 0 && pt_vis_dist.x < width && 
                    pt_vis_dist.y >= 0 && pt_vis_dist.y < height) 
                {
                    // CALCULAR ERROR
                    // Normalmente para control visual sobre la imagen, usamos el punto DISTORSIONADO
                    // porque es donde realmente está el objeto en los píxeles que vemos.
                    // Pero si quieres usar el modelo lineal, usa pt_math_undist.
                    
                    error_x = cx - pt_math_undist.x; // Usamos el distorsionado para coincidir con la imagen
                    error_distance = cv::norm(center_pt - pt_math_dist);
                    
                    // DIBUJAR
                    cv::circle(cv_ptr->image, cv::Point((int)pt_vis_dist.x, (int)pt_vis_dist.y), 6, cv::Scalar(0, 0, 255), -1);
                    cv::line(cv_ptr->image, center_pt, cv::Point((int)pt_vis_dist.x, (int)pt_vis_dist.y), cv::Scalar(255, 0, 0), 2);
                    
                    // Opcional: Dibujar punto ideal (azul cian) para ver la diferencia
                    // cv::circle(cv_ptr->image, cv::Point((int)pt_math_undist.x, (int)pt_math_undist.y), 4, cv::Scalar(255, 255, 0), 1);
                }
            }

            cv::imshow("Drone1 Camera (ProjectPoints)", cv_ptr->image);
            cv::waitKey(1);
        }

        error_msg.point.x = error_x; 
        error_msg.point.y = 0.0;
        error_msg.point.z = 0.0;
        error_pub.publish(error_msg);
        
        if (error_distance > 0) {
            ROS_INFO_THROTTLE(1.0, "Error Yaw: %.2f px", error_x);
        }

        rate.sleep();
    }

    return 0;
}