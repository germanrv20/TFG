#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "gazebo_msgs/ModelStates.h"
#include <geometry_msgs/Pose.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>

// -----------------------------------------
// Variables globales
// -----------------------------------------

geometry_msgs::Pose pose_d1_gz;
geometry_msgs::Pose pose_d2_gz;
bool d1_gz_received = false;
bool d2_gz_received = false;

sensor_msgs::Image latest_image;
bool image_received = false;

Eigen::Matrix4d Tm_d1 = Eigen::Matrix4d::Identity();
Eigen::Matrix4d Tm_d2 = Eigen::Matrix4d::Identity();

// -----------------------------------------
// Callbacks
// -----------------------------------------

void imageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    latest_image = *msg;
    image_received = true;

    ROS_INFO("Imagen recibida: ancho=%d, alto=%d, encoding=%s",
             msg->width, msg->height, msg->encoding.c_str());
}

void modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
    for (size_t i = 0; i < msg->name.size(); i++)
    {
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

// -----------------------------------------
// Funciones de transformación
// -----------------------------------------

Eigen::Matrix4d poseGazeboToTransform(const geometry_msgs::Pose& pose_msg, int tipo=1)
{
    double x = pose_msg.position.x;
    double y = pose_msg.position.y;
    double z = pose_msg.position.z;

    

    double qx = pose_msg.orientation.x;
    double qy = pose_msg.orientation.y;
    double qz = pose_msg.orientation.z;
    double qw = pose_msg.orientation.w;

    Eigen::Quaterniond q(qw, qx, qy, qz);
    Eigen::Matrix3d R = q.toRotationMatrix();

    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3,3>(0,0) = R;
    T.block<3,1>(0,3) << x, y, z;

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

Eigen::Matrix4d getCameraTransform()
{
    Eigen::Matrix4d T_c_d1 = Eigen::Matrix4d::Identity();

    // --- Traslación ---
    double x = 0.0;
    double y = 0.0;
    double z = 0.13;  // -0.02 + 0.15
    T_c_d1(0,3) = x;
    T_c_d1(1,3) = y;
    T_c_d1(2,3) = z;

    // --- Rotación ---
    // La cámara mira hacia adelante (+X del dron)
    // No aplicar pitch de 90°
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;

    Eigen::Matrix3d R;
    R = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())
      * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());

    T_c_d1.block<3,3>(0,0) = R;

    return T_c_d1;
}


// -----------------------------------------
// Main
// -----------------------------------------

int main(int argc, char** argv)
{
    ros::init(argc, argv, "drone_projection_gazebo_node");
    ros::NodeHandle nh;

    ROS_INFO("Nodo iniciado correctamente usando Gazebo!");

    // Suscriptores
    ros::Subscriber model_states_sub = nh.subscribe("/gazebo/model_states", 10, modelStatesCallback);
    ros::Subscriber image_sub = nh.subscribe("/webcam/image_raw", 10, imageCallback);

    ros::Rate rate(20); // 20 Hz

    while (ros::ok())
    {
        ros::spinOnce();

        if (d1_gz_received) {
            Tm_d1 = poseGazeboToTransform(pose_d1_gz, 1);
            std::cout << "Matriz de transformación del dron1:\n" << Tm_d1 << std::endl;
        }

        if (d2_gz_received) {
            Tm_d2 = poseGazeboToTransform(pose_d2_gz, 2);
            std::cout << "Matriz de transformación del dron2:\n" << Tm_d2 << std::endl;
        }

        Eigen::Matrix4d Td1_m = invertTransform(Tm_d1);
        std::cout << "Matriz Td1_m:\n" << Td1_m << std::endl;

        Eigen::Matrix4d Tc_d1 = getCameraTransform();
        std::cout << "Matriz Tc_d1:\n" << Tc_d1 << std::endl;

        Eigen::Matrix4d Tc_d2 = Tc_d1 * Td1_m * Tm_d2;
        std::cout << "Matriz Tc_d2:\n" << Tc_d2 << std::endl;

        Eigen::Matrix4d Td1_d2 = Td1_m * Tm_d2;
        std::cout << "Matriz Td1_d2:\n" << Td1_d2 << std::endl;

        if (image_received) {
            cv_bridge::CvImagePtr cv_ptr;
            try {
                cv_ptr = cv_bridge::toCvCopy(latest_image, "bgr8");
            } catch (cv_bridge::Exception& e) {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                continue;
            }

            double width = 640.0;
            double height = 480.0;
            double h_fov = 1.2;

            double fx = width / (2.0 * tan(h_fov / 2.0));
            double fy = fx;
            double cx = width / 2.0;
            double cy = height / 2.0;

            Eigen::Matrix3d K;
            K << fx, 0, cx,
                 0, fy, cy,
                 0, 0, 1;

            Eigen::Vector3d p_c_d2 = Tc_d2.block<3,1>(0,3);
            double X = p_c_d2(0);
            double Y = p_c_d2(1);
            double Z = p_c_d2(2);
           

            if (X > 0) {
                int u = fx * (-Y / X) + cx;
		        int v = fy * (-Z / X) + cy;   


                ROS_INFO("Pixel proyectado: u=%d, v=%d, Z=%.3f", u, v, Z);

                if (u >= 0 && u < width && v >= 0 && v < height)
                    cv::circle(cv_ptr->image, cv::Point(u, v), 6, cv::Scalar(0,0,255), -1);
                else
                    ROS_WARN("El punto proyectado está fuera de la imagen (u=%d, v=%d)", u, v);

                cv::imshow("Drone1 Camera", cv_ptr->image);
                cv::waitKey(1);
            }
        }

        rate.sleep();
    }

    return 0;
}

