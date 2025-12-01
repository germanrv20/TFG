#!/usr/bin/env python

import rospy
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D # Necesario para 3D
from geometry_msgs.msg import PoseStamped
from collections import deque
import numpy as np

# --- CONFIGURACIÓN ---
DRONE_NS = "/drone2"
WINDOW_SIZE = 400    # Historial de puntos
# ---------------------

# Buffers (Setpoint y Realidad)
ref_x = deque(maxlen=WINDOW_SIZE)
ref_y = deque(maxlen=WINDOW_SIZE)
ref_z = deque(maxlen=WINDOW_SIZE)

act_x = deque(maxlen=WINDOW_SIZE)
act_y = deque(maxlen=WINDOW_SIZE)
act_z = deque(maxlen=WINDOW_SIZE)

def setpoint_cb(msg):
    ref_x.append(msg.pose.position.x)
    ref_y.append(msg.pose.position.y)
    ref_z.append(msg.pose.position.z)

def local_pose_cb(msg):
    act_x.append(msg.pose.position.x)
    act_y.append(msg.pose.position.y)
    act_z.append(msg.pose.position.z)

def live_plotter_3d():
    rospy.init_node('live_3d_plotter', anonymous=True)
    
    rospy.Subscriber(f"{DRONE_NS}/mavros/setpoint_position/local", PoseStamped, setpoint_cb)
    rospy.Subscriber(f"{DRONE_NS}/mavros/local_position/pose", PoseStamped, local_pose_cb)
    
    plt.ion()
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d') # Eje 3D
    
    rospy.loginfo(f"Iniciando ploteo 3D para {DRONE_NS}...")
    
    rate = rospy.Rate(10) # 10 Hz
    
    while not rospy.is_shutdown():
        if len(ref_x) > 0 and len(act_x) > 0:
            
            ax.clear()
            
            # --- PLOTEAR DATOS ---
            # Setpoint (Ideal) - Azul Punteado
            ax.plot(ref_x, ref_y, ref_z, 'b--', label='Setpoint (Ideal)', linewidth=1)
            # Realidad (Dron) - Rojo Sólido
            ax.plot(act_x, act_y, act_z, 'r-', label='Real (Dron)', linewidth=2)
            
            # --- CONFIGURACIÓN DE EJES ---
            ax.set_title("Trayectoria 3D del Dron")
            ax.set_xlabel("X (m)")
            ax.set_ylabel("Y (m)")
            ax.set_zlabel("Z (Altura - m)")
            
            # Fijar límites para que no 'baile' el gráfico (Ajustar según tu trayectoria)
            # Centro aproximado (0, -10, 7)
            ax.set_xlim(-5, 5)
            ax.set_ylim(-15, -5)
            ax.set_zlim(4, 10)
            
            ax.legend()
            
            fig.canvas.draw()
            fig.canvas.flush_events()
            
        rate.sleep()

if __name__ == '__main__':
    try:
        live_plotter_3d()
    except rospy.ROSInterruptException:
        pass