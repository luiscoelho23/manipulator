import numpy as np
import matplotlib.pyplot as plt

import PyKDL as kdl
from urdf_parser_py.urdf import URDF
import rl.kdl_parser as kdl_parser

# Função de cinemática direta (para calcular a posição cartesiana a partir dos ângulos das juntas)
def forward_kinematics(angles):
    
    joint_angles = kdl.JntArray(7)
    joint_angles[0] = (-180) * np.pi /180  # Joint 1 angle in radians
    joint_angles[1] = (angles[0]) * np.pi /180  # Joint 2 angle in radians
    joint_angles[2] = (0) * np.pi /180  # Joint 3 angle in radians
    joint_angles[3] = (-180 + angles[1]) * np.pi /180  # Joint 4 angle in radians
    joint_angles[4] = (0) * np.pi /180  # Joint 5 angle in radians
    joint_angles[5] = (135 + angles[2]) * np.pi /180  # Joint 6 angle in radians
    joint_angles[6] = 0 * np.pi /180  # Joint 7 angle in radians
    
    fk_solver = kdl.ChainFkSolverPos_recursive(kdl_chain)
    eeframe = kdl.Frame()
    fk_solver.JntToCart(joint_angles, eeframe)
    
    return np.array([eeframe.p.x() ,eeframe.p.y() ,eeframe.p.z()])

# Função de cinemática inversa aproximada usando gradiente descendente
def gradient_descent_ik(joint_angles, target_position, learning_rate=0.01, tolerance=1e-3):
    max_iterations = 50000  # Limite de iterações para convergência
    for _ in range(max_iterations):
        # Calcular a posição cartesiana atual
        current_position = forward_kinematics(joint_angles)
        # Calcular o erro cartesiano
        error_cartesian = target_position - current_position
        error_magnitude = np.linalg.norm(error_cartesian)

        # Parar se o erro for menor que a tolerância
        if error_magnitude < tolerance:
            break

        # Aproximação do gradiente por diferenças finitas
        gradient = np.zeros(3)
        for i in range(3):
            joint_angles_temp = joint_angles.copy()
            joint_angles_temp[i] += 0.1  # Pequena variação para estimar o gradiente
            new_position = forward_kinematics(joint_angles_temp)
            gradient[i] = (np.linalg.norm(target_position - new_position) - error_magnitude) / 0.001

        # Atualizar ângulos das juntas na direção oposta ao gradiente para minimizar o erro
        joint_angles -= learning_rate * gradient
    
    return joint_angles

#Load Robot Kinematics
robot = URDF.from_xml_file("/home/luisc/ws_manipulator/src/manipulator/resources/robot_description/manipulator.urdf")
(_,kdl_tree) = kdl_parser.treeFromUrdfModel(robot)
kdl_chain = kdl_tree.getChain("panda_link0", "panda_finger")


# Trajetória desejada em coordenadas cartesianas
trajectory_desired = np.array([[-0.5 + 0.03 * t, 0 , 0.5 * np.sin(t)] for t in range(100)])

# Load the CSV file, skipping the first row (header)
trajectory_desired = np.genfromtxt('/home/luisc/ws_manipulator/src/manipulator/resources/dmp/dados/xyzhuman/ee_trajectory_5.csv', delimiter=',', skip_header=1)

# Ângulos iniciais das juntas
joint_angles = [90,90,20]  # Ângulos iniciais
trajectory_actual_cartesian = []
trajectory_actual_joint_angles = []

# Loop para seguir a trajetória desejada, ajustando os ângulos para minimizar o erro
for desired_position in trajectory_desired:
    
    # Atualizar os ângulos das juntas usando o algoritmo de gradiente descendente
    joint_angles = gradient_descent_ik(joint_angles, desired_position)    
    joint_angle = np.array(joint_angles)

    # Salvar a posição atual em angulos    
    trajectory_actual_joint_angles.append(joint_angle)

    # Calcular a posição atual após ajuste dos ângulos
    current_position = forward_kinematics(joint_angles)
    
    # Salvar a posição atual calculada para visualização
    trajectory_actual_cartesian.append(current_position)

# Converte a lista de trajetórias reais para um array numpy
trajectory_actual_cartesian = np.array(trajectory_actual_cartesian)
trajectory_actual_joint_angles = np.array(trajectory_actual_joint_angles)

np.savetxt("/home/luisc/ws_manipulator/src/manipulator/resources/dmp/3dmp.csv", trajectory_actual_joint_angles, delimiter=",")

# Visualização das trajetórias
plt.figure(figsize=(12, 6))
plt.plot(trajectory_desired[:, 0], trajectory_desired[:, 2], label="Trajetória Desejada", linestyle="--")
plt.plot(trajectory_actual_cartesian[:, 0], trajectory_actual_cartesian[:, 2], label="Trajetória Real", linestyle="-")
plt.xlabel("X (metros)")
plt.ylabel("Z (metros)")
plt.legend()
plt.grid()
plt.show()