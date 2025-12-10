#!/usr/bin/env python3
from sympy import *
from .kinematics_zy import KinematicsManipulatorB
import matplotlib.pyplot as plt

class DynamicsManipulatorB():
  def __init__(self):
    pass
  def define_kinematics(self, kinematics:KinematicsManipulatorB):
    self.kinematics = kinematics
  def define_dynamics(self, mass = [0.25, 0.25]):
    print("Definiendo variables en sympy para dinámica")
    # Transformaciones de centros de masa
    self.kinematics.T_0_2 = self.kinematics.T_0_1 * self.kinematics.T_1_2
    self.kinematics.T_1_C1 = self.kinematics.trans_homo(self.kinematics.l1 / 2, 0, 0, 0, 0, 0)
    self.kinematics.T_2_C2 = self.kinematics.trans_homo(self.kinematics.l2 / 2, 0, 0, 0, 0, 0)
    self.kinematics.T_0_C1 = simplify(self.kinematics.T_0_1 * self.kinematics.T_1_C1)
    self.kinematics.T_0_C2 = simplify(self.kinematics.T_0_2 * self.kinematics.T_2_C2)
    #Rotaciones
    self.kinematics.R_0_1 = self.kinematics.T_0_1[:3, :3]
    self.kinematics.R_1_2 = self.kinematics.T_1_2[:3, :3]
    #Vectores de posición de sistemas de referencia
    self.p_0_1 = Matrix([[self.kinematics.l1], [0], [0]]) 
    self.p_1_2 = Matrix([[self.kinematics.l2], [0], [0]]) 
    #Vectores de posición de centros de masa
    self.p_1_C1 = Matrix([[self.kinematics.l1 / 2], [0], [0]]) 
    self.p_2_C2 = Matrix([[self.kinematics.l2 / 2], [0], [0]]) 
    #Vectores de posición absoluta de centros de masa (para energía potencial)
    self.p_0_C1 = self.kinematics.T_0_C1[:3, 3]
    self.p_0_C2 = self.kinematics.T_0_C2[:3, 3]
    
    # Variables de velocidad angular
    self.theta_0_1_dot = Symbol('theta_0_1_dot')
    self.theta_1_2_dot = Symbol('theta_1_2_dot')
    # Variables de aceleración angular
    self.theta_0_1_dot_dot = Symbol('theta_0_1_dot_dot')
    self.theta_1_2_dot_dot = Symbol('theta_1_2_dot_dot')
    # Masas
    self.m1 = mass[0]
    self.m2 = mass[1]
    # Matrices de inercia
    self.Ic1 = self.inertia_cylinder_vertical(self.kinematics.l1, 0.1, self.m1)
    self.Ic2 = self.inertia_cylinder_horizontal(self.kinematics.l2, 0.1, self.m2)
    #Gravedad
    self.g = -9.81

  def lagrange_effort_generator(self):
    print("Generando propagación de velocidad")
    #Velocidades angulares de sistemas
    omega_0_0 = Matrix([[0], [0], [0]])
    omega_1_1 = self.kinematics.R_0_1.transpose() * (omega_0_0 + Matrix([[0], [0], [self.theta_0_1_dot]]))
    omega_2_2 = self.kinematics.R_1_2.transpose() * (omega_1_1 + Matrix([[0], [0], [self.theta_1_2_dot]]))
    #Velocidades angulares de centros de masa
    omega_1_C1 = omega_1_1
    omega_2_C2 = omega_2_2
    #Velocidades lineales de sistemas
    v_1_1 = Matrix([[0], [0], [0]])
    v_2_2 = self.kinematics.R_1_2.transpose() * (v_1_1 + omega_1_1.cross(self.p_1_2))
    #Velocidades lineales de centros de masa
    v_1_C1 = v_1_1 + omega_1_C1.cross(self.p_1_C1)
    v_2_C2 = v_2_2 + omega_2_C2.cross(self.p_2_C2)
    print("Generando ecuaciones de lagrange")
    #Euler-Lagrange
    #Energía cinética
    k1 = 0.5 * self.m1 * v_1_C1.dot(v_1_C1) + 0.5 * omega_1_C1.dot(self.Ic1*omega_1_C1)
    k2 = 0.5 * self.m2 * v_2_C2.dot(v_2_C2) + 0.5 * omega_2_C2.dot(self.Ic2*omega_2_C2)
    k = k1 + k2
    #Energía potencial
    u1 = - self.m1 * Matrix([0, 0, self.g]).dot(self.p_0_C1)
    u2 = - self.m2 * Matrix([0, 0, self.g]).dot(self.p_0_C2)
    u = u1 + u2
    #Lagrangiano
    La = k - u
    #Reasignando variables de las juntas a la clase actual
    self.theta_0_1 = self.kinematics.theta_0_1
    self.theta_1_2 = self.kinematics.theta_1_2
    #Derivadas respecto al espacio de trabajo
    La_dot_q = Matrix([[diff(La, self.theta_0_1)], 
                       [diff(La, self.theta_1_2)]])
    #Derivadas respecto a la derivada del espacio de trabajo
    La_dot_q_dot = Matrix([[diff(La, self.theta_0_1_dot)], 
                            [diff(La, self.theta_1_2_dot)]])
    #Derivada total
    La_dot_q_dot_dt = (diff(La_dot_q_dot, self.theta_0_1) * self.theta_0_1_dot + 
                       diff(La_dot_q_dot, self.theta_1_2) * self.theta_1_2_dot + 
                       diff(La_dot_q_dot, self.theta_0_1_dot) * self.theta_0_1_dot_dot + 
                       diff(La_dot_q_dot, self.theta_1_2_dot) * self.theta_1_2_dot_dot)
    #Pares en las juntas
    tau = La_dot_q_dot_dt - La_dot_q
    tau_f = lambdify([self.theta_0_1,         self.theta_1_2,
                      self.theta_0_1_dot,     self.theta_1_2_dot,
                      self.theta_0_1_dot_dot, self.theta_1_2_dot_dot], tau)
    #Generar valores numéricos
    self.tau_m = Matrix.zeros(2, self.kinematics.samples)
    #Ciclo para todas las muestras
    print("Muestreando las ec. de lagrange")
    for i in range(self.kinematics.samples):
      self.tau_m[:, i] = tau_f( float(self.kinematics.q_m[0, i]),         float(self.kinematics.q_m[1, i]),
                                float(self.kinematics.q_dot_m[0, i]),     float(self.kinematics.q_dot_m[1, i]), 
                                float(self.kinematics.q_dot_dot_m[0, i]), float(self.kinematics.q_dot_dot_m[1, i]))

  def effort_graph(self):
    fig, ((tau_1_g, tau_2_g)) = plt.subplots(nrows=1, ncols = 2, figsize=(15,4))
    fig.suptitle("Pares en las juntas (Robot RRR en el plano XY)")

    tau_1_g.set_title("Esfuerzo junta 1")
    tau_1_g.plot(self.kinematics.t_m.T, self.tau_m[0, :].T, 'r-', linewidth=2)
    tau_1_g.set_xlabel("Tiempo (s)")
    tau_1_g.set_ylabel("Par (N·m)")
    tau_1_g.grid(True)

    tau_2_g.set_title("Esfuerzo junta 2")
    tau_2_g.plot(self.kinematics.t_m.T, self.tau_m[1, :].T, 'g-', linewidth=2)
    tau_2_g.set_xlabel("Tiempo (s)")
    tau_2_g.set_ylabel("Par (N·m)")
    tau_2_g.grid(True)

    plt.tight_layout()
    plt.show()

  def redefine_kinematics(self):
    self.kinematics = KinematicsManipulatorA()
    self.kinematics.direct_kinematics()
    self.kinematics.trajectory_generator(q_in=[0.59, 2.6, 0.25])
    self.kinematics.inverse_kinematics()
  
  def redirect_print(self, new_print):
    global print
    print = new_print

  def inertia_cylinder_vertical(self, height, radius, mass):
    Ixx = Iyy = (mass / 12.0) * (3 * radius**2 + height**2)
    Izz = 0.5 * mass * radius**2
    
    return Matrix([
      [Ixx, 0, 0], 
      [0, Iyy, 0], 
      [0, 0, Izz]
    ])
  
  def inertia_cylinder_horizontal(self, length, radius, mass):
    Ixx = Izz = (mass / 12.0) * (3 * radius**2 + length**2)
    Iyy = 0.5 * mass * radius**2
    
    return Matrix([
      [Ixx, 0, 0], 
      [0, Iyy, 0], 
      [0, 0, Izz]
    ])

def main():
  robot = DynamicsManipulatorB()
  robot.redefine_kinematics()
  robot.kinematics.ws_graph()
  robot.kinematics.q_graph()
  robot.define_dynamics()
  robot.lagrange_effort_generator()
  robot.effort_graph()

if __name__ == "__main__":
  main()
