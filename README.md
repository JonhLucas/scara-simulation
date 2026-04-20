# 🤖 SCARA Simulation

Simulação de um robô do tipo **SCARA (Selective Compliance Articulated Robot Arm)**, focada em estudo de cinemática, controle e visualização do movimento.

---

## 📌 Sobre o projeto

Este projeto tem como objetivo criar uma simulação interativa de um robô SCARA, permitindo:

- Visualizar o movimento do robô
- Estudar cinemática direta e inversa
- Experimentar trajetórias e posicionamento

---

## 🚀 Funcionalidades

- ✔️ Modelagem do robô SCARA  
- ✔️ Cinemática direta (Forward Kinematics)  
- ✔️ Cinemática inversa (Inverse Kinematics)  
- ✔️ Simulação de movimento  
- ✔️ Visualização (2D ou 3D, dependendo da implementação)  

---

## 🛠️ Tecnologias utilizadas

- Python (NumPy / opencv)  
- ROS2
- CoppeliaSim

---

## 📂 Estrutura do projeto

```bash
scara-simulation/
│
├── custom_interfaces/           # interfaces customizadas para actions e services
├── simulations-main/            # cena e scripts de integração com coppeliasim
├── scara_ros/                   # ROS packages
├── requiremnts.txt
└── README.md
