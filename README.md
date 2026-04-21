# 🤖 SCARA Simulation

Simulação de um robô do tipo **SCARA (Selective Compliance Articulated Robot Arm)**, focada em estudo de cinemática, controle e visualização do movimento.

---

## 📌 Sobre o projeto

Este projeto tem como objetivo criar uma simulação interativa de um robô SCARA, permitindo:

- Visualizar o movimento do robô
- Estudar cinemática direta e inversa
- Experimentar trajetórias e posicionamento

  [▶️ Ver demonstração](https://www.youtube.com/watch?v=LEkecfbkOco)

---

## 🚀 Funcionalidades

- ✔️ Modelagem do robô SCARA  
- ✔️ Cinemática direta (Forward Kinematics)  
- ✔️ Cinemática inversa (Inverse Kinematics)  
- ✔️ Simulação de movimento  
- ✔️ Visualização (2D ou 3D, dependendo da implementação)  

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

```
## 📋 Requisitos

Antes de executar o projeto, certifique-se de que os seguintes requisitos estão atendidos:

### 🧰 Software

- CoppeliaSim
  - Instalado e configurado com suporte à API Python (Remote API / ZMQ)
  - Certifique-se de que o interpretador Python esteja corretamente vinculado

- Python >= 3.12  
  - Recomenda-se uso de ambiente virtual (`venv`)

- ROS2 JAZZY
  - Instalado e configurado corretamente no sistema  
  - Workspace inicializado (ex: `ros2_ws`)

---

### 📦 Dependências Python

As bibliotecas Python utilizadas no projeto estão listadas no arquivo `requirements.txt`.

Para instalar todas as dependências, execute:

```bash
pip install -r requirements.txt
```


