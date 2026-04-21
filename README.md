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
## ▶️ Inicialização do Sistema

Siga a sequência abaixo para iniciar corretamente a simulação do robô SCARA.

---

### 1. 🔧 Compilar o workspace ROS 2

No diretório do workspace:

```bash
colcon build
```
### 2. 🔄 Carregar o ambiente
```bash
source ~/<ros_ws>/install/setup.bash
```
### 3. 🤖 Executar os nós do ROS 2

Inicie os nós responsáveis pela simulação:
```bash
ros2 run vision vision
ros2 run conveyor_wrapper conveyor_bringup
ros2 run scara_supervisor run_supervisor
ros2 run scara_wrapper scara_bringup
```
### 4. 🎮 Iniciar a cena no CoppeliaSim
  - Abra o CoppeliaSim
  - Carregue a cena do projeto (.ttt ou .ttm)
  - Clique em Play para iniciar a simulação
