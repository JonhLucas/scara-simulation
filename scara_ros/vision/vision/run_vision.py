import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from cv_bridge import CvBridge
import tf_transformations
from custom_interfaces.srv import ArucoDetect
import numpy as np
import cv2
from functools import partial

# Configurações de Simulação
SIMULATION = True
Ts = 0.1

class VisionSupervisor(Node):
    def __init__(self):
        super().__init__('vision')
        
        # Log de inicialização e verificação de ambiente
        self.get_logger().info(f"Iniciando Vision com OpenCV {cv2.__version__}")
        
        # Matriz da câmera e distorção (Ajuste conforme sua calibração)
        self.distCoeffs = np.array([0, 0, 0, 0, 0], dtype=np.float32)
        self.cameraMatrix = np.array([[200, 0, 150],
                                      [0, 200, 150],
                                      [0,  0,  1]], dtype=np.float32)
        
        self.bridge = CvBridge()
        
        # --- Configuração ArUco para OpenCV 4.13 ---
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters() 
        self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.parameters)

        # Definição dos pontos 3D do marcador para o solvePnP
        self.aruco_length = 0.1 # 10cm
        self.obj_points = np.array([
            [-self.aruco_length/2,  self.aruco_length/2, 0],
            [ self.aruco_length/2,  self.aruco_length/2, 0],
            [ self.aruco_length/2, -self.aruco_length/2, 0],
            [-self.aruco_length/2, -self.aruco_length/2, 0]
        ], dtype=np.float32)

        # ROS: Subscrições, Publicações e Serviços
        self.image_sub = self.create_subscription(Image, 'camera/image', self.image_callback, 10) 
        self.goal_publisher = self.create_publisher(Pose, 'supervisor/goal_pose', 10)
        
        self.aruco_service = self.create_service(ArucoDetect, 'aruco_detect', self.detect_aruco_service_callback)
        self.aruco_client = self.create_client(ArucoDetect, 'aruco_detect')

        self.image = None
        self.timer = self.create_timer(Ts, self.control_loop)

    def image_callback(self, msg: Image):  
        self.image = msg

    def control_loop(self):
        if self.image is not None:
            self.call_aruco_service(self.image)

    def detect_aruco_service_callback(self, request, response):
        try:
            # Converte imagem ROS para OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(request.image, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"Erro no CvBridge: {e}")
            return response
        
        pose = Pose()
        pose.orientation.w = 1.0 # Valor padrão para evitar quatérnio inválido

        # Processamento de imagem
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        
        # Aplicando os flips conforme sua lógica original
        gray = cv2.flip(gray, 0)
        gray = cv2.flip(gray, 1)

        # Detecção usando API 4.13
        corners, ids, _ = self.detector.detectMarkers(gray)

        if ids is not None:
            # Usando solvePnP para estimativa de pose (Padrão para 4.13)
            ret, rvecs, tvecs = cv2.solvePnP(
                self.obj_points, 
                corners[0], 
                self.cameraMatrix, 
                self.distCoeffs, 
                flags=cv2.SOLVEPNP_ITERATIVE
            )
            
            if ret:
                # Lógica de transformação de coordenadas do seu projeto
                pose.position.x = 0.69 - float(tvecs[1][0])
                pose.position.y = -0.217 + float(tvecs[0][0])
                pose.position.z = -0.0749 + float(tvecs[2][0])

                # Converter vetor de rotação para Matriz e depois Quatérnio
                rot_matrix = np.eye(4)
                rmat, _ = cv2.Rodrigues(rvecs)
                rot_matrix[0:3, 0:3] = rmat
                quat = tf_transformations.quaternion_from_matrix(rot_matrix)

                # Mapeamento de eixos original
                pose.orientation.x = round(float(quat[3]), 6)
                pose.orientation.y = round(float(quat[2]), 6)
                pose.orientation.z = round(float(quat[1]), 6)
                pose.orientation.w = round(float(quat[0]), 6)

                self.goal_publisher.publish(pose)
        
        response.pose = pose
        return response

    def call_aruco_service(self, msg):
        if not self.aruco_client.service_is_ready():
            return
        
        request = ArucoDetect.Request()
        request.image = msg
        future = self.aruco_client.call_async(request)
        future.add_done_callback(self.callback_aruco)
    
    def callback_aruco(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error(f"Chamada ao serviço falhou: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = VisionSupervisor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()