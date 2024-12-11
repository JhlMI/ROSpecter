import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np
from std_msgs.msg import String

class ArucoDistanceEstimator(Node):
    def __init__(self):
        super().__init__('aruco_distance_estimator')

        # Suscripción a las imágenes comprimidas
        self.subscription = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.image_callback,
            10
        )

        # Publicar las coordenadas y distancia
        self.publisher = self.create_publisher(String, 'Aruco/coordinates', 10)

        self.bridge = CvBridge()

        # Parámetros de la cámara calibrada
        self.camera_matrix = np.array([[489.46, 0., 329.15],
                                      [0., 502.77, 191.332],
                                      [0., 0., 1.]])  # La matriz de cámara
        self.dist_coeffs = np.array([[-0.3958, 0.04085373, -0.00395506, -0.02253743, 0.05494331]])  # Coeficientes de distorsión

        # Definir el diccionario de ArUco
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

        self.get_logger().info("Aruco Distance Estimator Node Started")

    def image_callback(self, msg):
        try:
            # Convertir mensaje a imagen OpenCV
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            # Detectar marcadores ArUco
            corners, ids, _ = cv2.aruco.detectMarkers(frame, self.aruco_dict)
            if len(corners) > 0:
                # Dibujar los marcadores detectados
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)

                # Estimar la pose del marcador (rotación y traslación)
                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.05, self.camera_matrix, self.dist_coeffs)

                # Dibujar los ejes de coordenadas sobre el marcador
                for i in range(len(ids)):
                    cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvec[i], tvec[i], 0.1)
                    # Obtener la distancia al marcador (norma del vector de traslación)
                    distance = np.linalg.norm(tvec[i])
                    distance = distance * 2.0

                    center_x = (corners[i][0][0][0] + corners[i][0][2][0]) / 2
                    center_y = (corners[i][0][0][1] + corners[i][0][2][1]) / 2

                    # Crear el mensaje con la información del marcador
                    message = f"ID: {ids[i][0]}, X: {center_x:.2f}, Y: {center_y:.2f}, Dis: {distance:.2f}"

                    # Publicar el mensaje en el tópico
                    self.publisher.publish(String(data=message))

                    # Log del mensaje publicado
                    self.get_logger().info(f"Published: {message}")

            else:
                # Publicar un mensaje con ID negativo cuando no se detecten marcadores
                no_marker_message = "ID: -1, X: 0.0, Y: 0.0, Dis: 0.0"
                self.publisher.publish(String(data=no_marker_message))
                self.get_logger().info(f"Published: {no_marker_message}")

            cv2.imshow("Aruco Marker Distance", frame)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDistanceEstimator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
