import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np

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

        # Publicar las coordenadas del ArUco más cercano
        self.publisher = self.create_publisher(String, 'Aruco/coordinates', 10)

        self.bridge = CvBridge()

        # Parámetros de la cámara calibrada
        self.camera_matrix = np.array([[489.46, 0., 329.15],
                                      [0., 502.77, 191.332],
                                      [0., 0., 1.]])
        self.dist_coeffs = np.array([[-0.3958, 0.04085373, -0.00395506, -0.02253743, 0.05494331]])

        # Diccionario de ArUco
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

        self.get_logger().info("Aruco Distance Estimator Node Started")

    def image_callback(self, msg):
        try:
            # Convertir mensaje a imagen OpenCV
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            # Detectar marcadores ArUco
            corners, ids, _ = cv2.aruco.detectMarkers(frame, self.aruco_dict)
            if ids is not None:
                # Dibujar los marcadores detectados
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)

                # Estimar la pose de los marcadores
                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.05, self.camera_matrix, self.dist_coeffs)

                closest_id = None
                closest_distance = float('inf')
                closest_coords = None

                for i in range(len(ids)):
                    # Calcular distancia al marcador actual
                    distance = np.linalg.norm(tvec[i]) * 2.0

                    if distance < closest_distance:
                        closest_distance = distance
                        closest_id = ids[i][0]
                        center_x = (corners[i][0][0][0] + corners[i][0][2][0]) / 2
                        center_y = (corners[i][0][0][1] + corners[i][0][2][1]) / 2
                        closest_coords = (center_x, center_y, closest_distance)

                if closest_id is not None:
                    # Publicar coordenadas y distancia del ArUco más cercano
                    center_x, center_y, distance = closest_coords
                    message = f"ID: {closest_id}, X: {center_x:.2f}, Y: {center_y:.2f}, Dis: {distance:.2f}"
                    self.publisher.publish(String(data=message))
                    self.get_logger().info(f"Tracking closest ArUco ID {closest_id}: {message}")

                    # Dibujar el target en el frame
                    cv2.circle(frame, (int(center_x), int(center_y)), 5, (0, 0, 255), -1)
                    height, width, _ = frame.shape
                    cv2.circle(frame, (width//2, height//2), 5, (0, 255, 0), -1)
                    text = f"Target: ID {closest_id} Dis: {distance:.2f}"
                    cv2.putText(frame, text, (10, height - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            else:
                no_marker_message = "ID: -1, X: 0.0, Y: 0.0, Dis: 0.0"
                self.publisher.publish(String(data=no_marker_message))
                self.get_logger().info(f"Published: {no_marker_message}")

            cv2.imshow("Aruco Tracker", frame)
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

