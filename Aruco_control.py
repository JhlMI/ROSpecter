import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class ArucoPID(Node):
    def __init__(self):
        super().__init__('aruco_pid')
        self.subscription = self.create_subscription(
            String,
            'Aruco/coordinates',  # Tópico de coordenadas del ArUco
            self.aruco_callback,
            10)

        self.publisher_ = self.create_publisher(String, 'Data_esp32', 5)  # Tópico de salida PWM

        # Parámetros PID
        self.kp = 0.2
        self.ki = 0.0
        self.kd = 0.5
        self.setpoint = 300  # Centro de la cámara
        self.prev_error = 0.0
        self.integral = 0.0

        self.flag1 = False


        self.inital = True
        self.on_move = False
        self.target_id = None
        self.pwm_msg = String()

        
        self.get_logger().info("PID Node for ArUco Tracking Started")
        #self.motor_angle(180,0,0)
        


    def motor_angle(self,angle, pwm1 , pwm2):
        self.pwm_msg.data = f"{angle},{pwm2},{pwm1}"

        self.publisher_.publish(self.pwm_msg)

    def control_PID(self , current_X ):
        error = self.setpoint - current_X
        
        self.integral += error
        derivative = error - self.prev_error
        output =int( self.kp * error + self.ki * self.integral + self.kd * derivative)
        self.prev_error = error

        if output > 45:
            output= 45
        elif output <-45:
            output= -45
        if abs(output) <=10:
            output = 0

        pwm1 = max(min(70 + int(output), 80), 0)  # Límite de PWM (0 a 100)
        pwm2 = max(min(70 - int(output), 80), 0)

        self.motor_angle(-1,pwm1,pwm2)
        self.get_logger().info(f"Error: {error}, PWM: {pwm2},{pwm1}")



    def aruco_callback(self, msg):
        data =  msg.data.split(',')
        aruco_id = int( data[0].split(':')[1] )
        aruco_X = float ( data[1].split(':')[1] )
        dis = float ( data[3].split(':')[1] )
      
     
        if (aruco_id == -1):
            self.motor_angle(-1,0,0)

        if (dis > 0.24):
            self.control_PID(aruco_X)
            
        elif (aruco_id != -1):

            self.motor_angle(-1,0,0)
            if aruco_id == 0:
                angle= -1
           
            if aruco_id == 1:
           
                angle= 90
            if aruco_id == 2:
        
                angle= 270

            if aruco_id == 3:
                angle = 0
            
            
            self.motor_angle(angle,0,0)
            #time.sleep(0.5)
        


        #self.get_logger().info(f"ID: {aruco_id}, X: {aruco_X}, Dis:{dis}")

def main(args=None):    


    rclpy.init(args=args)
    pid_node = ArucoPID()
    #pid_node.motor_angle(-1,50,50)
    rclpy.spin(pid_node)
    pid_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
