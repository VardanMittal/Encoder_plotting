import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import serial
import threading

class SerialReader(Node):
    
    def __init__(self) -> None:
        super().__init__('serial_reader')
        self.publisher_motor = self.create_publisher(Float32, 'Motor_encoder', 10)
        self.publisher_propeller = self.create_publisher(Float32, 'Propeller_encoder', 10)
        self.ser = serial.Serial('COM8', 115200, timeout=1)  # Adjust the baudrate as per your ESP32 configuration
        
        # Start the serial reading thread
        thread = threading.Thread(target=self.read_serial_data)
        thread.start()

    def read_serial_data(self):
        while True:
            try:
                line = self.ser.readline().decode('utf-8').strip()
                if line:
                    data = line.split(',')
                    if len(data) == 2:
                        motor_value = float(data[0])
                        propeller_value = float(data[1])
                        
                        self.publish_data(motor_value, propeller_value)
            except Exception as e:
                self.get_logger().error(f"Error reading serial data: {e}")

    def publish_data(self, motor_value, propeller_value):
        motor_msg = Float32()
        motor_msg.data = motor_value
        self.publisher_motor.publish(motor_msg)
        
        propeller_msg = Float32()
        propeller_msg.data = propeller_value
        self.publisher_propeller.publish(propeller_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SerialReader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

