#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Vector3
import socket
import struct

class ImuBridge(Node):
    def __init__(self):
        super().__init__('imu_bridge')
        
        # Настройка ROS2 publisher
        self.publisher_ = self.create_publisher(Imu, 'imu/data', 10)
        
        # Подключение к TCP-серверу с гироскопом
        self.last_digit = 87  # Измените на нужный IP робота
        self.host = f"192.168.5.{self.last_digit}"
        self.port = 12345
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        
        try:
            self.sock.connect((self.host, self.port))
            self.get_logger().info(f"Подключено к {self.host}:{self.port}")
        except socket.error as e:
            self.get_logger().error(f"Ошибка подключения: {e}")
            raise  # Остановит ноду, если подключение невозможно
        # Таймер для чтения данных (10 Гц)
        self.timer = self.create_timer(0.01, self.read_and_publish)

    def unpack6BytesToFloats(self, bytes_data):
        """Конвертация 24 байт (6 float) в список чисел."""
        return [struct.unpack('<f', bytes_data[i*4:(i+1)*4])[0] for i in range(6)]

    def read_and_publish(self):
        try:
            data = self.sock.recv(24)  # 6 float × 4 байта
            if len(data) != 24:
                self.get_logger().warn(f"Получено {len(data)} байт (ожидалось 24)")
                return

            # Распаковка данных: [ax, ay, az, gx, gy, gz]
            values = self.unpack6BytesToFloats(data)
            
            # Заполнение сообщения Imu
            msg = Imu()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'imu_link'
            
            # Ускорение (первые 3 значения)
            msg.linear_acceleration = Vector3(
                x=values[0], 
                y=values[1], 
                z=values[2]
            )
            
            # Угловая скорость (последние 3 значения)
            msg.angular_velocity = Vector3(
                x=values[3], 
                y=values[4], 
                z=values[5]
            )
            
            self.publisher_.publish(msg)
            # self.get_logger().info(f"Данные: accel={values[:3]}, gyro={values[3:]}", throttle_duration_sec=1)

        except Exception as e:
            self.get_logger().error(f"Ошибка: {e}")
            self.sock.close()
            raise

def main(args=None):
    rclpy.init(args=args)
    node = ImuBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.sock.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()