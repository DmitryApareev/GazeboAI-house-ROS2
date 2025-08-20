import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
import cv2
from cv_bridge import CvBridge
import csv
import os
import time


class LidarCameraSaver(Node):
    def __init__(self):
        super().__init__('lidar_camera_saver')

        # Подписка на топик камеры
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)

        # Подписка на топик лидара
        self.lidar_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10)

        # Инициализация конвертера для изображений
        self.bridge = CvBridge()

        # Настройка директорий
        self.image_dir = 'images'  # Папка для сохранения изображений
        os.makedirs(self.image_dir, exist_ok=True)

        # Настройка CSV-файла
        self.csv_file = open('lidar_camera_data_images.csv', 'w', newline='', encoding='utf-8')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['Timestamp', 'Image Path', 'Lidar Data'])

        # Переменные для хранения данных
        self.lidar_data = []  # Данные лидара
        self.image_path = None  # Путь к текущему изображению

    def image_callback(self, msg):
        """Обработчик изображений с камеры."""
        try:
            # Конвертация изображения ROS -> OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

            # Формируем имя файла для изображения
            timestamp = time.strftime('%Y%m%d_%H%M%S', time.localtime(time.time()))
            image_filename = f'{self.image_dir}/image_{timestamp}.png'

            # Сохраняем изображение в файл
            cv2.imwrite(image_filename, cv_image)

            # Сохраняем путь к изображению
            self.image_path = image_filename

            # Сохраняем данные (если есть лидарные данные)
            self.save_data()

        except Exception as e:
            self.get_logger().error(f'Error saving image: {e}')

    def lidar_callback(self, msg):
        """Обработчик данных с лидара."""
        self.lidar_data = []  # Сохраняем только свежие данные

        # Фильтруем данные лидара: берем только углы от -30 до +30 градусов
        angle = msg.angle_min
        for r in msg.ranges:
            angle_deg = angle * 180.0 / 3.14159265359
            if -30 <= angle_deg <= 30:  # Углы перед роботом
                self.lidar_data.append(r)
            angle += msg.angle_increment

    def save_data(self):
        """Сохраняет данные с камеры и лидара в CSV."""
        if self.image_path and self.lidar_data:
            timestamp = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(time.time()))
            self.csv_writer.writerow([timestamp, self.image_path, self.lidar_data])
            self.get_logger().info(f'Saved data: {timestamp}, Image: {self.image_path}')

    def destroy_node(self):
        """Закрытие файла при завершении узла."""
        self.csv_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LidarCameraSaver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

