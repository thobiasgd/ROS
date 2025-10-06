#!/usr/bin//env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import String

class RobotNewsStationNode(Node):
    def __init__(self):
        super().__init__("robot_news_station")
        self.robot_name_ = "C3PO"
        self.publisher_ = self.create_publisher(String, "robot_news", 10) # 1- tipo da mensagem # 2- nome do tópico a ser publicado #3- queue size
        self.timer_ = self.create_timer(0.5, self.publish_news) # Cria o timer para char a função de publicação
        self.get_logger().info("Robot News Station has been started.")

    def publish_news(self): # Esta é a própria função que publica através do callback do timer
        msg = String() # class importada do tipo "String"
        msg.data = "Hi, this is " + self.robot_name_ +" from the robot news station." # dado da mensagem
        self.publisher_.publish(msg) # faz o comando de publicar para o tópico "robot_news"

def main(args = None):
    rclpy.init(args = args)
    node = RobotNewsStationNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
