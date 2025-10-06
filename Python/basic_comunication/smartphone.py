import rclpy
from rclpy.node import Node
from example_interfaces.msg import String

class SmartphoneNode(Node):
    def __init__(self):
        super().__init__("smartphone")
        self.subscriber_ = self.create_subscription(
            String, "robot_news", self.callback_robot_news, 10) #1- Tipo da msg #2 - nome do tópico #3- função callback #4- queue size
        self.get_logger().info("Smartphone has been started.")

    def callback_robot_news(self, msg: String): # callback que será chamado toda vez que o subscriber receber a mensagem
        self.get_logger().info(msg.data) # printa a prórria mensagem recebida


def main(args = None):
    rclpy.init(args=args)
    node = SmartphoneNode()
    rclpy.spin(node)
    rclpy.shutdown()

    

if __name__ == '__main__':
    main()
