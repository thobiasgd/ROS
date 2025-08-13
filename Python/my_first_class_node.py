#!/usr/bin/env python3

import rclpy # biblioteca padrão do ros para o python
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__("py_class_test")
        self.get_logger().info("Hello World") # printa algo no terminal

def main(args = None):
    rclpy.init(args = args) # inicializa a comunicação com o ros
    node = MyNode()
    rclpy.spin(node) # Faz com que o nó, continue "rodando", até pressionar "ctrl c". #1 nome do nó
    rclpy.shutdown() # esta normalmente é a linha final do nó

if __name__ == '__main__':
    main()
