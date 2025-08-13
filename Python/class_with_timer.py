#!/usr/bin/env python3

import rclpy # biblioteca padrão do ros para o python
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__("py_timer_class_node")
        self.counter_ = 0 # criando um atributo de contador
        self.get_logger().info("Hello World") # printa algo no terminal
        self.create_timer(1.0, self.timer_callback) #1 tempo - #2 função de callback

    def timer_callback(self): # função que será chamada a cada segundo
        self.get_logger().info("Hello " + str(self.counter_))
        self.counter_ += 1

def main(args = None):
    rclpy.init(args = args) # inicializa a comunicação com o ros
    node = MyNode()
    rclpy.spin(node) # Faz com que o nó, continue "rodando", até pressionar "ctrl c". #1 nome do nó
    rclpy.shutdown() # esta normalmente é a linha final do nó

if __name__ == '__main__':
    main()
