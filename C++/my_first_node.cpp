#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv){
    rclcpp::init(argc, argv); // Inicializa a comunicação com o ros2
    auto node = std::make_shared<rclcpp::Node>("cpp_test"); // Basicamente em c++, todos os nós são criados como smart pointers. Em Node: #1 Nome do nó
    RCLCPP_INFO(node->get_logger(), "Hello World"); // Um macro onde: #1 método de info do nó x. 2# a mensagem
    rclcpp::spin(node);// Mantém o nó rodando
    rclcpp::shutdown(); // Finaliza a comunicação com o ros2
    return 0;
}
