## Criando e Compilando Pacotes
Estando na pasta "raíz" do workspace, realize o comando a seguir.
```
cd src
ros2 pkg create my_cpp_pkg --build-type ament_cmake --dependencies rclcpp
cd ..
colcon build --packages-select my_cpp_pkg
```
Um script padrão em c++ de como construir um nó está no código abaixo. Deve-se notar que sempre que se é contruido um novo nó, há a necessidade de declara-lo como um 'smart pointer'.
```c++
#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv){
    rclcpp::init(argc, argv); // Inicializa a comunicação com o ros2
    auto node = std::make_shared<rclcpp::Node>(); // Basicamente em c++, todos os nós são criados como smart pointers
    RCLCPP_INFO(node->get_logger(), "Hello World"); // Um macro onde: #1 método de info do nó x. 2# a mensagem
    rclcpp::shutdown(); // Finaliza a comunicação com o ros2
    return 0;
}
```
