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
Ao contrário da sua versão em python, o c++ exige compilara para que o programa rodo, e para isso devemos reazlizar uma alteração no arquivo `CMakeLists.txt`. Antes de qualquer coisa, a fim de possuir um código mais limpo, pode-se excluir o seguinte trecho do arquivo.
```cmake
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()
```
A partir daí, entre o bloco de `find_package(...)` e a ulima linha `ament_package()`, cria-se o comando para definição do executável, cujos parâmetros são: nome do executável - caminho do arquivo .cpp a partir do `CMakeLists.txt`. Em seguida, insere-se a linha que informa ao programa, quais dependências o executável precisa para compilar. E por fim, precisa-se instalar o programa, e para isso, se escreve o bloco do `install()`. O arquivo final fica desta maneira:
```cmake
cmake_minimum_required(VERSION 3.8)
project(my_cpp_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(cpp_node src/my_first_node.cpp) # adicionar o executável. #1 nome do executável #2 caminho do arquivo
ament_target_dependencies(cpp_node rclcpp) # informa as dependências. #1 nome do executavel que foi criado na linha anterior #2 a dependência em si


install(TARGET_SUPPORTS_SHARED_LIBS
  cpp_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

``
