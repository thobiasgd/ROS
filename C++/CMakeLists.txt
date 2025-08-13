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


install(TARGETS
  cpp_node # adicione em baixo deste, cada um dos novos executáveis
  DESTINATION lib/${PROJECT_NAME} # Insto se escreve apenas uma vez
)

ament_package()
