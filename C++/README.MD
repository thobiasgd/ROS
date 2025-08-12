## Criando e Compilando Pacotes
Estando na pasta "raíz" do workspace, realize o comando a seguir.
```
cd src
ros2 pkg create my_cpp_pkg --build-type ament_cmake --dependencies rclcpp
cd ..
colcon build --packages-select my_cpp_pkg

```
