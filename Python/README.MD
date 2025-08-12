## Criando e Compilando Pacotes
Estando na pasta "raíz" do workspace, realize o comando a seguir.
```
cd src
ros2 pkg create my_py_pkg --build-type ament_python --dependencies rclpy
colcon build --packages-select my_py_pkg

```
