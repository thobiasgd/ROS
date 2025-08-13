## Criando e Compilando Pacotes
Estando na pasta "raíz" do workspace, realize o comando a seguir.
```
cd src
ros2 pkg create my_py_pkg --build-type ament_python --dependencies rclpy
colcon build --packages-select my_py_pkg

```
## Criando um nó
Para criar um nó, segue-se o seguinte padrão
```
import rclpy # biblioteca padrão do ros para o python
from rclpy.node import Node

def main(args = None):
    rclpy.init(args = args) # inicializa a comunicação com o ros
    node = Node("py_test") #1 - Nome do nó
    node.get_logger().info("Hello World") # printa algo no terminal
    rclpy.shutdown() # esta normalmente é a linha final do nó

if __name__ == '__main__':
    main()
```
