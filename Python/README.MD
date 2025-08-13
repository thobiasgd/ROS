## Criando e Compilando Pacotes
Estando na pasta "raíz" do workspace, realize o comando a seguir.
```
cd src
ros2 pkg create my_py_pkg --build-type ament_python --dependencies rclpy
colcon build --packages-select my_py_pkg

```
## Criando um nó
Para criar um nó, segue-se o seguinte padrão (a primeira linha ```#!/usr/bin/env python3```serve para dizer ao programa que interpretador deve usar.
```python
#!/usr/bin/env python3

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
Garanta que o terminal se encontre no diretório do script do nó. Para inicializar o nó e fazer o teste, deve-se primeiro dar permissão ao arquivo, e em seguida, inicializa-lo normalmente.
```
chmod +x my_fist_node.py
./my_fist_node.py
```
