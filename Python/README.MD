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
    rclpy.spin(node) # Faz com que o nó, continue "rodando", até pressionar "ctrl c". #1 nome do nó
    rclpy.shutdown() # esta normalmente é a linha final do nó

if __name__ == '__main__':
    main()
```
Garanta que o terminal se encontre no diretório do script do nó. Para inicializar o nó e fazer o teste, deve-se primeiro dar permissão ao arquivo, e em seguida, inicializa-lo normalmente.
```
chmod +x my_fist_node.py
./my_fist_node.py
```
### Instalando o arquivo
Recomenda-se instalar o arquivo. fazendo o passo a seguir, o ros cria um executável para incializar o programa.
Vá para o arquivo `setup.py` e adicione um novo argumento para o dicionario `entry_points` no índice 'console_scripts'.
```python
entry_points={
    'console_scripts': [
        "py_node = my_py_pkg.my_fist_node:main" # nome do executável = pasta_do_executavel.nome_do_executavel:nome_da_funcao
    ],
}
```
Para adicionar mais execuáveis a este pacote, adicione mais nós à lista de 'console_scripts'
### Executar o executável
Volte na pasta raíz do workspace, compile, e isso criará um executável na pasta 'install'
```bash
colcon build
```
Agora, para executar o executável, chama a função `ros2 run`
```bash
ros2 run my_py_pkg py_node
```
## Criando um nó com classes
Para criar em forma de class, não muda muita coisa. cria-se uma classe que herda da classe Node, e faz-se as inicializações necessárias. não se esqueça de adicionar o novo programa no setup.py e também de buildar a aplicação.
```python
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
```
