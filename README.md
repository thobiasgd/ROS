# ROS2
## Criando um workspace
na pasta de workspace serão instaldos todos os pacotes necessários de um projeto. para isto, crie uma pasta em Home com o nome desejado, depois uma pasta src dentro da mesma.
```
mkdir -p ros2_ws/src
```
Após isso, inicializaremos o build a fim de construir a estrutura de pastas
```
colcon build
```
Afim de não precisar fazer o source toda vez que usar o ros, recomenda-se usar o seguinte comando:
```
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```
Da mesma maneira que o código anterior, precisamos inicializar os comandos do nosso workspace para não precisar ficar inicializando-o com source a todo momento, o arquivo que realiza esta função se encontra na pasta insall que acabamos de cirar com nosso build.
```
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

## Criando Pacores
Pacotes servem para separar o código em blocos reusáveis. Por exemplo, um pacote que trata os dados de uma camera, um outro para controlar as rodas, e um outro para lidar com o tratamento de motion planning, etc...
Para começar a criar um pacote (package), vá para a pasta srs do workspace, e em seguida use o comando de packages do ros2.
Note: Primeiro argumento é a função "create", o segundo argumento se refere ao nome. A long option build-type se refere ao compilador, que pode ser ament_python, para python, e ament_cmake, para c++. podemos colocar dependências do projeto também com o --dependencies.
```
cd src
ros2 pkg create my_py_pkg --build-type ament_python --dependencies rclpy
```
Para finalizar, realize o build (afim de eficiência, recomenda-se buildar apenas os pacotes modificados usando `--packages-select pkgname`.
```
colcon build --packages-select my_py_pkg
```

