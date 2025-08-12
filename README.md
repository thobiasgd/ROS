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
