# **Comunicação entre nodes**

## Criando publishers
A própria herança de ```Node``` já permite usar o médodo ```self.nome = self.create_publisher(tipoDado, dado)```
```python
class RobotNewsStationNode(Node):
    def __init__(self):
        super().__init__("robot_news_station")
        self.publisher_ = self.create_publisher(String, "robot_news", 10) # 1- tipo da mensagem # 2- nome do tópico a ser publicado #3- queue size
        self.timer_ = self.create_timer(0.5, self.publish_news) # Cria o timer para char a função de publicação
        self.get_logger().info("Robot News Station has been started.")

    def publish_news(self): # Esta é a própria função que publica através do callback do timer
        msg = String() # class importada do tipo "String"
        msg.data = "Hello" # dado da mensagem
        self.publisher_.publish(msg) # faz o comando de publicar para o tópico "robot_news"
```

Deve-se ater ao fato que quando criamos o pacote e utilizamos o ```--dependencies rclpy```, nós já incluimos no arquivo ```package.xml``` a linha necessária para o pacote em questão, mas como neste exemplo estamos utilizando um novo "pacote" de ```String``` precisamos modificar o arquivo para que o ROS entenda também. Logo abaixo de ```<depend>rclpy</depend>```, adicione:
```xml
<depend>example_interfaces</depend>
```
