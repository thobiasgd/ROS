# **Comunicação entre nodes**

## **Criando publishers**
A própria herança de ```Node``` já permite usar o médodo ```self.nome = self.create_publisher(tipoDado, dado)```
```cpp
class RobotNewsStationNode(Node):
    def __init__(self):
        super().__init__("robot_news_station")
        self.robot_name_ = "C3PO"
        self.publisher_ = self.create_publisher(String, "robot_news", 10) # 1- tipo da mensagem # 2- nome do tópico a ser publicado #3- queue size
        self.timer_ = self.create_timer(0.5, self.publish_news) # Cria o timer para char a função de publicação
        self.get_logger().info("Robot News Station has been started.")

    def publish_news(self): # Esta é a própria função que publica através do callback do timer
        msg = String() # class importada do tipo "String"
        msg.data = "Hi, this is " + self.robot_name_ +" from the robot news station." # dado da mensagem
        self.publisher_.publish(msg) # faz o comando de publicar para o tópico "robot_news"
```
![publisher_and_topic](https://github.com/thobiasgd/ROS/blob/5ba2b96b2e14f9b594920d5f330b5e3970e6aa6d/Python/basic_comunication/publisherTopic.png)

## **Criando Subscriber**
Assim como no publisher, a própria classe já possui tudo o que é necessário para a criação dos subscribers.
```cpp
class SmartphoneNode(Node):
    def __init__(self):
        super().__init__("smartphone")
        self.subscriber_ = self.create_subscription(
            String, "robot_news", self.callback_robot_news, 10) #1- Tipo da msg #2 - nome do tópico #3- função callback #4- queue size
        self.get_logger().info("Smartphone has been started.")

    def callback_robot_news(self, msg: String): # callback que será chamado toda vez que o subscriber receber a mensagem
        self.get_logger().info(msg.data) # printa a prórria mensagem recebida
```
## Adicione as dependencias necessárias no ```package.xml```
Deve-se ater ao fato que quando criamos o pacote e utilizamos o ```--dependencies rclpy```, nós já incluimos no arquivo ```package.xml``` a linha necessária para o pacote em questão, mas como neste exemplo estamos utilizando um novo "pacote" de ```String``` precisamos modificar o arquivo para que o ROS entenda também. Logo abaixo de ```<depend>rclpy</depend>```, adicione:
```xml
<depend>example_interfaces</depend>
```
