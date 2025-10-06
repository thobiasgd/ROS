# **Comunicação entre nodes**

## **Criando publishers**
A própria herança de ```Node``` já permite usar o médodo ```publisher_ = this->create_publisher<example_interfaces::msg::String>("robot_news", 10)```
```cpp
class RobotNewsStationNode : public rclcpp::Node{
    public:
        RobotNewsStationNode() : Node("robot_news_station"), robot_name_("R2D2"){ 
            publisher_ = this->create_publisher<example_interfaces::msg::String>("robot_news", 10); //1- nome do tópico 2- queue size
            timer_ = this->create_wall_timer(0.5s, std::bind(&RobotNewsStationNode::publishNews, this)); // Criando o callback 1- tempo 2- bind da função que será chamada
            RCLCPP_INFO(this->get_logger(), "Robot News Station has been started.");
        }
    private:
        void publishNews(){
            auto msg = example_interfaces::msg::String(); // Criando o objeto que carrega a mensagem
            msg.data = "Hi, this is " + robot_name_ + " from the robot news station.";
            publisher_->publish(msg); // chama o método de publicação da classe Node
        }

        std::string robot_name_;
        rclcpp::Publisher<example_interfaces::msg::String>::SharedPtr publisher_; // prototipando o publisher
        rclcpp::TimerBase::SharedPtr timer_;
};
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
Deve-se ater ao fato que quando criamos o pacote e utilizamos o ```--dependencies rclcpp```, nós já incluimos no arquivo ```package.xml``` a linha necessária para o pacote em questão, mas como neste exemplo estamos utilizando um novo "pacote" de ```String``` precisamos modificar o arquivo para que o ROS entenda também. Logo abaixo de ```<depend>rclpy</depend>```, adicione:
```xml
<depend>example_interfaces</depend>
```
### **CMakeLists**
Como estamos trabalhando com c++, também deve-se adicionar a dependencia no ```CMakeLists.txt```. Logo abaixo de ```find_package(rclcpp REQUIRED)``` adicione :
```cmake
find_package(example_interfaces REQUIRED)
```
também adicionar o novo executável com a nova dependencia:
```cmake
add_executable(robot_news_station src/robot_news_station.cpp) 
ament_target_dependencies(robot_news_station rclcpp example_interfaces) 
```

