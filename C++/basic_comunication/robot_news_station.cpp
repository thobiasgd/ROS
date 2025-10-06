#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/string.hpp"// incluindo a interface de mensagem

using namespace std::chrono_literals; // para que o compilador entenda "0.5s" como meio segundo

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

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotNewsStationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
