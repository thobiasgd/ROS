#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/string.hpp"// incluindo a interface de mensagem

// Opcional: comolar "using namespace std::placeholder" para não precisar escrever tudo na linha 10 

class SmartphoneNode : public rclcpp::Node{
    public:
        SmartphoneNode() : Node("smartphone"){ 
           subscriber_ = this->create_subscription<example_interfaces::msg::String>(
            "robot_news", 10, std::bind(&SmartphoneNode::callbackRobotNews, this, std::placeholders::_1));
            RCLCPP_INFO(this->get_logger(), "Smartphone has been started.");
        }
    private:
       void callbackRobotNews(const example_interfaces::msg::String::SharedPtr msg){
        RCLCPP_INFO(this->get_logger(), "%s", msg->data.c_str());
       }

       rclcpp::Subscription<example_interfaces::msg::String>::SharedPtr subscriber_;

};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SmartphoneNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
