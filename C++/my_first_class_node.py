#include "rclcpp/rclcpp.hpp"

class MyNode : public rclcpp::Node{ // Criando a classe e herdando da classe Node
public: 
    MyNode() : Node("cpp_class_test"){ // A partir desta linhas seguem-se os comandos da classe em si.
        RCLCPP_INFO(this->get_logger(), "Hello World"); 
        timer_ = this->create_wall_timer(std::chrono::seconds(1), 
                                         std::bind(&MyNode::timerCallBack, this));
    }

private:
    void timerCallBack(){
        RCLCPP_INFO(this->get_logger(), "Hello %d", counter_);
        counter_++;
    }
    rclcpp::TimerBase::SharedPtr timer_;
    int counter_ = 0;
};

int main(int argc, char **argv){
    rclcpp::init(argc, argv); // Inicializa a comunicação com o ros2
    auto node = std::make_shared<MyNode>();
    rclcpp::spin(node);// Mantém o nó rodando
    rclcpp::shutdown(); // Finaliza a comunicação com o ros2
    return 0;
}
