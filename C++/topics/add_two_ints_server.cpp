#include "rclcpp/rclcpp.hpp"                     
#include "example_interfaces/srv/add_two_ints.hpp" 
using namespace std::placeholders;               // Permite o uso dos placeholders (_1, _2) usados em std::bind para associar parâmetros à função callback

class AddTwoIntsServerNode : public rclcpp::Node{
    public:
        // Construtor da classe: inicializa o nó com o nome "add_two_ints_server"
        AddTwoIntsServerNode(): Node("add_two_ints_server"){
            // Cria o serviço chamado "add_two_ints", do tipo "AddTwoInts"
            // std::bind é usado para associar a função callback "callbackAddTwoInts" a este serviço
            // "_1" e "_2" representam os parâmetros request e response passados automaticamente pelo ROS2
            server_ = this->create_service<example_interfaces::srv::AddTwoInts>(
                "add_two_ints", std::bind(&AddTwoIntsServerNode::callbackAddTwoInts, this, _1, _2));
            RCLCPP_INFO(this->get_logger(), "Add Two Ints Service has been started...");
        }

    private:
        void callbackAddTwoInts(const example_interfaces::srv::AddTwoInts::Request::SharedPtr request,
                                const example_interfaces::srv::AddTwoInts::Response::SharedPtr response){
            response->sum = request->a + request->b;
            RCLCPP_INFO(this->get_logger(), "%d + %d = %d", 
                        (int)request->a, (int)request->b, (int)response->sum);
        }
 
        rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr server_;
};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);                      
    auto node = std::make_shared<AddTwoIntsServerNode>();  
    rclcpp::spin(node);                            
    rclcpp::shutdown();                            
    return 0;                                     
}
