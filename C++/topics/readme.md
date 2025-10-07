# **Comunicação com Tópicos**

Os services (serviços) são um dos três principais mecanismos de comunicação no ROS2, junto com tópicos e ações. Eles seguem um padrão de requisição–resposta:
um cliente (client) envia uma requisição (request), o servidor (server) processa e retorna uma resposta (response).

Os serviços são úteis para tarefas pontuais que têm início, meio e fim definidos, como “somar dois números”, “mover o robô até um ponto” ou “salvar um arquivo”.

## **Estrutura de um server**
Cada service é definido por uma interface (no pacote example_interfaces, por exemplo).
Essa interface define dois tipos de mensagens:

- Request: mensagem enviada do cliente para o servidor.
- Response: mensagem retornada do servidor para o cliente.

No caso do AddTwoInts, temos:
```cpp
int64 a
int64 b
---
int64 sum
```
## **Criando um server**
O servidor é o nó responsável por receber requisições e retornar respostas.
A função callback define o que acontece quando uma requisição chega.
```cpp
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
```
## **Criando o Client**
O cliente é o nó que envia uma requisição ao servidor e espera pela resposta.
Antes de enviar, ele precisa garantir que o serviço já está ativo.
```cpp
#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

class AddTwoIntsClientNode : public rclcpp::Node{
    public:
        AddTwoIntsClientNode() : Node("add_two_ints_client"){
            client_ = this->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");
        }

        void callAddTwoInts(int a, int b){
            while (!client_->wait_for_service(1s)){
                RCLCPP_WARN(this->get_logger(), "Waiting for the server...");
            }

            auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
            request->a = a;
            request->b = b;

            client_->async_send_request(
                request, std::bind(&AddTwoIntsClientNode::callbackCallAddTwoInts, this, _1));
        }
    private:
        void callbackCallAddTwoInts(rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedFuture future){
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "Sum: %d", (int)response->sum);
        }
        rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client_;
};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AddTwoIntsClientNode>();
    node->callAddTwoInts(10, 5);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

```
### **Chamando a função do client**
```cpp
int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AddTwoIntsClientNode>();
    node->callAddTwoInts(10, 5);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```
