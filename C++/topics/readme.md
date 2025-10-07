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
# Define a classe do nó cliente, herdando de Node
class AddTwoIntsClientNode(Node):
    def __init__(self):
        # Inicializa o nó com o nome "add_two_ints_client"
        super().__init__("add_two_ints_client")
        # Cria um cliente para o serviço "add_two_ints", do tipo AddTwoInts
        self.client_ = self.create_client(AddTwoInts, "add_two_ints")

    # Função que envia a requisição ao servidor
    def call_add_two_ints(self, a, b):
        # Aguarda até que o serviço esteja disponível (verifica a cada 1 segundo)
        while not self.client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Waiting for Add Two Ints server...")

        # Cria o objeto de requisição (mensagem de entrada)
        request = AddTwoInts.Request()
        # Define os valores dos parâmetros da requisição
        request.a = a
        request.b = b

        # Envia a requisição de forma assíncrona (não bloqueia a execução)
        future = self.client_.call_async(request)
        # Define a função de callback que será chamada quando a resposta chegar
        future.add_done_callback(self.callback_call_add_two_ints)

    # Função de callback chamada quando o servidor responde
    def callback_call_add_two_ints(self, future):
        # Obtém a resposta do futuro (objeto retornado pelo servidor)
        response = future.result()
        # Exibe no terminal o valor retornado (soma)
        self.get_logger().info("Got response: " + str(response.sum))

```
### **Chamando a função do client**
```cpp
def main(args = None):
    rclpy.init(args = args) 
    node = AddTwoIntsClientNode()
    node.call_add_two_ints(2, 7)
    rclpy.spin(node) 
    rclpy.shutdown() 

if __name__ == '__main__':
    main()
```
