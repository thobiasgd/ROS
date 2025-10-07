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
´´´´bash
int64 a
int64 b
---
int64 sum
´´´
## **Criando um server**
O servidor é o nó responsável por receber requisições e retornar respostas.
A função callback define o que acontece quando uma requisição chega.
```python
class AddTwoIntsServerNode(Node):
    def __init__(self):
        super().__init__("add_two_ints_server")
        self.server_ = self.create_service(AddTwoInts, "add_two_ints", self.callback_add_two_ints) #1- tipo do dado 2-nome do service 3- callback
        self.get_logger().info("Add Two Ints Server has been started...")

    def callback_add_two_ints(self, request: AddTwoInts.Request, response: AddTwoInts.Response):
        response.sum = request.a + request.b
        return response # não esquecer de colocar um return
```
## **Criando o Client**
O cliente é o nó que envia uma requisição ao servidor e espera pela resposta.
Antes de enviar, ele precisa garantir que o serviço já está ativo.
```python
class AddTwoIntsClientNode(Node):
    def __init__(self):
        super().__init__("add_two_ints_client")
        self.client_ = self.create_client(AddTwoInts, "add_two_ints")

    def call_add_two_ints(self, a, b):
        while not self.client_.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Add Two Ints server...")

        request = AddTwoInts.Request()
        request.a = a
        request.b = b

        future = self.client_.call_async(request)
        future.add_done_callback(self.callback_call_add_two_ints)

    def callback_call_add_two_ints(self, future):
        response = future.result()
        self.get_logger().info("Got response: " + str(response.sum))
```
### **Chamando a função do client**
```python
def main(args = None):
    rclpy.init(args = args) 
    node = AddTwoIntsClientNode()
    node.call_add_two_ints(2, 7)
    rclpy.spin(node) 
    rclpy.shutdown() 

if __name__ == '__main__':
    main()
```
