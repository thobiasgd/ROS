# **Comunicação com Tópicos**

## **Criando o Server**
Tópicos são uma estrutura de comunicação entre client e server. O Client envia uma request, o server processa a mensagem, e envia uma resposta. Vale ressaltar que os tópicos usam as interfaces como estrutura de dados, portanto recebem um "tipo" de mensagem pré determinada, e enviar um "tipo" de mensagem pré determinada também. Na criação do service, recomenda-se nomea-lo sempre começando com um "verbo", pois normalmente um server sempre é requisitado para realizar uma "ação".
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
