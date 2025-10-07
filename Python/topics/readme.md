# **Comunicação com Tópicos**
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
