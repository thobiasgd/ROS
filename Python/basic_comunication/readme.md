# **Comunicação entre nodes**

## Criando publishers
A própria herança de ```Node``` já permite usar o médodo ```self.nome = self.create_publisher(tipoDado, dado)```

Deve-se ater ao fato que quando criamos o pacote e utilizamos o ```--dependencies rclpy```, nós já incluimos no arquivo ```package.xml``` a linha necessária para o pacote em questão, mas como neste exemplo estamos utilizando um novo "pacote" de ```String``` precisamos modificar o arquivo para que o ROS entenda também. Logo abaixo de ```<depend>rclpy</depend>```, adicione:
```xml
<depend>example_interfaces</depend>
```
