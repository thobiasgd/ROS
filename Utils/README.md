# Comandos úteis para desenvolvimento ROS

## **Lista dos Nós**
Aprensenta no terminal os nós que etão sendo rodados no momento.
```bash
ros2 node list
```
## **Informações do nó**
Para obter mais informações sobre um nó especifico use:
```bash
ros2 node info /node_name
```
## **Renomeando**
É importante saber, que não se pode ter dois nós com o mesmo nome rodando ao mesmo tempo, apesar de funcionar em programas simples, a chance de gerar erros em programas mais complexos é grande. Para os casos onde se é preciso utilziar o mesmo nó, recomenda-se iniciliza-los com nomes diferentes, e para isto, basta reconema-los na chamada usando:
```bash
ros2 run pkg_name node_name --ros-args -r __node:=new_node_name # -r = --remap
```
## **Symlink**
Um comando útil para o desenvolvimento, é usar ```--symlink-install``` que faz com qua quando usamos o ```colcon build```, não precisamos recompilar toda vez que fizermos alguma alteração no arquivo principal. Ele faz isso informando ao ros rodar o próprio arquivo principal, e não o "executável" que fica na pasta ```install```. Recomenda-se usar esta técnica apenas para a fase de desenvolvimento, pois há chances de gerar erros.
```bash
colcon build --packages-select my_pkg --symlink-install
```

## **RQT**
Uma das melhores ferramentas no desenvolvimento ROS é o rqt. uma ferramenta gráfica usada para ter uma perspectiva gráfica dos nós. Ela mostra quais nós estão inicializados, e como estão interconectados. Para inicializa-la basta usar:
```
rqt_graph
```

## **Escutar um Tópico**
Para fazer o debug de um tópico, e saber o que está sendo transmitido para um determinado tópico, pode-se usar o seguinte comando (eliminando a necessidade de criar um outro node apenas para saber o que está sendo transmitido):
```bash
ros2 topic echo /topic_name
```

## **Enviar para um Tópico**
Também é possuivel enviar dados para um tópico específico passando como parâmetros a frequencia em hz, o nome do tópico, o tipo da mensagem e por fim a mensagem em si (funciona apenas para datatypes simples):
```bash
ros2 topic pub -r 5 /robot_news example_interfaces/msg/String "{data: 'Hello from terminal'}"
```

## **Renomeando um Tópico**
Assim como os nós,  também é possível renomear um tópico com os argumentos do ````--ros-args```:
```bash
ros2 run pkg_name executable_name --ros-args -r old_topic_name:=new_topic_name
```
## **Enviando um request para um topic manualmente**
Normalmente, precisa-se criar um cliente para um server, mas para fins de testes, pode ser útil enviar estruturas de requests para um server para testes.
```bash
ros2 service call /service_name service_interface data
```
Exemplo:
```bash
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 3, b: 7}"
requester: making request: example_interfaces.srv.AddTwoInts_Request(a=3, b=7)

response:
example_interfaces.srv.AddTwoInts_Response(sum=10)
```

## **Criando Interfaces**
Recomenda-se criar um novo pacote onde haverá todas as interfaces customizadas, para evitar que inconviniencias de dependências no futuro. O primeiro passao para criar a interface customizada, é ir no diretório do pacote e adicionar as seguintes linhas no arquivo ```package.xml```:
```xml
<buildtool_type>rosidl_default_generators</buildtool_type>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```
Após isto, deve-se também alterar o ```CMakeLists.txt```:
```cmake
rosidl_generate_interfaces(${PROJECT_NAME}
  "your custom interfaces will be here"
  "one per line"
  "no comma for separating lines"
)

ament_export_dependencies(rosidl_default_runtime)
```
Após definir as dependencias, cria-se uma pasta para a nova interface, e em seguida o arquivo que por sua vez deve começar com uma letra maiuscula e também sem nenhum tipo de caracter especial como "_" ou "-".
```bash
mkdir msg
cd msg
touch HardwareStatus.msg
```
Para criar a nova estrutura da interface, pode-se usar as estruturas de dados primitivas (ex: float, int etc...) ou basear em uma interface já construida (Setbool, Vector, etc...). Abrindo o arquivo .msg podemos construir da seguinte forma:
```msg
float64 temperature
bool are_motors_ready
string debug_message    
```
