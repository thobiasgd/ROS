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
