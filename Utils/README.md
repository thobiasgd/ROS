# Comandos úteis para desenvolvimento ROS

Aprensenta no terminal os nós que etão sendo rodados no momento.
```bash
ros2 node list
```
Para obter mais informações sobre um nó especifico use:
```bash
ros2 node info /nome_do _no
```
É importante saber, que não se pode ter dois nós com o mesmo nome rodando ao mesmo tempo, apesar de funcionar em programas simples, a chance de gerar erros em programas mais complexos é grande. Para os casos onde se é preciso utilziar o mesmo nó, recomenda-se iniciliza-los com nomes diferentes, e para isto, basta reconema-los na chamada usando:
```bash
ros2 run pkg_name node_name --ros-args -r __node:=new_node_name # -r = --remap
```
