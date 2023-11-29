# Vídeo

https://github.com/FelipeCampos14/entregaveis-modulo8/assets/99193547/523a1938-4416-4718-93b8-936140ccedfc

## Descrição

Nesta ponderada foi exigido que se criasse um pacote que fosse capaz de mapear um ambiente e, após ser mapeado, poder ser navegado.

Com este intuito foi criado um pacote, contendo 2 launch files. O primeiro é responsável por lançar o gazebo, o teleop e o cartógrafo. Para fazer isso foi necessário lançar dois pacotes pais:

```
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

```
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True 
```

Já o segundo launch file salva e lança o mapa pelo cartógrafo. Também para este se usa um pacote pai para lançar o mapa, este é:

```
ros2 run nav2_map_server map_saver_cli -f src/my_robot_controller/mapa/mapa_ponderada.yaml'
```

```
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True mapa_ponderada.yaml
```

Além disso, dois nós foram criados. O primeiro nó seta a posição inicial do robô, enquanto a segunda o envia para uma outra posição. Para rodá-los também foi criado um lançador, que executa os nós: set_pose_node e go_to_pose_node.

## Problema

Eu tive problema para integrar o lançador e os pacotes do ros, como tento demonstrar no vídeo, pois, por algum motivo que não entendi, ao contruir o pacote com o comando ```colcon build```, todo o workspace era configurado, menos a pasta de launch.


# Execução

Devido ao problema que comentei, tive que rodar a ponderada da seguinte forma:

Primeiramente há um launcher que lança o rvizz, o gazebo e o teleop do robô. Idealmente ele deveria estar integrado com os pacotes para que fosse possível rodá-lo na pasta raiz, mas como não consegui, tentei simular uma experiência pareciada colocando o path dos launchers. Portanto deve-se inicialmente estar na pasta raiz, a do workspace, neste caso o 'ros2_ws', e rodar o seguinte comando:

```
ros2 launch src/my_robot_controller/launch/launcher1.py
```

Após isso é necessário salvar o mapa, para depois poder lançá-lo. Portanto ambos foram feitos simultaneamente no launcher2.py que pode ser lançado com o comando:

```
ros2 launch src/my_robot_controller/launch/launcher2.py
```

E finalmente basta rodar o terceiro launcher, que executa os dois nós do ros.

```
ros2 launch src/my_robot_controller/launch/launcher3.py
```
