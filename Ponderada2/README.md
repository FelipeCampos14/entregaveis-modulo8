Nesta ponderada foi exigido que se criasse um pacote que fosse capaz de mapear um ambiente e, após ser mapeado, poder ser navegado.

Com este intuito foi criado um pacote, contendo 2 launch files. O primeiro é responsável por lançar o gazebo, o teleop e o cartógrafo. Para fazer isso foi necessário lançar dois pacotes pais:

```
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

```
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True 
```

Já o segundo launch file lança o mapa salvo pelo cartógrafo e executa os nós. Também para este se usa um pacote pai para lançar o mapa, este é:

```
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True mapa_ponderada.yaml
```

Além disso, dois nós foram criados. O primeiro nó seta a posição inicial do robô, enquanto a segunda o envia para uma outra posição.