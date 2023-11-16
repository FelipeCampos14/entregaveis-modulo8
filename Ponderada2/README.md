Nesta ponderada foi exigido que se criasse um pacote que fosse capaz de mapear um ambiente e, após ser mapeado, poder ser navegado.

Com este intuito foi criado um pacote, contendo 2 launch files. O primeiro é responsável por lançar o gazebo, o teleop e o cartógrafo. Para fazer isso foi necessário lançar dois pacotes pais, 


Já o segundo launch file lança o mapa salvo pelo cartógrafo e executa os nós. 

Além disso, dois nós foram criados. O primeiro nó seta a posição inicial do robô, enquanto a segunda o envia para uma outra posição.