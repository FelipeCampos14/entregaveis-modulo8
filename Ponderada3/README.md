
# Vídeo

https://github.com/FelipeCampos14/entregaveis-modulo8/assets/99193547/d216fe7b-59f9-45be-a6ce-2c64cbffaaf1

# Como rodar
A estrutura escolhida foi a de um workspace do ros, pois inicialmente seria integrado com o robô. Apesar de não ter sido integrado com o robô, a praticidade de dar source no workspace para carregar todas as dependências me manteve utilizando essa estratégia.

Primeiramente é necessário carregar os pacotes:
´´´
colcon build
´´´

Agora é necessário dar source para carregar as depedências:
´´´
source install/local_setup.bash
´´´

Finalmente para rodar a ponderada, basta rodar o nó:
´´´
ros2 run ponderada3 chat_node
´´´

