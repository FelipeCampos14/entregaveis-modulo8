# Vídeo
https://github.com/FelipeCampos14/entregaveis-modulo8/assets/99193547/64c97174-64f1-4df9-a8b7-43b425e2145f

# Execução

```
pip install -r requirements.txt
```

```
sudo apt install vlc
```

```
python3 main.py
```
#Explicação Para esta ponderada utilizei as biblioteca whisper para a transcrição áudio, deep_translator, que utiliza a api do Goodle tradutor, para a tradução, a biblioteca gtts, Google Text-to-Speech, para tranformar o o texto transcrito em áudio e finalmente a biblioteca VLC, pois aparentemente é a única que funciona com o windows do meu computador para reproduzir arquivos de áudio. Procurei dentro dessa última biblioteca uma função que fizesse para o código apenas quando a função 'play()' parasse então usei a biblioteca time para fazer uma gambiarra mesmo, com uma outra biblioteca que pega a duração de um áudio. 

O fluxo é simples e ilustrado no vídeo: o usuário diz se quer traduzir algum áudio, seleciona um áudio disponpivel, seleciona uma linguagem disponpivel e recebe o áudio transcrito para a linguagem deseja, enquanto ouve também.