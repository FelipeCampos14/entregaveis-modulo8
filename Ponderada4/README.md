# Vídeo


https://github.com/FelipeCampos14/entregaveis-modulo8/assets/99193547/c25c532a-576a-4560-a5bc-00f1c375c37a



# Explicação
Para esta ponderada foi criado uma venv como no tutorial do autoestudo, porém em python. Consequentemente, para rodar o projeto é necessário instalar as dependências necessárias por meio do comando:

```
pip3 install requirements.txt
```

Agora com as dependências baixadas, deve-se criar o modelo usando o biblioteca do ollama e o arquivo Modelfile, que contém o prompt que define como o LLM deve responder os inputs do Chatbot.

```
ollama create securityAdvisor -f Modelfile
```

Por último é necessário rodar o arquivo python, que realizará realizará POST para o modelo e também subirá a interface do chatbot.

```
python3 ollama.py
```

Basta acessar a url que ele disponibilizar e poderá usar o Chatbot.
