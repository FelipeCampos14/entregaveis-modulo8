# Observação:

Esta atividade foi entregue muito aleḿ do prazo, então deveria estar sujeita a desconto em respeito a aqueles que entregaram dentro do prazo estipulado. Acabei me enrolando e estou entregando atrasado mesmo. Mesmo que não valha nota, apenas queria entender e fazer a atividade.

# Vídeo 

https://github.com/FelipeCampos14/entregaveis-modulo8/assets/99193547/1c5d2690-de2e-4b3e-961b-9293d1fb762b

# Explicação

Para esta ponderada dei uma adaptada na interface gráfica do Gradio da ponderada 4 do Chatbot. Para obter os dados da página web que foi demanda ser consultada, utilizei a biblioteca Beautiful Soup, uma biblioteca de scrapping, que ajudou a extrair as informações. Ao fazer o screapping dos dados, os salvei em um documento chamado 'ponderada.txt' na pasta 'data'. Para consultar o documento utilizei o código de RAG para transformar os dados de um documento em vetores que podem ser lidos pelo modelo. Finalmente o modelo recebe essas informações como contexto, sendo capaz de responder perguntas específicas sobre esses dados.

Para rodar basta baixar as depedência com:

```
pip install -r requirements.txt 
```

Executar o código em python:

```
python3 modelo.py
```


E finalmente clicar na url dada para acessar a interface gráfica do Gradio.
