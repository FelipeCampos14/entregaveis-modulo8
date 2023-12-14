from langchain.chat_models import ChatOpenAI
from langchain.prompts import ChatPromptTemplate
from langchain.schema.runnable import RunnableLambda, RunnablePassthrough
from langchain.document_loaders import TextLoader
from langchain.embeddings.sentence_transformer import SentenceTransformerEmbeddings
from langchain.text_splitter import CharacterTextSplitter
from langchain.vectorstores import Chroma
from dotenv import load_dotenv
from bs4 import BeautifulSoup
import gradio as gr
import requests
import json

# url of the page
url = 'https://www.deakin.edu.au/students/study-support/faculties/sebe/abe/workshop/rules-safety'
response = requests.get(url)

# get the html content
soup = BeautifulSoup(response.text, 'html.parser')

# write a document
with open('./data/ponderada.txt', 'w') as file:
    # Write your content to the file
    for p in soup.find_all('p'):
        file.write(p.text)

# load the document and split it into chunks
loader = TextLoader("./data/ponderada.txt")
documents = loader.load()

load_dotenv()

# split it into chunks
text_splitter = CharacterTextSplitter(chunk_size=5000, chunk_overlap=0)
docs = text_splitter.split_documents(documents)

# create the open-source embedding function
embedding_function = SentenceTransformerEmbeddings(model_name="all-MiniLM-L6-v2")

# load it into Chroma
vectorstore = Chroma.from_documents(docs, embedding_function)

retriever = vectorstore.as_retriever()

template = """Answer the question based only on the following context:
{context}
"""

prompt = ChatPromptTemplate.from_template(template)

model = ChatOpenAI(model="gpt-3.5-turbo")

chain = (
    {"context": retriever, "question": RunnablePassthrough()}
    | prompt
    | model
)

with gr.Blocks() as demo:
    chatbot = gr.Chatbot()
    msg = gr.Textbox()
        
    def respond(message, chat_history):
        answer_array = []
        for s in chain.stream(message):
            answer_array += s.content
        answer = ''.join(answer_array)
        chat_history.append((message, answer))
        return "", chat_history

    msg.submit(respond, [msg, chatbot], [msg, chatbot])

demo.launch()