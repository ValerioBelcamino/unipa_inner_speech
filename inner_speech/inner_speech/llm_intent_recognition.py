import getpass
import os
import sys
import json
from multiprocessing.shared_memory import SharedMemory
from langchain_community.graphs import Neo4jGraph
from langchain.chains import GraphCypherQAChain
from langchain_groq import ChatGroq
from langchain_openai import ChatOpenAI
from langchain_core.prompts import FewShotPromptTemplate, PromptTemplate
from langchain_core.example_selectors import SemanticSimilarityExampleSelector
from langchain_core.output_parsers.string import StrOutputParser
from langchain_neo4j import Neo4jVector
from langchain_openai import OpenAIEmbeddings
from neo4j import GraphDatabase 

os.environ["GROQ_API_KEY"]

ws_dir = os.getenv("ROS2_WORKSPACE", "/home/belca/Desktop/ros2_foxy_ws")  # Replace with your workspace path if needed
source_dir = os.path.join(ws_dir, 'src', 'inner_speech', 'inner_speech')

def main():
    if len(sys.argv) < 2:
        print("Error: Shared memory name not provided", file=sys.stderr)
        sys.exit(1)

    # Attach to the shared memory block
    shared_memory_name = sys.argv[1]
    shm = SharedMemory(name=shared_memory_name)


    uri = "bolt://localhost:7689"  # Replace with your URI if not localhost
    username = "neo4j"             # Replace with your username
    password = "12341234"     # Replace with your password

    examples = []

    with open(os.path.join(source_dir, 'FewShot_intent.json'), 'r') as f:
            examples=json.load(f)["examples"]

    print(examples)
    try:
        graph = Neo4jGraph(uri, username, password)
        schema=graph.schema
    

        
        llm = ChatGroq(model="llama3-70b-8192",temperature=0)#llama3-70b-8192

        example_prompt = PromptTemplate.from_template(
            """Question: {question}\nRisposta: {risposta}"""
        )


        prompt = FewShotPromptTemplate(
            examples=examples,
            example_prompt=example_prompt,
            prefix='''Sono un Robot di nome Pepper, esperto in Neo4j. Devo aiutare un utente a soddisfare le sue esigenze alimentari ed ho a disposizione una base di conoscenza con il seguente scema: {schema}.
    Data una richiesta dell'utente, devo capire se è pertinente all'argomento. 
    Le azioni che sono in grado di effettuare sono le seguenti:
    1) Aggiungere un nuovo utente alla base di conoscenza. Parametri [nome_utente, calorie, proteine, carboidrati, grassi, intolleranze]
    2) Dare informazioni a un utente riguardo uno specifico piatto. Posso svolgere questa azione solamente se mi è stato fornito il nome dell'utente. Parametri [nome_utente, nome_piatto]
    3) Proporre un pasto all'utente basandomi sulle sue esigenze alimentari. Un pasto è inteso come una combinazione di piatti. Posso svolgere questa azione solamente se mi è stato fornito il nome dell'utente. Parametri [nome_utente]
    Nel caso in cui la domanda non sia pertinente all'argomento o non ricada nelle funzionalità che sono in grado di svolgere, devo rispondere in modo educato che sono un Robot in fase di training e non sono in grado di rispondere.
    Nel caso in cui manchi un'informazione posso richiedere all'utente di ampliare la sua domanda.
    ''',

            suffix='''Devo rispondere in linguaggio naturale in lingua italiana, devo parlare in prima persona e devo srotolare il mio ragionamento come se stessi pensando ad alta voce e spiegare come ho interpretato le intenzioni dell'utente basandomi sulla sua domanda. Al termine del ragionamento devo specificare che tipo di azione mi è stata richiesta esplicitandone il numero (1, 2 o 3). In caso di domanda incompleta o non pertinente userò 0.
    Question: {question},\nRisposta: ''',
            input_variables=["question", "schema"],
        )

        print(prompt)

        llm_response = (
        llm.bind(stop=["\nRisposta:"])
        | StrOutputParser()
        )


        # Input to feed into the LLM
        input_data = {
        }

        # USER INPUT
        user_input = input("\033[32mVuoi chiedere qualcosa??\n\033[0m")
        input_data["question"] = user_input

        formatted_prompt = prompt.format(question = input_data["question"], schema = schema)
        print(formatted_prompt)
        print(type(formatted_prompt))

        llm_response = llm_response.invoke(formatted_prompt)
        print("\033[1;32mRisposta\033[0m")
        print(llm_response)
        print(type(llm_response))

        result = {}
        result['Domanda'] = user_input
        result['Risposta'] = llm_response
        result_string = json.dumps(result)


        try:
            # Write data to shared memory
            shm.buf[:len(result_string)] = result_string.encode("utf-8")
        except Exception as e:
            print(f"Subprocess error: {e}", file=sys.stderr)
        finally:
            shm.close()

    finally:
        # Chiudi esplicitamente il driver alla fine
        graph._driver.close()




if __name__ == "__main__":
    main()