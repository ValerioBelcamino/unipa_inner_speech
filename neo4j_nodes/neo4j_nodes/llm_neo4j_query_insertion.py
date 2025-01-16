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
from export_query_results import generate_pl_file, generate_csv_file

os.environ["GROQ_API_KEY"]
ws_dir = os.getenv("ROS2_WORKSPACE", "/home/belca/Desktop/ros2_foxy_ws")  # Replace with your workspace path if needed
source_dir = os.path.join(ws_dir, 'src', 'neo4j_nodes', 'neo4j_nodes')

def main():
    if len(sys.argv) < 2:
        print("Error: Shared memory name not provided", file=sys.stderr)
        sys.exit(1)

    # Attach to the shared memory block
    shared_memory_name = sys.argv[1]
    shm = SharedMemory(name=shared_memory_name)

    input_data = bytes(shm.buf[:]).decode("utf-8").strip("\x00")  # Remove null bytes
    input_data = json.loads(input_data)
    print("Input data:")
    print(input_data)
    print('\n\n')

    # Clear the shared memory buffer by overwriting it with null bytes
    shm.buf[:] = b'\x00' * len(shm.buf)  # Set the entire buffer to null bytes

    uri = "bolt://localhost:7689"  # Replace with your URI if not localhost
    username = "neo4j"             # Replace with your username
    password = "12341234"     # Replace with your password

    examples = []

    with open(os.path.join(source_dir, 'fewshot_examples', 'FewShot_query_insertion.json'), 'r') as f:
        examples=json.load(f)["examples"]

    # for ex in examples:
    #     ex["parameters"] = json.dumps(ex["parameters"])

    print(examples)
    try:
        graph = Neo4jGraph(uri, username, password)
        schema=graph.schema
    

        
        llm = ChatGroq(model="llama3-70b-8192",temperature=0)#llama3-70b-8192

        example_prompt = PromptTemplate.from_template(
            """Question: {question}\nParameters: {parameters}\nQuery: {query}"""
        )


        prompt = FewShotPromptTemplate(
            examples=examples,
            example_prompt=example_prompt,
            prefix='''Tu sei un Robot di nome Pepper, esperto in Neo4j. Dato una domanda in input crea due query Cypher sintatticamente corrette da eseguire.
    Quì trovi lo schema con le informazioni del database neo4j {schema}.
    Sotto trovi un numero di esempi di domande e le corrispettive query Cypher.''',

            suffix='''Ritornami esclusivamente le query e non aggiungere altro testo. Rispondi in italiano.
    Question: {question},\nParameters: {parameters},\nQuery: ''',
            input_variables=["question", "parameters", "schema"],
            # input_types={"question":'str', "parameters":'dict', "schema":'str'}
        )

        print(prompt)

        cypher_response = (
        llm.bind(stop=["\nCypherResult:"])
        | StrOutputParser()
        )

        print(input_data["parameters"])
        formatted_prompt = prompt.format(question = input_data["question"], parameters =  json.dumps(input_data["parameters"]), schema = schema)
        # formatted_prompt = prompt.format(question = input_data["question"], parameters =  input_data["parameters"], schema = schema)
        print(formatted_prompt)
        print(type(formatted_prompt))



        cypher_query = cypher_response.invoke(formatted_prompt)
        print("\033[1;32mCypher query\033[0m")
        print()
        print("\033[32m"+cypher_query+"\033[0m")
        print()

        print(type(cypher_query))


        driver = GraphDatabase.driver(uri, auth=(username, password))

        try:
            with driver.session() as session:
                # Execute the query
                result = session.run(cypher_query)
                print("Query results:")
                print(result)
                # for record in result:
                #     print(record)

        except Exception as e:
            print("Error:", e.message)
        finally:
            driver.close()


        result = {}
        # Populate the result dictionary
        result['user_input'] = input_data["question"]

        # For demonstration, include the raw Cypher query used (optional)
        result['queries'] = cypher_query

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