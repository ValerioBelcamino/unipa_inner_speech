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

    with open('/home/belca/Desktop/Palermo/neo4j_test/FewShot_query.json', 'r') as f:
            examples=json.load(f)["examples"]

    print(examples)
    try:
        graph = Neo4jGraph(uri, username, password)
        schema=graph.schema
    

        
        llm = ChatGroq(model="llama3-70b-8192",temperature=0)#llama3-70b-8192

        example_prompt = PromptTemplate.from_template(
            """Question: {question}\nQuery1: {query1}\nQuery2: {query2}"""
        )


        prompt = FewShotPromptTemplate(
            examples=examples,
            example_prompt=example_prompt,
            prefix='''Tu sei un Robot di nome Pepper, esperto in Neo4j. Dato una domanda in input crea due query Cypher sintatticamente corrette da eseguire.
    Quì trovi lo schema con le informazioni del database neo4j {schema}.
    Sotto trovi un numero di esempi di domande e le corrispettive query Cypher. Rispondi in italiano.''',

            suffix='''Ritornami esclusivamente le query e non aggiungere altro testo. Rispondi in italiano.
    Question: {question},\nQuery1: ''',
            input_variables=["question", "schema"],
        )

        print(prompt)

        cypher_response = (
        llm.bind(stop=["\nCypherResult:"])
        | StrOutputParser()
        )


        # Input to feed into the LLM
        input_data = {
            "question": "Ciao, mi chiamo Charlie, che cosa posso mangiare oggi? Non mi va di mangiare carne.",
        }

        # USER INPUT
        user_input = input("\033[32mVuoi chiedere qualcosa??\n\033[0m")
        input_data["question"] = user_input

        formatted_prompt = prompt.format(question = input_data["question"], schema = schema)
        print(formatted_prompt)
        print(type(formatted_prompt))



        cypher = cypher_response.invoke(formatted_prompt)
        print("\033[1;32mCypher query\033[0m")
        print()
        print("\033[32m"+cypher+"\033[0m")
        print()

        print(type(cypher))

        queries = [s.strip() for s in cypher.split('\n')]
        print(queries)
        if len(queries) != 2:
            print("erroreeeeeee")

        driver = GraphDatabase.driver(uri, auth=(username, password))

        user_results = []
        recipes = []

        try:
            for i, q in enumerate(queries):
                print(q)
                with driver.session() as session:
                    # Execute the query
                    result = session.run(q)
                    print("Query results:")
                    # print(result)
                    for record in result:
                        print(record)
                        if i == 0:
                            user_results.append(record)
                        if i == 1:
                            recipes.append(record)
                print('\n\n')
        except Exception as e:
            print("Error:", e.message)
        finally:
            driver.close()

        if len(user_results) != 1:
            print("problema con l'utente")

        if len(recipes) == 0:
            print("problema con le ricette")

        print(f'Found user:')
        print(user_results)
        print(f'Found Recipes:')
        print(recipes)
        print(f'N Recipes: {len(recipes)}')

        user_string = f'''user_request:{user_input}.
name:{user_results[0]['name']},
calories:{user_results[0]['daily_calories']},
proteins:{user_results[0]['daily_proteins']},
carbs:{user_results[0]['daily_carbs']},
fats:{user_results[0]['daily_fats']},
allergies: {', '.join(user_results[0]['allergies'])}'''

        result = {}
        result['user_input'] = user_string
        result['queries'] = ',\n'.join(queries)

        result_string = json.dumps(result)

        print('Result String\n', result_string)

        generate_pl_file(user_results, recipes)
        generate_csv_file(user_results, recipes)

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