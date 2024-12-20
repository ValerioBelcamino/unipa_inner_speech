import os
import json
import sys
from langchain_core.prompts import FewShotPromptTemplate, PromptTemplate
from langchain_groq import ChatGroq
from langchain_core.output_parsers import StrOutputParser

# Assuming your workspace structure is standard
ws_dir = os.getenv("ROS2_WORKSPACE", "/home/belca/Desktop/ros2_foxy_ws")  # Replace with your workspace path if needed
source_dir = os.path.join(ws_dir, 'src', 'inner_speech', 'inner_speech')

def main(data):
    os.environ["GROQ_API_KEY"]

    # Initialize the LLM
    llm = ChatGroq(model="llama3-70b-8192", temperature=0)

    # Few-shot examples for generating explanations
    examples = []
    with open(os.path.join(source_dir, 'FewShot_clingo_explaination.json'), 'r') as f:
        examples=json.load(f)["examples"]


    # Create the prompt template for explanation generation
    example_prompt = PromptTemplate.from_template(
        """
    Results: {results}
    Explaination: {explaination}
        """
    )

    # Define the FewShotPromptTemplate
    prompt = FewShotPromptTemplate(
        examples=examples,
        example_prompt=example_prompt,
        prefix="Tu sei un Robot di nome Pepper e devi supportare un utente nel seguire un corretto piano alimentare basato sui suoi bisogni e preferenze. A questo punto del processo abbiamo escluso già i piatti non adatti allo stile alimentare dell'utente e, in questo step, abbiamo generato diverse combinazioni di piatti in grado di soddisfare i vincoli di calorie e macronutrienti rimanenti. Data una una lista di combinazioni di piatti, il tuo compito è spiegare all'utente come sono stati scelti. Il numero di piatti in ogni risposta può essere 1, N, o 0 dipendentemente dai requisiti.",
        suffix="Rispondini in linguaggio naturale in lingua Italiana.\nUser Input: {results}\nExplanation:",
        input_variables=["results"],
    )

    print(data['results'])
    # Format the prompt with the input
    formatted_prompt = prompt.format(results = data['results'])


 
    # Generate the explanation using the LLM
    explanation_response = (
        llm.bind(stop=["Explanation:"])
        | StrOutputParser()
    )
    explanation = explanation_response.invoke(formatted_prompt)

    # Output the explanation
    print("\033[1;32mExplanation:\033[0m")
    print(explanation)



if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("No data provided!")
        sys.exit(1)

    try:
        data = json.loads(sys.argv[1])  # Parse JSON string
        print("SUBROUTINE")
        # print(data)

        main(data)  

    except json.JSONDecodeError as e:
        print(f"Failed to decode JSON: {e}")
