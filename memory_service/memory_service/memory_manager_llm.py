from pydantic import BaseModel
from shared_utils.llm_helpers import LLM_Initializer
from groq import BadRequestError
from typing import List, Optional
from langchain_core.messages import SystemMessage, HumanMessage
import textwrap


class Memory(BaseModel):
    """
    Pydantic model representing the memory state.

    Attributes:
        memory (List[str]): 
            A list of strings in natural language where each string is a fact, piece of information,
            or memory entry that the LLM maintains about the execution context 
            or user requests. This list is updated whenever new relevant information 
            is received.
    """
    memory: List[str]


class MemoryManagerLLM(LLM_Initializer):
    def __init__(self, node_name: str):
        super().__init__(node_name)

        # Define system prompt to guide the LLM about its memory management role
        self.system_prompt = (
            """You are the memory management module of our architecture. 
            You receive relevant information about the execution of user requests. 
            Your task is to maintain and update the memory: you can store new facts, 
            update existing ones, or summarize when appropriate. 
            Avoid repetition of same concepts throughout the memory entries: if something is already present in memory you are allowed not to modify the list.
            Memory is represented as a list of strings in natural language. 
            Always return the updated memory list as structured output."""
        )

        # Bind pydantic class for strict output format
        self._llm = self._llm.with_structured_output(Memory)



    def get_LLM_response(self, current_memory:str, user_input: str, queries: Optional[List[str]], results: str, explanation: str) -> List[str]:
        """
        Accepts parameters from the update_memory service,
        passes them to the LLM with structured output,
        and returns the updated memory list.
        """

        # Compose prompt with system instructions and input data
        prompt = [  
            SystemMessage(content=self.system_prompt),
            HumanMessage(content=
                textwrap.dedent(
                    f"""Current Memory: {current_memory}.
                    User Input: {user_input}.
                    Queries: {queries}.
                    Results: {results}.
                    Explanation: {explanation}.
                    Update the memory accordingly and return the updated list of facts as JSON structured output."""
                ))
        ]

        try:
            # Call LLM with structured output expecting Memory pydantic model
            llm_response = self._llm.invoke(
                prompt,
            )
            print(llm_response)

            # llm_response is a Memory instance with updated memory list
            updated_memory = llm_response.memory

        except BadRequestError as e:
            print(f"\033[31mError: {e}\033[0m")
            updated_memory = []

        return updated_memory

