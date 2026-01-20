from shared_utils.customization_helpers import create_scenario_tools
from langchain_core.messages import SystemMessage, HumanMessage
from shared_utils.llm_helpers import LLM_Initializer
from pydantic import BaseModel, Field
from groq import BadRequestError
from typing import Optional
import textwrap
import time
import ast
import re 



class ScopeDetection_LLM(LLM_Initializer):
    def __init__(
        self,
        node_name: str,
        test_tools=None,
        use_db_adapter: bool = True,
        use_scenario_description: bool = True,
    ):
        super().__init__(node_name, use_db_adapter=use_db_adapter, use_scenario_description=use_scenario_description)

        # Dynamically create the scenario tools
        if test_tools is None:
            self.scenario_tools_dict = create_scenario_tools()
        else:
            self.scenario_tools_dict = test_tools
        print(f"\033[1;38;5;207mWe have the following scenarios:\033[0m")
        print(f"\033[1;38;5;207m", [f'{name}: {pcls.__doc__}' for name, pcls in self.scenario_tools_dict.items()], "\033[0m")

        self._llm = self._llm.bind_tools(self.scenario_tools_dict.values())

        print(f"\033[1;38;5;207mLoaded {len(self.scenario_tools_dict)} scenario tool(s).\033[0m")
        print()


    # Redefine abstractmethod from the parent class with more parameters (must be Noneable by default)
    def get_LLM_response(self, user_input, inner_speech=None, return_time=False):
        """
        Function to get the LLM response for a given user input.
        """
        prompt = [  
            SystemMessage(content=
                textwrap.dedent(
                    f"""You are the Scope Detection module of the architecture and you have to select the most appropriate scenario given a user request.
                    You have at your disposal:
                    -The inner speech reasoning provided by the previous module 
                    -The list of all the available scenarios together with their supported tasks
                    You have to select one of these scenario tools.
                    If the user prompt is not related to the tasks supported by any scenario you should not answer.
                    """
                )),
            HumanMessage(content=
                textwrap.dedent(
                    f"""User Question is: {user_input}.
                    The inner speech is: {inner_speech}.
                    """
                ))
        ]

        results = {}
        try:
            initial_time = time.time()
            llm_response = self._llm.invoke(prompt)
            llm_response_time = time.time() - initial_time
            print(f'\033[91m{llm_response}\033[0m')
            tool_result = llm_response.tool_calls[0]['args']
            print(f"\033[34mReason: {tool_result['reason']}\033[0m")


            tool_calls = llm_response.tool_calls
        except BadRequestError as e:
            print(f"\033[31mError: {e}\033[0m")
            llm_response = re.findall(r"<tool-use>(.*)</tool-use>", str(e))[0]
            llm_response = ast.literal_eval(llm_response)
            tool_calls = llm_response.tool_calls
            tool_name = e
            llm_response_time = -1

        tool_calls = [tool_call for tool_call in tool_calls if tool_call['name'] in self.scenario_tools_dict.keys()]

        if tool_calls == []: # no tool called -> out of scope
            tool_result = {}
            tool_name = 'OutOfScope'
        else:
            tool_name = tool_calls[0]['name']

        results['scenario'] = tool_name
        results['reason'] = tool_result['reason']
        if return_time:
            return results, llm_response_time
        else:
            return results
