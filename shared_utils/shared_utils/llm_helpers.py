from db_adapters import DBFactory  
from abc import ABC, abstractmethod
from langchain.chat_models import init_chat_model
from shared_utils.customization_helpers import load_all_intent_models, get_scenario_description
from dotenv import load_dotenv
from groq import BadRequestError
import json
import ast
import os


class LLM_Initializer(ABC):
    def __init__(
                self, 
                node_name:str,
                use_intent_tools:bool,
                ):
        
        self.node_name = node_name
        self.use_intent_tools = use_intent_tools

        # Load environment variables from .env and .config files
        self.CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
        self.BASE_DIR = os.path.abspath(os.path.join(self.CURRENT_DIR, "../../../../../../unipa_inner_speech"))

        self.dotenv_path = os.path.join(self.BASE_DIR, ".env")
        load_dotenv(self.dotenv_path, override=True)
        self.dotconfig_path = os.path.join(self.BASE_DIR, ".config")
        load_dotenv(self.dotconfig_path)

        # Create default DB adapter
        self.db_type = os.getenv("DB_TYPE") 
        self._db = DBFactory.create_adapter(self.db_type)

        self.llm_config = ast.literal_eval(os.getenv("LLM_CONFIG"))[self.node_name]
        self._llm = init_chat_model(
                                    model=self.llm_config['model_name'], 
                                    model_provider=self.llm_config['model_provider'], 
                                    temperature=self.llm_config['temperature'], 
                                    api_key=os.getenv("GROQ_API_KEY")
                                )

        print()
        self.scenario = os.getenv("SCENARIO")
        print(f"\033[34mUsing {self.scenario}!\033[0m")
        self.context_scenario = get_scenario_description(self.scenario)
        print(f"\033[34mDesciription: {self.context_scenario}\033[0m")

        if self.use_intent_tools:
            self._dynamic_intent_tools_dict = load_all_intent_models(self.scenario)
            print(f"\033[34mLoaded {self._dynamic_intent_tools_dict} intent_tool(s).\033[0m")
            self._dynamic_intent_toolnames = [dit.__name__ for dit in self._dynamic_intent_tools_dict.values()]
            print(f"\033[1;38;5;207mLoaded {len(self._dynamic_intent_toolnames)} intent_tool(s).\033[0m")


        
    @abstractmethod
    def get_LLM_response(self, user_input):
        try:
            self._llm.invoke(user_input)
        except BadRequestError as e:
            print(f"\033[31mError: {e}\033[0m")


    def get_default_db_driver(self):
        return self._db
    

    def get_default_db_schema(self):
        return self._db.get_schema()
    

    def initialize_llm(self):
        self._llm = init_chat_model(
                model=self.llm_config['model_name'], 
                model_provider=self.llm_config['model_provider'], 
                temperature=self.llm_config['temperature'], 
                api_key=os.getenv("GROQ_API_KEY")
                )   
        
    def get_llm(self):
        if self._llm is None:
            print("\033[31mSomething is wrong with LLM Initialization!!!\033[0m")
            return None
        return self._llm
    
    def get_intent_tools_and_names(self):
        if not self.use_intent_tools:
            print("\033[31mThis LLM has been initialized without intent tools, can't access them!!!\033[0m")
        return self._dynamic_intent_tools_dict, self._dynamic_intent_toolnames
        
