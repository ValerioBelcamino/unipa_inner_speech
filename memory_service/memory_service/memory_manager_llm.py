from pydantic import BaseModel
import os
from datetime import datetime as time
import chromadb
from chromadb.errors import NotFoundError
from langchain_chroma import Chroma
from langchain.tools import tool
from langchain_core.prompts import ChatPromptTemplate
from langchain_core.runnables import RunnablePassthrough
from langchain_mistralai import MistralAIEmbeddings
from langgraph.graph import MessagesState, StateGraph, END, START
from langchain_core.messages import HumanMessage, AIMessage
from typing import Dict, Any, Literal, TypedDict
from langchain.chat_models import init_chat_model
from typing import Literal
from dotenv import load_dotenv
from shared_utils.llm_helpers import LLM_Initializer
load_dotenv()

# Define the agent state    
class AgentState(TypedDict):
    messages: list
    tool_calls: list[Dict[str, Any]]
    core_memory: list[str]
    retrieved_memory: str = ""
    current_interaction: str = Literal["insert", "retrieve"]
    maximum_historical_messages: int = 5  # Limit the number of historical messages to keep
    core_memory_limit: int = 150  # Character limit for core memory


class InsertCoreMemories(BaseModel):
    """These are the core memories of our assistant. They should store relevant facts about the user in a concise and synthetic manner. 
    This information is going to be used as context for every interaction so you have to make sure to only store important facts and avoid redundancy.
    Include in memories all the facts that you knew before and add any new relevant fact that you can find in the exceeding messages.
    If you see contrasting facts, you should replace the old ones with the new ones."""
    memories: list[str]

class SplitCoreAndArchivalMemory(BaseModel):
    """This tool splits the memories into core and archival memories.
    If you see contrasting facts, you should replace the old ones with the new ones."""
    core_memories: list[str]
    archival_memories: list[str]

# Initialize ChromaDB client and collection for archival memory
chroma_client = chromadb.PersistentClient(path="./chroma_db")
collection_name = "memory_archive"

# Create or get the collection
try:
    collection = chroma_client.get_collection(name=collection_name)
except NotFoundError:
    collection = chroma_client.create_collection(name=collection_name)

# Initialize embeddings and vector store
embeddings = MistralAIEmbeddings()
vector_store = Chroma(
    client=chroma_client,
    collection_name=collection_name,
    embedding_function=embeddings
)

class MemoryAgentLLM(LLM_Initializer):

    def __init__(self, node_name:str):
        super().__init__(node_name)

    # Redefine abstractmethod from the parent class with more parameters (must be Noneable by default)
    def get_LLM_response(self, prompt):
        self._llm.invoke(prompt)

    def get_LLM(self):
        return self._llm
    
memoryAgentLLM = MemoryAgentLLM('memory_agent')

def messages_to_str(messages) -> str:
    """Convert a list of messages to a single string for prompt input."""
    if isinstance(messages, list):
        return "\n".join(messages_to_str(msg) for msg in messages)
    elif isinstance(messages, HumanMessage):
        return f"Human: {messages.content}"
    elif isinstance(messages, AIMessage):
        return f"AI: {messages.content}"
    else:
        return str(messages)

@tool
def insert_archival_memories(memories: list[str]):
    """These are the archival memories of our assistant. They should store less frequently accessed information that may be useful for long-term context.
    Given old memories from the historical interactions, you should summarize them into this other memory."""
    doc_ids = [f"memory_{len(collection.get()['ids']) + i}" for i in range(1, len(memories) + 1)]
    print(f"\tInserting archival memories: {memories} with IDs: {doc_ids}")
    vector_store.add_texts(texts=memories, ids=doc_ids, metadatas=[{"timestamp": str(time.now())}]*len(memories))
    return f"Memories added with IDs: {', '.join(doc_ids)}"



# Tool to check if information is sufficient to answer the query
class InformationSufficiency(BaseModel):
    """This tool checks if the information provided is sufficient to answer the user's query.
    Answer True if sufficient, False otherwise.
    Estimate sufficiency ONLY based on the provided context."""
    is_sufficient: bool


def interaction_type_node(state: AgentState) -> str:
    return state["current_interaction"]

def empty_node(state: AgentState) -> bool:
    return state

def check_information_sufficiency(state: AgentState) -> bool:
    print("\tChecking information sufficiency")
    
    if len(state["messages"]) == 0:
        return False
    
    user_query = state["messages"][-1].content
    core_memory = state["core_memory"]
    previous_messages = state["messages"][:-1]
    previous_messages = messages_to_str(previous_messages)
    prompt = ChatPromptTemplate.from_messages([
        ("system", "You are a tool that checks if the information provided is sufficient to answer the user's query."),
        ("human", """User query: {user_query}
Known facts: {core_memory}
Your previous interactions: 

{previous_messages}

Is the information sufficient to answer the query?""")
    ])
    
    router_llm = memoryAgentLLM.get_LLM().bind_tools([InformationSufficiency])
    chain = prompt | router_llm
    response = chain.invoke({"user_query": user_query, "core_memory": core_memory, "previous_messages": previous_messages})
    # extract and coerce
    try:
        is_sufficient = response.tool_calls[0]['args']["is_sufficient"]
        if isinstance(is_sufficient, str):
            is_sufficient = is_sufficient.strip().lower() in ["true", "1", "yes"]
    except Exception:
        is_sufficient = False
    print(f"\tInformation sufficiency: {is_sufficient}")

    return bool(is_sufficient)

# Tool to add a new memory to the archive
@tool
def add_memory(memory_content: str) -> str:
    """Add a new memory to the archival vector store."""
    print(f"\tAdding memory: {memory_content}")
    doc_id = f"memory_{len(collection.get()['ids']) + 1}"
    vector_store.add_texts(texts=[memory_content], ids=[doc_id])
    return f"Memory added with ID: {doc_id}"

# Tool to retrieve memories from the archive
@tool
def retrieve_memory(query: str, k: int = 3) -> str:
    """Retrieve relevant memories from the archival vector store based on a query.
    Returns up to k relevant memories.
    Think about the best value of k based on the complexity of the query."""
    docs = vector_store.similarity_search(query, k=k)
    print(docs)
    results = "\n".join([f"ID: {doc.id}, Content: {doc.page_content}" for doc in docs])
    return results if results else "No relevant memories found."

# Define the retrieval node
def retrieval_node(state: AgentState):
    print("\tRetrieval node activated")
    prompt = ChatPromptTemplate.from_messages([
        ("system", """You are an agent who retrieves memories from the archive to enhance currently available context. 
         Current memory context: {core_memory}"""),
        ("human", "{input}")
    ])
    
    # Bind tools to LLM
    llm_with_tools = memoryAgentLLM.get_LLM().bind_tools([retrieve_memory])
    
    chain = prompt | llm_with_tools
    response = chain.invoke({"input": state["messages"][-1].content, "core_memory": state["core_memory"]})
    
    return {"tool_calls": state["tool_calls"] + [response]}

# Define tool execution node
def tool_node(state: AgentState):
    print("\tTool node activated")
    messages = state["tool_calls"]
    last_message = messages[-1]
    
    tool_calls = last_message.tool_calls if hasattr(last_message, 'tool_calls') else []
    results = []
    
    for tool_call in tool_calls:
        tool_name = tool_call["name"]
        tool_args = tool_call["args"]
        
        if tool_name == "add_memory":
            print(f"Invoking add_memory with args: {tool_args}")
            result = add_memory.invoke(tool_args)
        elif tool_name == "retrieve_memory":
            print(f"Invoking retrieve_memory with args: {tool_args}")
            result = retrieve_memory.invoke(tool_args)
        elif tool_name == "InsertCoreMemories":
            print(f"Invoking InsertCoreMemories with args: {tool_args}")
            state["core_memory"] = tool_args["memories"]
            result = "Core memories updated."
        elif tool_name == "insert_archival_memories":
            print(f"Invoking insert_archival_memories with args: {tool_args}")
            result = insert_archival_memories.invoke(tool_args)
        elif tool_name == "SplitCoreAndArchivalMemory":
            print(f"Invoking SplitCoreAndArchivalMemory with args: {tool_args}")
            state["core_memory"] = tool_args["core_memories"]
            result = insert_archival_memories.invoke({"memories": tool_args["archival_memories"]})
        else:
            result = "Unknown tool"
        
        results.append(result)
    
    # Update memory context with retrieval results if applicable
    if "retrieve_memory" in [tc["name"] for tc in tool_calls]:
        retrieved_memory = "\n".join(results)
    else: 
        retrieved_memory = state.get("retrieved_memory", "")
    
    return {"tool_calls": messages + [AIMessage(content="\n".join(results))], "retrieved_memory": retrieved_memory, "core_memory": state["core_memory"]}

def generate_answer(state: AgentState):
    print("\tAnswer agent node activated")
    prompt = ChatPromptTemplate.from_messages([
        ("system", """You are a helpful agent that replies to user queries. 
         You answer only based on the following information:
         Facts about the user: {core_memory}
         Retrieved memories: {retrieved_memory}
         Your previous interactions: {messages}
         Select the most relevant information to answer the user's query as best as you can. 
         If you don't know the answer, simply say you don't know. 
         Do not make up an answer."""),
        ("human", "{input}")
    ])
    
    chain = prompt | memoryAgentLLM.get_LLM()
    
    response = chain.invoke({"input": state["messages"][-1].content, 
                             "core_memory": state["core_memory"], 
                             "retrieved_memory": state["retrieved_memory"], 
                             "messages": messages_to_str(state["messages"][:-1])})

    return {"messages": state["messages"] + [response]}

def exceed_memory_limit(state: AgentState) -> bool:
    print("\tInsert memories interaction selected")

    if len(state['messages']) > state['maximum_historical_messages']:
        return True
    return False

def exceed_core_memory_limit(state: AgentState) -> bool:
    print("\tChecking core memory limit")

    if len("\n".join(state['core_memory'])) > state["core_memory_limit"]:
        return True
    return False

def summarize_memories_node(state: AgentState):

    exceeding_messages = state['messages'][:-state['maximum_historical_messages']]
    exceeding_messages = messages_to_str(exceeding_messages)
    print(f"\tSummarizing {exceeding_messages}")

    prompt = ChatPromptTemplate.from_messages([
        ("system", """You are a tool that extracts and adds new facts to the agent's core memories.
        Review the old messages from historical interactions and identify any new, concise, and important facts about the user.
        Add these facts to the core memories, ensuring no redundancy and replacing outdated or contradictory information.
        Only include facts that are relevant and likely to be frequently referenced in future interactions."""),
        ("human", """The messages exceeding the limit are: 
         
        {exceeding_messages}
        
Extract any new facts about the user from these messages and rewrite the old core memories.
Current core memories are: {core_memory}.
Focus on preferences, opinions, or personal facts mentioned by the user.""")
    ])
    summarizer_llm = memoryAgentLLM.get_LLM().bind_tools([InsertCoreMemories])
    chain = prompt | summarizer_llm
    core_memories = {i:cm for i,cm in enumerate(state["core_memory"])}
    response = chain.invoke({"exceeding_messages": exceeding_messages, "core_memory": core_memories})
    print(f"\tSummarization result: {response}")
    return {"tool_calls": state["tool_calls"] + [response], "messages": state["messages"][-state["maximum_historical_messages"]:]}  # Keep only the last N messages

def summarize_core_memories_node(state: AgentState):
    
    print(f"\tSummarizing core memories: {state['core_memory']}")

    prompt = ChatPromptTemplate.from_messages([
        ("system", """You are a tool that decides what to keep in the core memories and what to move to archival memory.
        In the core memories you should keep facts that are most important and could be accessed frequently.
         In the archival memory you should store less frequently accessed information that may be useful for long-term context.
         The limit of the core memory is {core_memory_limit} characters, so you should make sure to keep the core memory under this limit.
         """),
        ("human", """The current core memories are: {core_memory}. 
         The length is {core_memory_length} characters out of the allowed {core_memory_limit} characters.""")])

    summarizer_llm = memoryAgentLLM.get_LLM().bind_tools([SplitCoreAndArchivalMemory])
    chain = prompt | summarizer_llm
    core_memory = "\n".join(state["core_memory"])
    response = chain.invoke({"core_memory": core_memory, "core_memory_length": len(core_memory), "core_memory_limit": state["core_memory_limit"]})
    print(f"\tCore memory summarization result: {response}")
    return {"tool_calls": state["tool_calls"] + [response]}


# Build the graph
graph = StateGraph(AgentState)


graph.add_node("retrieve", retrieval_node)
graph.add_node("execute_tool", tool_node)
graph.add_node("generate_answer", generate_answer)
graph.add_node("router", empty_node)

graph.add_edge("retrieve", "execute_tool")
graph.add_edge("execute_tool", "generate_answer")
graph.add_edge("generate_answer", END)

# Insertion nodes
graph.add_node("insert_memories", empty_node)
graph.add_node("summarize_memories", summarize_memories_node)
graph.add_node("summarize_core_memories", summarize_core_memories_node)
graph.add_node("execute_insertion_tool", tool_node)
graph.add_edge("summarize_memories",  "execute_insertion_tool")
graph.add_edge("execute_insertion_tool", END)
graph.add_edge("summarize_core_memories",  "execute_insertion_tool")

graph.add_conditional_edges(START, interaction_type_node, {'insert': "insert_memories", 'retrieve': "router"})
graph.add_conditional_edges('router', check_information_sufficiency, {True: "generate_answer", False: "retrieve"})

graph.add_conditional_edges('insert_memories', exceed_memory_limit, {True: "summarize_memories", False: END})
graph.add_conditional_edges('execute_insertion_tool', exceed_core_memory_limit, {True: "summarize_core_memories", False: END})

# Compile the graph
memory_agent = graph.compile()

class MemoryAgent():
    _instance = None  # class-level reference to hold the singleton instance

    def __new__(cls, *args, **kwargs):
        if cls._instance is None:  # if no instance exists, create one
            cls._instance = super(MemoryAgent, cls).__new__(cls)
        return cls._instance

    def __init__(self):
        if not hasattr(self, "state"):  # prevents re-initialization
            self.state = {
                "core_memory": [],
                "messages": [],
                "maximum_historical_messages": 5,
                "core_memory_limit": 150,
                "retrieved_memory": "",
                "tool_calls": []
            }
            self.up_to_date = False

    # Function to run the agent
    def run_memory_agent(self, interaction_mode="retrieve"):
        if interaction_mode == "retrieve":
            if self.up_to_date:
                return self.state
            else:
                self.up_to_date = True

        if interaction_mode == "insert":
            self.up_to_date = False
        
        self.state["current_interaction"] = interaction_mode
        print(f"Initial state: {self.state}")
        
        if self.state["messages"] == []:
            print("No messages to process.")
            return self.state
        
        self.state = memory_agent.invoke(self.state)
        for key, value in self.state.items():
            print(f"{key}: {value}")
        self.state["tool_calls"] = []

        return self.state

    def append_message(self, message: str, sender: Literal["user", "assistant"]):
        if sender == "user":
            self.state["messages"].append(
                HumanMessage(content=message, additional_kwargs={}, response_metadata={})
            )
        elif sender == "assistant":
            self.state["messages"].append(
                AIMessage(content=message, additional_kwargs={}, response_metadata={})
            )
        else:
            raise ValueError("Sender must be 'user' or 'assistant'")

# Fill archive with fake memories
# fake_memories = [
#     "User prefers coffee over tea in the morning.",
#     "User likes black tea in the afternoon.",
#     "User's favourite dish is risotto with mushrooms.",
#     "User feels unwell when eating goat cheese.",
#     "User's preferred snack is dark chocolate.",
# ]

# insert_archival_memories.invoke({"memories": fake_memories})
