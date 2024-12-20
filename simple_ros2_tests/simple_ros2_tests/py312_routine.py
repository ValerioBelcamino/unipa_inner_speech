import sys
print(f"Python version: {sys.version}")
import rclpy

from langchain_neo4j import Neo4jGraph
from langchain_core.prompts import ChatPromptTemplate, FewShotPromptTemplate, PromptTemplate

from langchain_openai import ChatOpenAI
from langchain_groq import ChatGroq
from langchain_core.output_parsers import StrOutputParser

