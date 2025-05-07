from typing import List, Dict, Any, Optional, Union
import sqlalchemy
from sqlalchemy import create_engine, text, MetaData, Table, Column, String, Integer, inspect
from sqlalchemy.exc import SQLAlchemyError
from .db_adapter import DBAdapter
from langchain_community.utilities.sql_database import SQLDatabase
from langchain_community.tools.sql_database.tool import QuerySQLDatabaseTool



class SQLAdapter(DBAdapter):
    """
    SQL implementation of the DBAdapter interface.
    Uses SQLAlchemy to support multiple SQL database engines.
    """
    
    def __init__(self, connection_string: str):
        """
        Initialize the SQL adapter
        
        Args:
            connection_string: SQLAlchemy connection string
                Examples:
                - SQLite: 'sqlite:///database.db'
                - PostgreSQL: 'postgresql://username:password@localhost:5432/dbname'
                - MySQL: 'mysql://username:password@localhost:3306/dbname'
        """
        
        self.connection_string = connection_string
        self.engine = None
        self.metadata = None
        self.inspector = None
        self.db = None
        self._prompt = """Given the following user question, generate a syntactically correct SQL query. 
        The purpose of the query is only to read data from the database. Ensure the query strictly adheres to the following rules:

        1. Read-Only Operations: The query must only retrieve data using SELECT. It must not modify the database in any way 
        (e.g., no INSERT, UPDATE, DELETE, DROP, or ALTER statements are allowed).
        2. Limited Rows: Unless explicitly stated in the user's question, limit the query to return at most 20 results. 
        Use the LIMIT clause for this purpose.
        3. Column Selection: Only retrieve the specific columns relevant to the question. Avoid selecting all columns (e.g., avoid SELECT *). 
        Limit the number of columns as much as you can choosing only the ones really necessary.
        4. Unique Results: Use DISTINCT if it is applicable.
        5. Valid Columns and Tables: Use only the column names and table names provided in the schema below. 
        Do not reference columns or tables that are not explicitly listed.
        6. Safe Execution: Avoid performing queries that could potentially degrade database performance 
        (e.g., avoid unbounded searches without LIMIT or filtering conditions).
        7. No Dangerous Clauses: The query must not include any clauses that can alter or harm the database, such as EXECUTE, TRUNCATE, or LOAD DATA.
        8. In case the user question contains a prohibited intentions, return an empty string.

        Schema:
        {schema}"""
        
    def get_prompt(self) -> str:
        """
        Get the prompt for the Neo4j adapter
        
        Returns:
            The prompt string
        """
        schema = self.get_schema()
        self._prompt = self._prompt.format(schema=schema)
        return self._prompt
        
    def connect(self) -> None:
        """Establish a connection to the SQL database"""
        try:
            self.engine = create_engine(self.connection_string)
            self.metadata = MetaData()
            self.metadata.reflect(bind=self.engine)
            self.inspector = inspect(self.engine)
            self.db = SQLDatabase(self.engine)
        except SQLAlchemyError as e:
            raise ConnectionError(f"Failed to connect to SQL database: {str(e)}")
        
    def disconnect(self) -> None:
        """Close the SQL database connection"""
        if self.engine:
            self.engine.dispose()
            
    def execute_query(self, query: str, parameters: Optional[Dict[str, Any]] = None) -> List[Dict[str, Any]]:
        """
        Execute a SQL query against the database
        
        Args:
            query: SQL query string
            parameters: Optional parameters for the query
            
        Returns:
            List of records as dictionaries
        """
        if not self.engine:
            self.connect()
        execute_query_tool = QuerySQLDatabaseTool(db=self.db)
        result = execute_query_tool.invoke(query)
        return result
    
    def get_schema(self) -> str:
        """
        Get the SQL database schema as a string, including up to 3 unique example values per column.

        Returns:
            String representation of the SQL schema with examples.
        """
        if not self.engine:
            self.connect()
        schema = self.db.get_table_info()
        return schema