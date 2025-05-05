import os
from typing import Dict, Any, Optional
from .db_adapter import DBAdapter
from .neo4j_adapter import Neo4jAdapter
from .sql_adapter import SQLAdapter


class DBFactory:
    """
    Factory class to create database adapters based on configuration.
    """
    
    @staticmethod
    def create_adapter(db_type: str, config: Optional[Dict[str, Any]] = None) -> DBAdapter:
        """
        Create a database adapter based on the specified type and configuration
        
        Args:
            db_type: Type of database adapter to create ('neo4j', 'sql', etc.)
            config: Configuration parameters for the adapter
            
        Returns:
            An instance of a DBAdapter implementation
        """
        config = config or {}
        
        if db_type.lower() == 'neo4j':
            uri = config.get('uri', os.getenv("NEO4J_URI"))
            username = config.get('username', os.getenv("NEO4J_USERNAME"))
            password = config.get('password', os.getenv("NEO4J_PASSWORD"))
            
            if not all([uri, username, password]):
                raise ValueError("Missing required Neo4j configuration parameters")
                
            adapter = Neo4jAdapter(uri, username, password)
            adapter.connect()
            return adapter
            
        elif db_type.lower() == 'sql':
            connection_string = config.get('connection_string', os.getenv("SQL_CONNECTION_STRING"))
            
            if not connection_string:
                raise ValueError("Missing required SQL connection string")
                
            adapter = SQLAdapter(connection_string)
            adapter.connect()
            return adapter
            
        else:
            raise ValueError(f"Unsupported database type: {db_type}")