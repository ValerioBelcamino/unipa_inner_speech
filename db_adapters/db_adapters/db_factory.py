import os
from typing import Dict, Any, Optional
from .db_adapter import DBAdapter
from .neo4j_adapter import Neo4jAdapter
from .sql_adapter import SQLAdapter
from .qdrant_adapter import QdrantAdapter


class DBFactory:
    """
    Factory class to create database adapters based on configuration.
    """
    
    @staticmethod
    def create_adapter(db_type: str, config: Optional[Dict[str, Any]] = None) -> DBAdapter:
        """
        Create a database adapter based on the specified type and configuration
        
        Args:
            db_type: Type of database adapter to create ('neo4j', 'sqlite', etc.)
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
            
        elif db_type.lower() == 'sqlite':
            db_config = {
                'user': config.get('user', os.getenv("SQL_USER")),
                'password': config.get('password', os.getenv("SQL_PASSWORD")),
                'host': config.get('host', os.getenv("SQL_HOST")),
                'database': config.get('database', os.getenv("SQL_DATABASE"))
            }
            connection_string = f"mysql+pymysql://{db_config['user']}:{db_config['password']}@{db_config['host']}/{db_config['database']}"
            
            if not connection_string:
                raise ValueError("Missing required SQL connection string")
                
            adapter = SQLAdapter(connection_string)
            adapter.connect()
            return adapter

        elif db_type.lower() == 'qdrant':
            host = config.get('host', os.getenv("QDRANT_HOST"))
            api_key = config.get('api_key', os.getenv("QDRANT_API_KEY"))
            
            if not all([host, api_key]):
                raise ValueError("Missing required Qdrant configuration parameters")
                
            adapter = QdrantAdapter(host, api_key)
            adapter.connect()
            return adapter
            
        else:
            raise ValueError(f"Unsupported database type: {db_type}")