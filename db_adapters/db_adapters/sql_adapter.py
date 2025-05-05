from typing import List, Dict, Any, Optional, Union
import sqlalchemy
from sqlalchemy import create_engine, text, MetaData, Table, Column, String, Integer, inspect
from sqlalchemy.exc import SQLAlchemyError
from .db_adapter import DBAdapter


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
        
    def connect(self) -> None:
        """Establish a connection to the SQL database"""
        try:
            self.engine = create_engine(self.connection_string)
            self.metadata = MetaData()
            self.metadata.reflect(bind=self.engine)
            self.inspector = inspect(self.engine)
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
            
        parameters = parameters or {}
        query_results = []
        
        try:
            with self.engine.connect() as connection:
                result = connection.execute(text(query), parameters)
                columns = result.keys()
                
                for row in result:
                    record = {}
                    for i, column in enumerate(columns):
                        record[column] = row[i]
                    query_results.append(record)
                    
            return query_results
            
        except SQLAlchemyError as e:
            print(f"SQL query execution error: {str(e)}")
            return []
    
    def get_schema(self) -> str:
        """
        Get the SQL database schema as a string
        
        Returns:
            String representation of the SQL schema
        """
        if not self.engine:
            self.connect()
            
        schema_info = []
        
        # Get all table names
        table_names = self.inspector.get_table_names()
        
        for table_name in table_names:
            table_info = f"Table: {table_name}\n"
            
            # Get column information
            columns = self.inspector.get_columns(table_name)
            column_info = []
            for column in columns:
                column_type = str(column['type'])
                nullable = "NULL" if column.get('nullable', True) else "NOT NULL"
                primary_key = "PRIMARY KEY" if column.get('primary_key', False) else ""
                column_info.append(f"  - {column['name']} ({column_type}) {nullable} {primary_key}")
            
            table_info += "\n".join(column_info)
            
            # Get foreign key information
            foreign_keys = self.inspector.get_foreign_keys(table_name)
            if foreign_keys:
                fk_info = []
                for fk in foreign_keys:
                    referred_table = fk['referred_table']
                    constrained_columns = ", ".join(fk['constrained_columns'])
                    referred_columns = ", ".join(fk['referred_columns'])
                    fk_info.append(f"  - Foreign Key: ({constrained_columns}) -> {referred_table}({referred_columns})")
                
                table_info += "\n" + "\n".join(fk_info)
                
            schema_info.append(table_info)
            
        return "\n\n".join(schema_info)
    
    def create_node(self, label: str, properties: Dict[str, Any]) -> Dict[str, Any]:
        """
        Create a record in a table (equivalent to a node in graph databases)
        
        Args:
            label: The table name
            properties: Column values
            
        Returns:
            The created record as a dictionary
        """
        if not self.engine:
            self.connect()
            
        # Construct the INSERT query
        columns = ", ".join(properties.keys())
        placeholders = ", ".join([f":{key}" for key in properties.keys()])
        query = f"INSERT INTO {label} ({columns}) VALUES ({placeholders}) RETURNING *"
        
        try:
            result = self.execute_query(query, properties)
            return result[0] if result else {}
        except SQLAlchemyError as e:
            print(f"Error creating record: {str(e)}")
            return {}
    
    def create_relationship(self, start_node_label: str, start_node_props: Dict[str, Any], 
                           end_node_label: str, end_node_props: Dict[str, Any],
                           relationship_type: str, relationship_props: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        """
        Create a relationship between two records (using a junction table)
        
        Args:
            start_node_label: Label of the start table
            start_node_props: Properties to identify the start record
            end_node_label: Label of the end table
            end_node_props: Properties to identify the end record
            relationship_type: Type of relationship (junction table name)
            relationship_props: Optional properties for the relationship
            
        Returns:
            The created relationship as a dictionary
        """
        if not self.engine:
            self.connect()
            
        relationship_props = relationship_props or {}
        
        # First, find the IDs of the start and end records
        start_conditions = " AND ".join([f"{k} = :{k}" for k in start_node_props.keys()])
        start_query = f"SELECT * FROM {start_node_label} WHERE {start_conditions} LIMIT 1"
        start_result = self.execute_query(start_query, start_node_props)
        
        if not start_result:
            print(f"Start record not found in {start_node_label}")
            return {}
        
        end_conditions = " AND ".join([f"{k} = :{k}" for k in end_node_props.keys()])
        end_query = f"SELECT * FROM {end_node_label} WHERE {end_conditions} LIMIT 1"
        end_result = self.execute_query(end_query, end_node_props)
        
        if not end_result:
            print(f"End record not found in {end_node_label}")
            return {}
        
        # Assume the primary key is 'id' - this would need to be adjusted for your schema
        start_id = start_result[0].get('id')
        end_id = end_result[0].get('id')
        
        if not start_id or not end_id:
            print("Could not determine primary keys for relationship")
            return {}
        
        # Create the relationship in the junction table
        rel_props = {
            f"{start_node_label.lower()}_id": start_id,
            f"{end_node_label.lower()}_id": end_id,
            **relationship_props
        }
        
        columns = ", ".join(rel_props.keys())
        placeholders = ", ".join([f":{key}" for key in rel_props.keys()])
        query = f"INSERT INTO {relationship_type} ({columns}) VALUES ({placeholders}) RETURNING *"
        
        try:
            result = self.execute_query(query, rel_props)
            return result[0] if result else {}
        except SQLAlchemyError as e:
            print(f"Error creating relationship: {str(e)}")
            return {}