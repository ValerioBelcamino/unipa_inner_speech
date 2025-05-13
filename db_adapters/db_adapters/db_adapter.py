from abc import ABC, abstractmethod
from typing import List, Dict, Any, Optional, Union


class DBAdapter(ABC):
    """
    Abstract interface for database adapters.
    All database implementations should inherit from this class.
    Abstract methods must be implemented by subclasses.
    Others are optional and can be implemented by subclasses if needed.
    """
    
    @abstractmethod
    def connect(self) -> None:
        """Establish a connection to the database"""
        pass
    
    @abstractmethod
    def disconnect(self) -> None:
        """Close the database connection"""
        pass

    @abstractmethod
    def get_prompt(self) -> str:
        """Get prompt for the query generation"""
        pass
    
    @abstractmethod
    def execute_query(self, query: str, parameters: Optional[Dict[str, Any]] = None) -> List[Dict[str, Any]]:
        """
        Execute a query against the database
        
        Args:
            query: The query string in the database's query language
            parameters: Optional parameters for the query
            
        Returns:
            List of records as dictionaries
        """
        pass
    
    @abstractmethod
    def get_schema(self) -> str:
        """
        Get the database schema as a string
        
        Returns:
            String representation of the database schema
        """
        pass
    
    def create_node(self, label: str, properties: Dict[str, Any]) -> Dict[str, Any]:
        """
        Create a node in the database
        
        Args:
            label: The node label/type
            properties: Node properties
            
        Returns:
            The created node as a dictionary
        """
        pass
    
    def create_relationship(self, start_node_label: str, start_node_props: Dict[str, Any], 
                           end_node_label: str, end_node_props: Dict[str, Any],
                           relationship_type: str, relationship_props: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        """
        Create a relationship between two nodes
        
        Args:
            start_node_label: Label of the start node
            start_node_props: Properties to identify the start node
            end_node_label: Label of the end node
            end_node_props: Properties to identify the end node
            relationship_type: Type of relationship
            relationship_props: Optional properties for the relationship
            
        Returns:
            The created relationship as a dictionary
        """
        pass