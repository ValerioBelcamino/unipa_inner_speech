from typing import List, Dict, Any, Optional, Union
from neo4j import GraphDatabase
from langchain_neo4j import Neo4jGraph
from .db_adapter import DBAdapter


class Neo4jAdapter(DBAdapter):
    """
    Neo4j implementation of the DBAdapter interface.
    This adopts singleton design pattern to have a single instance connected to the DB
    multiple connections are not supported in Neo4j
    """

    _instance = None

    def __new__(cls, **kwargs):
        if cls._instance is None:
            cls._instance = super(Neo4jAdapter, cls).__new__(cls)
        return cls._instance
    
    def __init__(self, uri: str, username: str, password: str):
        """
        Initialize the Neo4j adapter
        
        Args:
            uri: Neo4j connection URI
            username: Neo4j username
            password: Neo4j password
        """

        if not hasattr(self, "_initialized"):
            self.uri = uri
            self.username = username
            self.password = password
            self.driver = None
            self.graph = None
            self._initialized = True
        
    def connect(self) -> None:
        """Establish a connection to the Neo4j database"""
        self.driver = GraphDatabase.driver(self.uri, auth=(self.username, self.password))
        self.graph = Neo4jGraph(self.uri, self.username, self.password)
        
    def disconnect(self) -> None:
        """Close the Neo4j database connection"""
        if self.driver:
            self.driver.close()
        
    def execute_query(self, query: str, parameters: Optional[Dict[str, Any]] = None) -> List[Dict[str, Any]]:
        """
        Execute a Cypher query against the Neo4j database
        
        Args:
            query: Cypher query string
            parameters: Optional parameters for the query
            
        Returns:
            List of records as dictionaries
        """
        if not self.driver:
            self.connect()
            
        parameters = parameters or {}
        query_results = []
        
        with self.driver.session() as session:
            result = session.run(query, parameters)
            
            for record in result:
                recdict = {}
                for key, value in record.items():
                    if isinstance(value, list):
                        # It's a collected list of nodes
                        sublist = []
                        for item in value:
                            if hasattr(item, "items"):
                                subdict = {k: v for k, v in item.items()}
                                sublist.append(subdict)
                            else:
                                sublist.append(item)  # fallback if not a node
                        recdict[key] = sublist
                    elif hasattr(value, "items"):
                        # It's a single node
                        subdict = {k: v for k, v in value.items()}
                        recdict[key] = subdict
                    else:
                        recdict[key] = value  # fallback for primitives
                query_results.append(recdict)
                
        return query_results
    
    def get_schema(self) -> str:
        """
        Get the Neo4j database schema as a string
        
        Returns:
            String representation of the Neo4j schema
        """
        if not self.graph:
            self.connect()
            
        return self.graph.schema
    
    def create_node(self, label: str, properties: Dict[str, Any]) -> Dict[str, Any]:
        """
        Create a node in the Neo4j database
        
        Args:
            label: The node label
            properties: Node properties
            
        Returns:
            The created node as a dictionary
        """
        if not self.driver:
            self.connect()
            
        query = f"CREATE (n:{label} $props) RETURN n"
        result = self.execute_query(query, {"props": properties})
        return result[0] if result else {}
    
    def create_relationship(self, start_node_label: str, start_node_props: Dict[str, Any], 
                           end_node_label: str, end_node_props: Dict[str, Any],
                           relationship_type: str, relationship_props: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        """
        Create a relationship between two nodes in Neo4j
        
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
        if not self.driver:
            self.connect()
            
        relationship_props = relationship_props or {}
        
        query = f"""
        MATCH (a:{start_node_label})
        WHERE {' AND '.join([f'a.{k} = ${k}' for k in start_node_props])}
        MATCH (b:{end_node_label})
        WHERE {' AND '.join([f'b.{k} = ${k+"_end"}' for k in end_node_props])}
        CREATE (a)-[r:{relationship_type} $rel_props]->(b)
        RETURN a, r, b
        """
        
        params = {**start_node_props}
        for k, v in end_node_props.items():
            params[k+"_end"] = v
        params["rel_props"] = relationship_props
        
        result = self.execute_query(query, params)
        return result[0] if result else {}