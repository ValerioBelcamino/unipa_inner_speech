from .db_adapter import DBAdapter
from qdrant_client import QdrantClient
from langchain_qdrant import QdrantVectorStore
from langchain_mistralai import MistralAIEmbeddings


class QdrantAdapter(DBAdapter):
    """
    Qdrant implementation of the DBAdapter interface.
    This adopts singleton design pattern to have a single instance connected to the DB
    multiple connections are not supported in Qdrant
    """

    _instance = None

    def __new__(cls, *args, **kwargs):
        if cls._instance is None:
            cls._instance = super(QdrantAdapter, cls).__new__(cls)
        return cls._instance
    
    def __init__(self, host: str, api_key: str):
        """
        Initialize the Qdrant adapter
        
        Args:
            host: Qdrant host
            api_key: Qdrant API key
        """
        if not hasattr(self, "_initialized"):
            self.host = host
            self.api_key = api_key
            self.embeddings = MistralAIEmbeddings()
            self.collection_name = "movies_TMDb"
            self.client = None
            self.vector_store = None
            self._initialized = True
            self._prompt = """You are an expert Qdrant vector search translator who understands questions in Italian
        and converts them to Qdrant vector search queries.
        
        The documents in the Qdrant collection are movie descriptions from TMDb in the following format:
        {schema}"""

    def get_schema(self) -> str:
        """
        Get the schema of the Qdrant collection
        
        Returns:
            The schema string
        """
        schema = """page_content=Title: <title>
    Movie is released in <country> on <date> with duration of <int> minutes.
    Director: <director>
    Cast: <cast>
    Rating: <float> (TMDb)
    Genres: <genres> 
    metadata={'source': 'TMDB', 'type': 'info', 'sql_db_title': <title>}"""
        return schema
        
    def get_prompt(self) -> str:
        """
        Get the prompt for the Qdrant adapter
        
        Returns:
            The prompt string
        """
        schema = self.get_schema()
        self._prompt = self._prompt.format(schema=schema)
        return self._prompt
    
    def connect(self) -> None:
        """Establish a connection to the Qdrant database"""
        try:
            self.client = QdrantClient(
                url=self.host,
                api_key=self.api_key
            )
        except Exception as e:
            raise ConnectionError(f"Failed to connect to Qdrant: {e}")
        else:
            self.vector_store = QdrantVectorStore(
                client=self.client,
                embedding=self.embeddings,
                collection_name=self.collection_name
            )
        
    def disconnect(self) -> None:
        """Close the Qdrant database connection"""
        if self.client:
            self.client.close()
            self.client = None
            self.vector_store = None
    
    def execute_query(self, query: str):
        """
        Execute a query against the Qdrant database
        
        Args:
            query: Query string
            
        Returns:
            List of records as dictionaries
        """
        if not self.client:
            self.connect()
        result = self.vector_store.similarity_search(query, k=3)
        return result