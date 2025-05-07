from pydantic import BaseModel, Field



class MovieInfoTool(BaseModel):
    """Returns a query to fetch movie info from a vector store based on the user input."""
    query: str = Field(description="Query to Chroma vector db to fetch most similar movies")