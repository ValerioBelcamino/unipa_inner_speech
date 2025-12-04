from pydantic import BaseModel, Field



class ExerciseInformationTool(BaseModel):
    """Returns a Cypher query to return the exercise information from the database."""
    query: str = Field(description="Cypher query with the exercise information")
    _DB: str = "neo4j_physio"