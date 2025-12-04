from pydantic import BaseModel, Field



class SensorReadingsTool(BaseModel):
    """Returns a Cypher query to return the sensor readings information from the database."""
    query: str = Field(description="Cypher query with the sensor readings information")
    _DB: str = "neo4j_physio"