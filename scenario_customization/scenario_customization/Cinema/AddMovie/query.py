from pydantic import BaseModel, Field



class AddMovieTool(BaseModel):
    """Inserts user details (calories, macros, allergies) into the knowledge graph. Additionally, you must create the relations to allergens if provided."""
    query: str = Field(description="Cypher query to insert a user.")