from pydantic import BaseModel, Field
from typing import List



class MealPreparationTool(BaseModel):
    """Generates 2 queries: 
    first one to check user's allergies,
    second one return the dishes compatible with the user's allergies."""
    query: List[str] = Field(description="List of Cypher queries for meal planning and preparation.")
