from pydantic import BaseModel, Field



class DishInfoTool(BaseModel):
    """Returns a query to fetch dish info, and optionally evaluate user compatibility."""
    query: str = Field(description="Cypher query to fetch dish information and allergy compatibility.")