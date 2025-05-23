from pydantic import BaseModel, Field
from typing import List, Optional



class DishInfo(BaseModel):
    """User asks you to give them information about a specific dish.
    For example, about its nurtients, allergens or if this dish is suitable for the user."""

    nome_utente: Optional[str] = Field(description="The name of the user in lowercase", default='')
    nome_piatto: str = Field(description="The name of the dish in lowercase", default='')
    controllo_ingredienti: Optional[List[str]] = Field(description="Ingredients to check for in the dish", default=[])
