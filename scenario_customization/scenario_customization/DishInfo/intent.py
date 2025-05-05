from pydantic import BaseModel, Field
from typing import List



class DishInfo(BaseModel):
    """User asks you to give him information about a specific dish.
    For example, about its nurtients, allergens or if this dish is suitable for the user."""

    nome_utente: str = Field(description="The name of the user in lowercase", default='')
    nome_piatto: str = Field(description="The name of the dish in lowercase")
    controllo_ingredienti: List[str] = Field(description="Ingredients to check for in the dish", default=[])