from pydantic import BaseModel, Field
from typing import List, Optional



class AddMovie(BaseModel):
    """A new user asks you to add them to the database. 
    Extract necessary information from the user message. 
    Do not generate any new information, use only what user provided for you."""

    nome_utente: str = Field(description="The name of the user in lowercase")
    calorie: int = Field(description="How many calories user should eat per day", default=0)
    proteine: int = Field(description="How many grams of protein user should eat per day", default=0)
    carboidrati: int = Field(description="How many carbohydrates user should eat per day", default=0)
    grassi: int = Field(description="How many fats user should eat per day", default=0)
    intolleranze: Optional[List[str]] = Field(description="User's intollerances", default='')

mandatory_parameters = ['nome_utente', 'calorie', 'proteine', 'carboidrati', 'grassi']