from pydantic import BaseModel, Field
from typing import List, Optional



class SubstituteDish(BaseModel):
    """User asks you to propose an alternative dish based on their allergies and dietary plan.
    Extract necessary information from the user's message. 
    Do not generate any new information, use only what user provided for you."""

    nome_utente: str = Field(description="The name of the user in lowercase")
    ingredienti_rimossi: Optional[List[str]] = Field(description="Ingredients that the user wants to exclude", default=[])
    ingredienti_preferiti: Optional[List[str]] = Field(description="Ingredients that the user wants to include", default=[])
    solo_questi_ingredienti: Optional[List[str]] = Field(description="User wants the dish to consist only of these ingredients. If you fill it, leave ingredienti_preferiti empty", default=[])
    giorno: str = Field(description="Day of the week in italian when the user wants the dish", 
                        examples=['lunedi', 'martedi', 'mercoledi', 'giovedi', 'venerdi', 'sabato', 'domenica'],
                        default='')
    pasto: str = Field(description="Type of meal for which the user wants the dish. Map 'stasera'/'sera' to 'cena', 'mattina' to 'colazione', etc.",
                       examples=['colazione', 'pranzo', 'cena'],
                       default='')
    ha_piano_settimanale: bool = Field(description="Wheter or not the user has a weekly plan in the db", exclude=True)
    