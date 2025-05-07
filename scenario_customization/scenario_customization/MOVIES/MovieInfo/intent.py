from pydantic import BaseModel, Field
from typing import List, Optional



class MovieInfo(BaseModel):
    """User asks you to give them information about a specific dish.
    For example, about its nurtients, allergens or if this dish is suitable for the user."""

    title: str = Field(description="The title of the movie in lowercase", default='')
    director: str = Field(description="The name of the movie director in lowercase", default='')
    language: str = Field(description="Language of the movie", default='it')
    genre: str = Field(description="The genre of the movie", default='')
    year: int = Field(description="The year of the movie", default=0)
    actors: List[str] = Field(description="The actors of the movie", default=[])
    duration: int = Field(description="The duration of the movie in minutes", default=0)
    rating: float = Field(description="The rating of the movie", default=0.0)
    details: List[str] = Field(description="Additional details about the movie that are helpful to distinguish it from the others", default=[])
