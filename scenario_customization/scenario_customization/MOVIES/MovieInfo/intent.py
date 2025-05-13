from pydantic import BaseModel, Field
from typing import List, Optional



class MovieInfo(BaseModel):
    """User asks you to give them information about a specific movie.
    You have to extract details of the movie to use it for quering a vector database with movie information."""

    title: Optional[str] = Field(description="The title of the movie in lowercase", default='')
    director: Optional[str] = Field(description="The name of the movie director in lowercase", default='')
    language: Optional[str] = Field(description="Language of the movie", default='it')
    genre: Optional[str] = Field(description="The genre of the movie", default='')
    year: Optional[int] = Field(description="The year of the movie", default=0)
    actors: Optional[List[str]] = Field(description="The actors of the movie", default=[])
    duration: Optional[int] = Field(description="The duration of the movie in minutes", default=0)
    rating: Optional[float] = Field(description="The rating of the movie", default=0.0)
    details: Optional[List[str]] = Field(description="Additional details about the movie that are helpful to distinguish it from the others", default=[])
