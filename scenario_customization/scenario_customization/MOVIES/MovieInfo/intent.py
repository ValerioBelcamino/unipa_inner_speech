from pydantic import BaseModel, Field
from typing import List, Optional



class MovieInfo(BaseModel):
    """User asks you to give them information about a specific movie.
    You have to extract details of the movie to use it for quering a vector database with movie information.
    
    Extract only the information that is clearly and explicitly present in the user message.
    Do not fill in values using your own knowledge, guesses, or assumptions — even if the answer is obvious or well known. 
    Fields must remain blank or at their default value if not explicitly mentioned by the user.
    If the user provides information that doesn't match any predefined field, add it as a string to the `details` list.
    This is a strict extraction task, not a question-answering task."""

    title: Optional[str] = Field(description="The title of the movie in lowercase", default='')
    director: Optional[str] = Field(description="The name of the movie director in lowercase", default='')
    language: Optional[str] = Field(description="Language of the movie in ISO 639-1 standart", default='')
    genres: Optional[List[str]] = Field(description="The genres of the movie", default='')
    year: Optional[int] = Field(description="The year of the movie", default=0)
    actors: Optional[List[str]] = Field(description="The actors of the movie in lowercase", default=[])
    duration: Optional[int] = Field(description="The duration of the movie in minutes", default=0)
    rating: Optional[float] = Field(description="The rating of the movie", default=0.0)
    details: Optional[List[str]] = Field(description="Additional details about the movie that are helpful to distinguish it from the others", default=[])
