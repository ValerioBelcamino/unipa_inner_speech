from pydantic import BaseModel, Field
from typing import List, Optional



class TimetableInfo(BaseModel):
    """User asks you to find a timetable for a movie. 
    For example, the user wants to know when a specific movie is playing in a specific cinema.
    Extract relevant information from the user input and return it in a structured format."""

    title: Optional[str] = Field(description="Movie title in lowercase")
    cinema: Optional[str] = Field(description="Name of the cinema in Title Case", default='')
    language: Optional[str] = Field(description="Language of the movie", default='', examples=['it', 'en'])
    dates: Optional[List[str]] = Field(description="List of dates of the screening in the format YYYY-MM-DD", default=[])
    time: Optional[List[str]] = Field(description="List of times of the screening in the format HH:MM", default=[])
    