from pydantic import BaseModel, Field
from typing import List



class MovieTimetableTool(BaseModel):
    """Generates an SQL query to fetch movie information from a database of movie timetables based on user input."""
    query: str = Field(description="A valid SQL query to fetch movie info from a database")
    _DB: str = 'sqlite'
