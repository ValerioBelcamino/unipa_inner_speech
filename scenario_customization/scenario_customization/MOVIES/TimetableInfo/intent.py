from pydantic import BaseModel, Field
from typing import List, Optional
import datetime

today = datetime.datetime.now()
today_date = today.strftime("%Y-%m-%d")
today_time = today.strftime("%H:%M")
day_of_week = today.strftime("%A")

class TimetableInfo(BaseModel):
    """User asks you to find a timetable for a movie. 

    For example, the user wants to know when a specific movie is playing in a specific cinema.
    Extract relevant information from the user input and return it in a structured format.
    If the user does not provide one or more of the parameters, leave them empty."""

    title: Optional[str] = Field(description="Movie title in lowercase")
    cinema: Optional[str] = Field(description="Name of the cinema in Genova in Title Case", default='', examples=['UCI Fiumara', 'Circuito Odeon', 'Circuito America', 'Circuito San Pietro'])
    language: Optional[str] = Field(description="Language of the movie, using ISO 639-1 codes", default='', examples=['it', 'en'])
    dates: Optional[List[str]] = Field(description=f"List of dates of the screening in the format YYYY-MM-DD based on the fact that today is {day_of_week} {today_date}", default=[])
    time: Optional[List[str]] = Field(description=f"List of times of the screening in the format HH:MM based on the fact that now is {today_time}", default=[])
    