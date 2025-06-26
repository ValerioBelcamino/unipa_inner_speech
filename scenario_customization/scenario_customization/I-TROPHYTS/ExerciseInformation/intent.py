from pydantic import BaseModel, Field
from typing import List, Optional



class ExerciseInformation(BaseModel):
    """User asks you to give them information about a specific rehabilitation exercise.
    You have to extract details of the exercise to use it for quering a Neo4j database with exercise information.
    """

    exercise_name: Optional[str] = Field(description="The name of the exercise in lowercase (optional)", default='')
    user_name: Optional[str] = Field(description="The name of the patient in lowercase (optional)", default='')
    step_number: Optional[int] = Field(description="The index of a specific step composing an exercise", default=1)
    giorno: Optional[str] = Field(description="Day of the week", examples=['lunedi', 'martedi', 'mercoledi', 'giovedi', 'venerdi', 'sabato', 'domenica'],
                        default='')
    
