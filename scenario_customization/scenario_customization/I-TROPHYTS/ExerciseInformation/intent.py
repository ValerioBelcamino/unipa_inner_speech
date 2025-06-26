from pydantic import BaseModel, Field
from typing import List, Optional



class ExerciseInformation(BaseModel):
    """The tool can respond to two types of user requests:
1. If the user mentions a specific rehabilitation exercise, you can give information about it.
2. If the user does **not** mention any exercises, but asks for general user information (e.g., heart rate), and provides their name, you can still proceed.

Therefore, the `exercise_name` parameter is NOT mandatory when the user just wants general info like heart rate or parameters.
"""

    exercise_name: Optional[str] = Field(description="The name of the exercise in lowercase. Not necessary if the request only concerns user parameters.", default='')
    user_name: Optional[str] = Field(description="The name of the patient in lowercase", default='')
    step_number: Optional[int] = Field(description="The index of a specific step composing an exercise", default=1)
    giorno: Optional[str] = Field(description="Day of the week", examples=['lunedi', 'martedi', 'mercoledi', 'giovedi', 'venerdi', 'sabato', 'domenica'],
                        default='')
    
