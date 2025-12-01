from pydantic import BaseModel, Field
from typing import List, Optional, Literal


class AnglesReading(BaseModel):
    """Sensor sample collected from an ESP32-S3 device during exercises.
    Contains raw IMU data, exercise context, device identity, and derived motion features.
    """

    time: float = Field(description="Seconds elapsed since the start of the exercise session.")
    accX: float = Field(description="Accelerometer reading on X-axis (m/s²), includes gravity.")
    accY: float = Field(description="Accelerometer reading on Y-axis (m/s²), includes gravity.")
    accZ: float = Field(description="Accelerometer reading on Z-axis (m/s²), includes gravity.")
    gyroX: float = Field(description="Gyroscope reading on X-axis (rad/s).")
    gyroY: float = Field(description="Gyroscope reading on Y-axis (rad/s).")
    gyroZ: float = Field(description="Gyroscope reading on Z-axis (rad/s).")
    device: str = Field(description="MAC address of the sensor device.")
    side: Literal["L", "R"] = Field(description="Body side performing the movement: L=left, R=right.")
    exercise: str = Field(description="Exercise type being performed")
    nth_repetition: int = Field(description="Repetition number within the exercise session.")
    angle: float = Field(description="Estimated angle, in degrees.")
    direction: Literal["up", "down"] = Field(description="Movement direction: up (ascending) or down (descending).")
    # message_type: str = Field(description="Category of message sent by the device (e.g., 'angles_data').")
    # sequence_number: int = Field(description="Sequential packet number within the transmitted block.")
    

class HeartRateReading(BaseModel):
    """Single heart-rate measurement captured by the COLMI R02 smart ring, including
    timestamp, bpm value, device identity, exercise context, and message metadata."""

    time: str = Field(description="Timestamp of the reading in ISO-8601 format.")
    value: int = Field(description="Heart rate in beats per minute (bpm).")
    field: str = Field(description="Type of measurement provided by the device (e.g., 'heartRate').")
    device: str = Field(description="Identifier of the heart-rate device (e.g., smart ring ID).")
    exercise: str = Field(description="Exercise label including side or variant, if applicable.")
    # message_type: str = Field(description="Category of message sent by the device (e.g., 'heartrate_data').")
    # sequence_number: int = Field(description="Sequential packet number for the transmitted reading.")


class SensorReadings(BaseModel):
    """The tool input schema for Sensor Readings."""
    angles_readings: AnglesReading = Field(description="Angle reading from sensors", default=None)
    heart_rate_readings: HeartRateReading = Field(description="Heart rate reading from sensors", default=None)
    