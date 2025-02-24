from dataclasses import dataclass
from typing import Optional
from commands2 import Command
from wpilib import Timer

@dataclass
class CommandState:
    start_time: Optional[float] = None
    time_limit: float = 0.0

class BaseCommand:
    def __init__(self, time_limit: float):
        self.state = CommandState(time_limit=time_limit)
    
    def initialize_timer(self):
        """Initialize the command timer"""
        self.state.start_time = Timer.getFPGATimestamp()
    
    def is_timed_out(self) -> bool:
        """Check if the command has exceeded its time limit"""
        if self.state.start_time is None:
            return False
        
        current_time = Timer.getFPGATimestamp()
        return (current_time - self.state.start_time) >= self.state.time_limit
    
    def create_command(self) -> Command:
        """Override this method in derived command classes"""
        raise NotImplementedError
