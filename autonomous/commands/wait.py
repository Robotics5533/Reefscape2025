from commands2 import Command
from wpilib import Timer
from autonomous.commands.base_command import BaseCommand

def create_wait_command(duration: float) -> Command:
    """
    Creates a command that waits for the specified duration in seconds.
    
    Args:
        duration: Time to wait in seconds
    """
    class WaitCommand(BaseCommand):
        def __init__(self):
            super().__init__()
            self.timer = Timer()
            self.duration = duration
            
        def initialize(self) -> None:
            self.timer.restart()
            
        def execute(self) -> None:
            pass
            
        def isFinished(self) -> bool:
            return self.timer.hasElapsed(self.duration)
            
        def end(self, interrupted: bool) -> None:
            self.timer.stop()
    
    return WaitCommand()
