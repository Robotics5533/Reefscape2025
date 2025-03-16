from commands2 import Command
from wpilib import SmartDashboard, Timer
from autonomous.commands.base_command import BaseCommand

def create_for_command(cmd, duration: float, end_func = None) -> Command:
    """
    Creates a command that runs the command for the current duration
    
    Args:
        cmd: command to run
        duration: Time to wait in seconds
        end_func: Optional function to run when the command ends
    """
    class ForCommand(Command):
        def __init__(self):
            super().__init__()
            self.timer = Timer()
            self.cmd = cmd
            self.duration = duration
            self.end_func = end_func
            
        def initialize(self) -> None:
            self.timer.restart()
            
        def execute(self) -> None:
            self.cmd()
            
        def isFinished(self) -> bool:
            return self.timer.hasElapsed(self.duration)
            
        def end(self, interrupted: bool) -> None:
            self.timer.stop()
            if self.end_func is not None:
                self.end_func()
    
    return ForCommand()
