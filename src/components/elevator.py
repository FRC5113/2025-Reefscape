from magicbot import AutonomousStateMachine, tunable, feedback
from phoenix6.hardware import TalonFX, CANcoder
from wpilib import DigitalInput, MotorControllerGroup
import wpimath.controller
from lemonlib.ctre import LemonTalonFX
from lemonlib.preference import SmartPreference, SmartProfile
import rev

from rev import SparkMax
class Elevator:

    # Motors and encoders
    right_elevator_motor: SparkMax
    left_elevator_motor: SparkMax
    elevator_encoder: CANcoder
    upper_elevator_switch: DigitalInput
    lower_elevator_switch: DigitalInput
    kCarriageMass: float  # kg
    kElevatorDrumRadius: float  # meters (2 inches)
    kMinElevatorHeight: float  # meters (2 inches)
    kMaxElevatorHeight: float  # meters (50 inches)
    kElevatorGearing: float
    elevator_profile: SmartProfile
    motor_group: MotorControllerGroup

    def setup(self):
        """Initialize motors and encoder."""
        self.right_elevator_motor.setInverted(True)

        # Configure motor controllers to follow each other
        self.motor_group = MotorControllerGroup(self.left_elevator_motor, self.right_elevator_motor)

        self.elevator_controller = self.elevator_profile.create_controller(
            f"Elevator"
        )

        # Initialize target height
        self.target_height = self.kMinElevatorHeight

    def set_target_height(self, height: float):
        """Set the target height for the elevator."""
        if height < self.kMinElevatorHeight or height > self.kMaxElevatorHeight:
            raise ValueError("Target height out of range")
        self.target_height = height

    def stop(self):
        """Stop the elevator."""
        self.motor_group.stopMotor()
    
    def move_up(self,varidk):
        """Move the elevator up."""
        self.motor_group.set(varidk)
    def move_down(self,varidk):
        """Move the elevator down."""
        self.motor_group.set(-varidk)
    

    def execute(self):
        """Update the motor control to reach the target height."""
        current_height = self.elevator_encoder.get_position().value
        if self.upper_elevator_switch.get() and self.target_height > current_height:
            self.stop()
            return
        if self.lower_elevator_switch.get() and self.target_height < current_height:
            self.stop()
            return

        output = self.elevator_controller.calculate(current_height, self.target_height)
        self.motor_group.set(output)

    @feedback
    def get_current_height(self) -> float:
        """Get the current height of the elevator."""
        return self.elevator_encoder.get_position().value