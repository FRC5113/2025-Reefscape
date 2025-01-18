from magicbot import AutonomousStateMachine, tunable, feedback
from phoenix6.hardware import TalonFX, CANcoder
from wpilib import DigitalInput, MotorControllerGroup
import wpimath.controller


class Elevator(AutonomousStateMachine):
    # Tunable parameters for PID control
    kP = tunable(0.1)
    kI = tunable(0.0)
    kD = tunable(0.0)
    kF = tunable(0.0)
    kElevatorGearing = 10.0
    kCarriageMass = 4.0  # kg
    kElevatorDrumRadius = 0.0508  # meters (2 inches)
    kMinElevatorHeight = 0.0508  # meters (2 inches)
    kMaxElevatorHeight = 1.27  # meters (50 inches)

    # Motors and encoders
    right_elevator_motor: TalonFX
    left_elevator_motor: TalonFX
    elevator_encoder: CANcoder
    upper_elevator_switch: DigitalInput
    lower_elevator_switch: DigitalInput

    # PID controller
    controller: wpimath.controller.PIDController

    def __init__(self):
        # Configure motor controllers and encoder
        self.motor_group = MotorControllerGroup(self.left_elevator_motor, self.right_elevator_motor)
        self.controller = wpimath.controller.PIDController(self.kP, self.kI, self.kD)

        # Initialize other variables
        self.target_position = 0  # Encoder ticks target
        self.elevator_encoder.set_position(0)  # Reset encoder at the start

    def set_position(self, position_ticks):
        """Set the target position for the elevator (in encoder ticks)."""
        self.target_position = position_ticks
        self.motor_group.set("Position", position_ticks)

    def stop(self):
        """Stop the elevator motors."""
        self.motor_group.set(0)

    def execute(self):
        """Update motor control logic based on current position and target position."""
        current_position = self.elevator_encoder.get_position()

        # Prevent movement beyond limits
        if not self.lower_elevator_switch.get() and current_position <= 0:
            self.motor_group.set(0)
        elif not self.upper_elevator_switch.get() and current_position >= self.target_position:
            self.motor_group.set(0)
        else:
            # Update motor control with PID
            pid_output = self.controller.calculate(current_position, self.target_position)
            self.motor_group.setVoltage(pid_output)

    def set_target_height(self, target_height):
        """Convert target height (in meters) to encoder ticks and set it."""
        ticks_per_meter = self.kElevatorGearing * 2048  # Example conversion factor
        target_ticks = target_height * ticks_per_meter
        self.set_position(target_ticks)

    @feedback
    def get_current_height(self):
        """Get the current height (in meters) from the encoder."""
        ticks_per_meter = self.kElevatorGearing * 2048  # Example conversion factor
        return self.elevator_encoder.get_position() / ticks_per_meter

    def disabledInit(self):
        """Stop elevator motors when robot is disabled."""
        self.stop()
