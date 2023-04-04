package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

import static frc.robot.Constants.*;

public class MoveElevatorCommand extends CommandBase {
    private final ElevatorSubsystem elevator;
    private final ArmSubsystem arm;
    private final DoubleSupplier elevatorSpeedSup;
    private final BooleanSupplier unsafeFlagSup;

    public MoveElevatorCommand(ElevatorSubsystem elevator, ArmSubsystem arm, DoubleSupplier elevatorSpeedSup, BooleanSupplier unsafeFlagSup) {
        this.elevator = elevator;
        this.arm = arm;
        this.elevatorSpeedSup = elevatorSpeedSup;
        this.unsafeFlagSup = unsafeFlagSup;

        addRequirements(elevator);
    }

    @Override
    public void execute() {
        double elevatorSpeed = elevatorSpeedSup.getAsDouble();
        double elevatorVoltage = elevatorSpeed * MAX_ELEVATOR_VOLTAGE;
        elevatorVoltage = clamp(elevatorVoltage, -MAX_ELEVATOR_VOLTAGE, MAX_ELEVATOR_VOLTAGE);
        if (unsafeFlagSup.getAsBoolean()) {
            elevator.setVoltageUnsafe(elevatorVoltage);
        } else {
            elevator.setVoltage(arm, elevatorVoltage);
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
