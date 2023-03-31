package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

import static frc.robot.Constants.*;

public class MoveArmCommand extends CommandBase {
    private final ArmSubsystem arm;
    private final ElevatorSubsystem elevator;
    private final DoubleSupplier armSpeedSup;
    private final BooleanSupplier unsafeFlagSup;

    public MoveArmCommand(ArmSubsystem arm, ElevatorSubsystem elevator, DoubleSupplier armSpeedSup, BooleanSupplier unsafeFlagSup) {
        this.elevator = elevator;
        this.arm = arm;
        this.armSpeedSup = armSpeedSup;
        this.unsafeFlagSup = unsafeFlagSup;

        addRequirements(arm);
    }

    @Override
    public void execute() {
        double armSpeed = armSpeedSup.getAsDouble();
        double armVoltage = armSpeed * MAX_ARM_VOLTAGE;
        if (Math.abs(armSpeed) > 0) {
            armVoltage += Math.signum(armSpeed) * MIN_ARM_VOLTAGE;
        }
        if (unsafeFlagSup.getAsBoolean()) {
            arm.setVoltageUnsafe(armVoltage);
        } else {
            arm.setVoltage(elevator, armVoltage);
        }
    }
}
