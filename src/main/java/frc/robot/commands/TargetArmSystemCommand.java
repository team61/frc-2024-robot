package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSystem;

public class TargetArmSystemCommand extends CommandBase {    
    public static TargetArmSystemCommand pickupMacroCommand = new TargetArmSystemCommand(Constants.armPickupMacroArmTarget, Constants.armPickupMacroElevatorTarget);
    public static TargetArmSystemCommand ampMacroCommand = new TargetArmSystemCommand(Constants.armAmpMacroArmTarget, Constants.armAmpMacroElevatorTarget);
    
    ArmSystem armSystem = ArmSystem.get();
    
    double armAngle, elevatorHeight;

    public TargetArmSystemCommand(double armAngle, double elevatorHeight) {
        this.armAngle = armAngle;
        this.elevatorHeight = elevatorHeight;
    }

    @Override
    public void initialize() {
        armSystem.armMotor.targetPosition = armAngle;
        armSystem.elevatorMotor.targetPosition = elevatorHeight;
    }

    @Override
    public void execute() {
        
    }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
