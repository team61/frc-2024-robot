package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSystem;

public class CalibrateAndZeroAnglesCommand extends CommandBase {
    private DriveSystem driveSystem = DriveSystem.get();

    public CalibrateAndZeroAnglesCommand() {
        //constructor
    }

    @Override
    public void initialize() {
        driveSystem.calibrateAngles();
        driveSystem.zero();
    }

    @Override
    public void execute() {
        //called repeatedly during execution
    }

    @Override
    public void end(boolean interrupted) {
        //called once at end
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
