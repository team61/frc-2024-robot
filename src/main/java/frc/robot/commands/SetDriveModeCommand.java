package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.DriveTrain;

import static frc.robot.Globals.*;

public class SetDriveModeCommand extends CommandBase {
    private final DriveTrain drivetrain;
    private final String mode;

    public SetDriveModeCommand(DriveTrain dt, String newMode) {
        drivetrain = dt;
        mode = newMode;

        addRequirements(dt);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        CURRENT_DRIVE_MODE = mode;
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return true;
    }
}
