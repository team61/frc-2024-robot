package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class ZeroOutMotorsCommand extends CommandBase {
    private final SwerveDriveSubsystem swervedrive;

    public ZeroOutMotorsCommand(SwerveDriveSubsystem sd) {
        swervedrive = sd;

        addRequirements(sd);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        swervedrive.zeroOutRotation();
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return true;
    }
}
