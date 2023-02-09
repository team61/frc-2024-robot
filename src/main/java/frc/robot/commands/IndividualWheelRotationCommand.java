package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class IndividualWheelRotationCommand extends CommandBase {
    private final SwerveDriveSubsystem swervedrive;
    private final int index;
    private final double volts;

    public IndividualWheelRotationCommand(SwerveDriveSubsystem sd, int i, double v) {
        swervedrive = sd;
        index = i;
        volts = v;

        addRequirements(sd);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        swervedrive.rotateIndividualWheel(index, volts);
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return true;
    }
}
