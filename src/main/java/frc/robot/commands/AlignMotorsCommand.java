package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;

public class AlignMotorsCommand extends CommandBase {
    private final DriveTrainSubsystem drivetrain;

    public AlignMotorsCommand(DriveTrainSubsystem dt) {
        drivetrain = dt;

        addRequirements(dt);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        drivetrain.alignMotors();
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
