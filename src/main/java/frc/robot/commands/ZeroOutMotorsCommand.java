package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;

public class ZeroOutMotorsCommand extends CommandBase {
    private final DriveTrainSubsystem drivetrain;
    private boolean finished = false;

    public ZeroOutMotorsCommand(DriveTrainSubsystem dt) {
        drivetrain = dt;

        addRequirements(dt);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        drivetrain.zeroOutRotation();

        end(false);
    }

    @Override
    public void end(boolean interrupted) {
        finished = true;
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}
