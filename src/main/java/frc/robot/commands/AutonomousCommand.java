package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DatabaseSubsystem;
import frc.robot.subsystems.DriveTrain;

import static frc.robot.Constants.*;

public class AutonomousCommand extends CommandBase {
    private final DriveTrain drivetrain;
    private final DatabaseSubsystem db;
    private boolean finished = false;

    public AutonomousCommand(DriveTrain dt, DatabaseSubsystem dbs) {
        drivetrain = dt;
        db = dbs;

        addRequirements(dt, dbs);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        long[] conePos = db.getConePos();
        if (conePos[0] == -1 || conePos[1] == -1) return;

        if (conePos[1] > 50) {
            drivetrain.swervedrive.driveVolts(2);
        } else {
            drivetrain.swervedrive.driveVolts(0);
        }

        if (conePos[0] > CAMERA_WIDTH / 2 - 40) {
            drivetrain.swervedrive.setRotationVoltage(0.5);
        } else if (conePos[0] < CAMERA_WIDTH / 2 + 40) {
            drivetrain.swervedrive.setRotationVoltage(-0.5);
        } else {
            drivetrain.swervedrive.setRotationVoltage(0);
        }

        if (Math.abs(conePos[0] - CAMERA_WIDTH / 2) < 40 && conePos[1] <= 50) {
            finished = true;
        }
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return finished;
    }
}
