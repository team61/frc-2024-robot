package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DatabaseSubsystem;
import frc.robot.subsystems.DriveTrain;

import static frc.robot.Constants.*;

import com.kauailabs.navx.frc.AHRS;

public class AutonomousCommand extends CommandBase {
    private final DriveTrain drivetrain;
    private final AHRS gyro;
    private final DatabaseSubsystem db;
    private long startTime;
    private int step;
    private boolean finished = false;

    public AutonomousCommand(DriveTrain dt, AHRS g, DatabaseSubsystem dbs) {
        drivetrain = dt;
        gyro = g;
        db = dbs;

        addRequirements(dt, dbs);
    }

    @Override
    public void initialize() {
        step = 0;
    }

    @Override
    public void execute() {
        drivetrain.swervedrive.alignMotors(FORWARDS);

        double error;
        switch (step) {
            case 0:
                gyro.zeroYaw();
                startTime = System.currentTimeMillis();
                step++;
                break;
            case 1:
                drivetrain.tankdrive.driveVolts(4, 4);
                if (System.currentTimeMillis() - startTime > 500) {
                    drivetrain.tankdrive.driveVolts(0, 0);
                    step++;
                }

                break;
            case 2:
                error = 180 - gyro.getAngle();
                double lVolts = -3 - error * 0.1;
                double rVolts = 3 + error * 0.1;
        
                drivetrain.tankdrive.driveVolts(lVolts, rVolts);
        
                if (Math.abs(gyro.getAngle() - 180) <= 15) {
                    drivetrain.tankdrive.driveVolts(0, 0);
                    startTime = System.currentTimeMillis();
                    step++;
                }

                break;
            case 3:
                error = -gyro.getRate();
                drivetrain.tankdrive.driveSpeed(-0.5 + error * 0.04, -0.5 - error * 0.12);

                if (System.currentTimeMillis() - startTime > 2500) {
                    drivetrain.tankdrive.driveVolts(0, 0);
                    step++;
                }

                break;
            default:
                finished = true;
        }
        // long[] conePos = db.getConePos();
        // if (conePos[0] == -1 || conePos[1] == -1) return;

        // if (conePos[1] > 50) {
        //     drivetrain.swervedrive.driveVolts(2);
        // } else {
        //     drivetrain.swervedrive.driveVolts(0);
        // }

        // if (conePos[0] > CAMERA_WIDTH / 2 - 40) {
        //     drivetrain.swervedrive.setRotationVoltage(0.5);
        // } else if (conePos[0] < CAMERA_WIDTH / 2 + 40) {
        //     drivetrain.swervedrive.setRotationVoltage(-0.5);
        // } else {
        //     drivetrain.swervedrive.setRotationVoltage(0);
        // }

        // if (Math.abs(conePos[0] - CAMERA_WIDTH / 2) < 40 && conePos[1] <= 50) {
        //     finished = true;
        // }
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return finished;
    }
}
