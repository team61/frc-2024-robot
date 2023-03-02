package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DatabaseSubsystem;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ElevatorSubsystem;

import static frc.robot.Constants.*;

import com.kauailabs.navx.frc.AHRS;

public class AutonomousCommand extends CommandBase {
    private final DriveTrain drivetrain;
    private final AHRS gyro;
    private final ElevatorSubsystem elevator;
    private final ArmSubsystem arm;
    private final ClawSubsystem claw;
    private final DatabaseSubsystem db;
    private int step;
    private long startTime;
    private String autoMode = MIDDLE;
    private boolean finished = false;

    public AutonomousCommand(DriveTrain dt, AHRS g, ElevatorSubsystem e, ArmSubsystem a, ClawSubsystem c, DatabaseSubsystem dbs) {
        drivetrain = dt;
        gyro = g;
        elevator = e;
        arm = a;
        claw = c;
        db = dbs;

        addRequirements(dt, e, a, c, dbs);
    }

    @Override
    public void initialize() {
        step = 0; // 0: start, 4: drive, 7: done
        startTime = -1;
        gyro.zeroYaw();

        autoMode = SmartDashboard.getString("Auto Selector", "middle") == OUTER ? OUTER : MIDDLE;
    }

    @Override
    public void execute() {
        double error;
        switch (step) {
            case 0:
                claw.close();
                claw.rotateUp();
                step++;
                break;
            case 1:
                arm.setVoltage(elevator, -MAX_ARM_VOLTAGE);
                if (arm.getPosition() < ARM_MIN_POSITION + 1000) {
                    step++;
                }
                break;
            case 2:
                arm.setVoltageUnsafe(0);
                claw.open();
                if (startTime == -1) {
                    startTime = System.currentTimeMillis();
                } else if (System.currentTimeMillis() - startTime > 500) {
                    startTime = -1;
                    step++;
                }
                break;
            case 3:
                if (arm.getPosition() <= ARM_MAX_POSITION - 5000) {
                    arm.setVoltage(elevator, MAX_ARM_VOLTAGE);
                } else {
                    claw.rotateDown();
                    arm.setVoltageUnsafe(0);
                    elevator.setVoltage(arm, MAX_ELEVATOR_VOLTAGE);
                    if (elevator.getPosition() > ELEVATOR_MAX_UNEXTENDED_POSITION) {
                        step++;
                    }
                }
                break;
            case 4:
                elevator.setVoltageUnsafe(0);
                startTime = System.currentTimeMillis();
                step++;
                break;
            case 5:
                error = -gyro.getRate();
                drivetrain.swervedrive.alignMotors(FORWARDS);
                if (autoMode.equals(MIDDLE)) {
                    drivetrain.tankdrive.driveSpeed(0.35 + error * 0.25, 0.35 - error * 0.27);

                    if (System.currentTimeMillis() - startTime > 3500) {
                        drivetrain.tankdrive.driveVolts(0, 0);
                        startTime = System.currentTimeMillis();
                        step++;
                    }
                } else {
                    double diff = System.currentTimeMillis() - startTime;
                    double coeff = diff / 3250 / 4;
                    drivetrain.tankdrive.driveSpeed((coeff + 0.1) + error * -Math.abs(error), (coeff + 0.1) - error * Math.abs(error));
                
                    if (System.currentTimeMillis() - startTime > 3250) {
                        drivetrain.tankdrive.driveVolts(0, 0);
                        startTime = System.currentTimeMillis();
                        step++;
                    }
                }

                break;
            case 6:
                drivetrain.swervedrive.alignMotors(DIAGONAL);

                if (System.currentTimeMillis() - startTime > 2000) {
                    step++;
                }
                break;
            default:
                finished = true;
        }

        // double error;
        // switch (step) {
        //     case 0:
        //         gyro.zeroYaw();
        //         startTime = System.currentTimeMillis();
        //         step++;
        //         break;
        //     case 1:
        //         drivetrain.tankdrive.driveVolts(4, 4);
        //         if (System.currentTimeMillis() - startTime > 500) {
        //             drivetrain.tankdrive.driveVolts(0, 0);
        //             step++;
        //         }

        //         break;
        //     case 2:
        //         error = 180 - gyro.getAngle();
        //         double lVolts = -3 - error * 0.1;
        //         double rVolts = 3 + error * 0.1;
        
        //         drivetrain.tankdrive.driveVolts(lVolts, rVolts);
        
        //         if (Math.abs(gyro.getAngle() - 180) <= 15) {
        //             drivetrain.tankdrive.driveVolts(0, 0);
        //             startTime = System.currentTimeMillis();
        //             step++;
        //         }

        //         break;
        //     case 3:
        //         error = -gyro.getRate();
        //         drivetrain.tankdrive.driveSpeed(-0.5 + error * 0.04, -0.5 - error * 0.12);

        //         if (System.currentTimeMillis() - startTime > 2500) {
        //             drivetrain.tankdrive.driveVolts(0, 0);
        //             step++;
        //         }

        //         break;
        //     default:
        //         finished = true;
        // }
    }

    @Override
    public void end(boolean interrupted) {
        step = 0;
        finished = false;
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}
