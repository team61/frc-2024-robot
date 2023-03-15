package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.recordings.OuterAuto;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.BalancingSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ElevatorSubsystem;

import static frc.robot.Constants.*;
import static frc.robot.Globals.*;

import java.util.function.BooleanSupplier;

import com.kauailabs.navx.frc.AHRS;

public class AutonomousCommand extends CommandBase {
    private final DriveTrain drivetrain;
    private final AHRS gyro;
    private final ElevatorSubsystem elevator;
    private final ArmSubsystem arm;
    private final ClawSubsystem claw;
    private final BalancingSubsystem balancer;
    // private final ArrayList<WPI_TalonFX> components = Robot.components;
    private final double[][] instructionsAxes;
    private final boolean[][] instructionsButtons;
    private boolean[] prevButtonStates = new boolean[]{};
    private int instructionIndex = 0;
    private String autoMode = MIDDLE;
    private boolean finished;

    public AutonomousCommand(DriveTrain dt, AHRS g, ElevatorSubsystem e, ArmSubsystem a, ClawSubsystem c, BalancingSubsystem b) {
        drivetrain = dt;
        gyro = g;
        elevator = e;
        arm = a;
        claw = c;
        balancer = b;

        addRequirements(dt, e, a, c, b);

        new OuterAuto();
        instructionsAxes = OuterAuto.axes;
        instructionsButtons = OuterAuto.buttons;
    }

    @Override
    public void initialize() {
        gyro.zeroYaw();

        autoMode = SmartDashboard.getString("Auto Selector", "middle").equals(OUTER) ? OUTER : MIDDLE;
    }

    @Override
    public void execute() {
        if (autoMode.equals(MIDDLE)) {
            middle();
        } else if (autoMode.equals(OUTER)) {
            outer();
        }
    }

    void middle() {
        new SequentialCommandGroup(
            new ParallelCommandGroup(
                new InstantCommand(claw::close),
                new InstantCommand(claw::rotateUp),
                new InstantCommand(() -> { arm.setVoltage(elevator, -MAX_ARM_VOLTAGE); })
            ),
            new WaitUntilCommand((BooleanSupplier)arm::isFullyExtended),
            new InstantCommand(arm::stop),
            new WaitCommand(0.2),
            new InstantCommand(claw::open),
            new WaitCommand(0.5),
            new ParallelCommandGroup(
                new WaitCommand(0.5).andThen(claw::rotateDown),
                new ParallelRaceGroup(
                    new RepeatCommand(
                        new ParallelCommandGroup(
                            new ConditionalCommand(
                                new InstantCommand(arm::stop),
                                new InstantCommand(() -> { arm.setVoltage(elevator, MAX_ARM_VOLTAGE); }),
                                arm::isFullyRetracted),
                            new ConditionalCommand(
                                new InstantCommand(elevator::stop),
                                new InstantCommand(() -> { elevator.setVoltage(arm, MAX_ELEVATOR_VOLTAGE); }),
                                elevator::isPastMaxUnextendedPosition
                            ),
                            new InstantCommand(() -> {
                                double error = gyro.getRate();
                                drivetrain.tankdrive.driveSpeed(0.4 + error * Math.abs(error / 4), 0.4 - error * Math.abs(error));
                                drivetrain.swervedrive.alignMotors(FORWARDS);
                            })
                        )
                    ),
                    new WaitCommand(2.8)
                )
            ),
            new ParallelCommandGroup(
                new InstantCommand(arm::stop),
                new InstantCommand(elevator::stop),
                new InstantCommand(drivetrain.tankdrive::stop)
            ),
            new WaitCommand(1),
            new ParallelRaceGroup(
                new RepeatCommand(
                    new InstantCommand(() -> {
                        double error = gyro.getRate();
                        drivetrain.tankdrive.driveSpeed(-0.4 + error * Math.abs(error / 4), -0.4 - error * Math.abs(error));
                        drivetrain.swervedrive.alignMotors(FORWARDS);
                    })
                ),
                new WaitCommand(1.5)
            ),
            new WaitCommand(1),
            new ToggleBalancingCommand(drivetrain.swervedrive, balancer)
        ).schedule();
        finished = true;
    }

    void outer() {
        // double[] voltages = instructions[instructionIndex];
        // for (int i = 0; i < voltages.length; i++) {
        //     // components.get(i).setVoltage(voltages[i]);
        //     components.get(i).set(ControlMode.Current, voltages[i]);
        // }
        // drivetrain.swervedrive.alignMotors(FORWARDS);
        // instructionIndex++;
        // if (instructionIndex >= instructions.length) {
        //     finished = true;
        // }
        double[] axes = instructionsAxes[instructionIndex];
        boolean[] buttons = instructionsButtons[instructionIndex];
        if (instructionIndex > 0) {
            prevButtonStates = instructionsButtons[instructionIndex - 1];
        } else {
            prevButtonStates = buttons;
        }

        if (CURRENT_DRIVE_MODE.equals(SWERVE_DRIVE)) {
            double speed = axes[0];
            double rotationVoltage = -axes[2] * MAX_ROTATION_VOLTAGE;
            double error = 0;
            if (CURRENT_DIRECTIONS[0].equals(FORWARDS)) {
                error = gyro.getRate();
            }
            drivetrain.tankdrive.driveSpeed(speed + error * Math.abs(error / 4), speed - error * Math.abs(error));
            drivetrain.swervedrive.setRotationVoltage(rotationVoltage);
            if (!IS_ROTATING && -axes[2] == 0) {
                drivetrain.swervedrive.alignMotors(MIDDLE);
            } else if (IS_ROTATING) {
                drivetrain.swervedrive.alignMotors(DIAGONAL);
            }
        } else {
            double lSpeed = axes[0];
            double rSpeed = axes[1];

            if (Math.abs(lSpeed) > 0.2 && Math.abs(rSpeed) > 0.2) {
                double error = gyro.getRate();
                lSpeed = lSpeed + error * Math.abs(error / 4);
                rSpeed = rSpeed - error * Math.abs(error);
            }
            drivetrain.tankdrive.driveSpeed(lSpeed, rSpeed);
            drivetrain.swervedrive.alignMotors(CURRENT_DIRECTIONS);
        }

        double elevatorVoltage = axes[3] * MAX_ELEVATOR_VOLTAGE;
        elevatorVoltage = clamp(elevatorVoltage, -MAX_ELEVATOR_VOLTAGE, MAX_ELEVATOR_VOLTAGE);
        if (buttons[2]) {
            elevator.setVoltageUnsafe(elevatorVoltage);
        } else {
            elevator.setVoltage(arm, elevatorVoltage);
        }

        double armSpeed = axes[4];
        double armVoltage = armSpeed * MAX_ARM_VOLTAGE;
        if (Math.abs(armSpeed) > 0) {
            armVoltage += Math.signum(armSpeed) * MIN_ARM_VOLTAGE;
        }
        if (buttons[4]) {
            arm.setVoltageUnsafe(armVoltage);
        } else {
            arm.setVoltage(elevator, armVoltage);
        }

        if (buttons[0] && !prevButtonStates[0]) {
            CURRENT_DRIVE_MODE = TANK_DRIVE;
        }

        if (!buttons[0] && prevButtonStates[0]) {
            CURRENT_DRIVE_MODE = SWERVE_DRIVE;
        }

        if (buttons[1] && !prevButtonStates[1]) {
            if (claw.isRotationUninitialized()) {
                claw.rotateUp();
            } else {
                claw.toggleRotation();
            }
        }

        if (buttons[3] && !prevButtonStates[3]) {
            if (claw.isGrabbingUninitialized()) {
                claw.close();
            } else {
                claw.toggleGrab();
            }
        }

        if (buttons[5] && !prevButtonStates[5]) {
            CURRENT_DIRECTIONS = new String[] { DIAGONAL, DIAGONAL, DIAGONAL, DIAGONAL };
            IS_ROTATING = true;
        }

        if (buttons[6] && !prevButtonStates[6]) {
            CURRENT_DIRECTIONS = new String[] { FORWARDS, FORWARDS, FORWARDS, FORWARDS };
            IS_ROTATING = false;
        }

        if (buttons[7] && !prevButtonStates[7]) {
            CURRENT_DIRECTIONS = new String[] { SIDEWAYS, SIDEWAYS, SIDEWAYS, SIDEWAYS };
            IS_ROTATING = false;
        }

        instructionIndex++;
        if (instructionIndex >= instructionsAxes.length) {
            finished = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        instructionIndex = 0;
        finished = false;
        drivetrain.swervedrive.driveSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}
