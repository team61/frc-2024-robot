package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
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
import frc.robot.SwerveConstants;
import frc.robot.recordings.OuterAuto;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.BalancingSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

import static frc.robot.Constants.*;
import static frc.robot.Globals.*;

import java.util.function.BooleanSupplier;

import com.kauailabs.navx.frc.AHRS;

public class AutonomousCommand extends CommandBase {
    private final SwerveDriveSubsystem swervedrive;
    private final DriveTrain drivetrain;
    private final AHRS gyro;
    private final ElevatorSubsystem elevator;
    private final ArmSubsystem arm;
    private final ClawSubsystem claw;
    private final BalancingSubsystem balancer;
    private final double[][] instructionsAxes;
    private final boolean[][] instructionsButtons;
    private boolean[] prevButtonStates = new boolean[]{};
    private int instructionIndex = 0;
    private String autoMode = MIDDLE;
    private boolean finished;

    public AutonomousCommand(SwerveDriveSubsystem sd, DriveTrain dt, AHRS g, ElevatorSubsystem e, ArmSubsystem a, ClawSubsystem c, BalancingSubsystem b) {
        swervedrive = sd;
        drivetrain = dt;
        gyro = g;
        elevator = e;
        arm = a;
        claw = c;
        balancer = b;

        addRequirements(sd, dt, e, a, c, b);

        new OuterAuto();
        instructionsAxes = OuterAuto.axes;
        instructionsButtons = OuterAuto.buttons;
    }

    @Override
    public void initialize() {
        gyro.zeroYaw();

        autoMode = SmartDashboard.getString("Auto Selector", "middle");
    }

    @Override
    public void execute() {
        if (autoMode.equals(MIDDLE)) {
            middle();
        } else if (autoMode.equals(OUTER)) {
            outer();
        } else if (autoMode.equals(REC)) {
            rec();
        }
    }

    void middle() {
        new SequentialCommandGroup(
            new ParallelCommandGroup(
                new InstantCommand(claw::close),
                new InstantCommand(claw::rotateUp)
            ),
            new WaitCommand(0.5),
            new InstantCommand(() -> { arm.setVoltage(elevator, -MAX_ARM_VOLTAGE); }),
            new WaitUntilCommand((BooleanSupplier)arm::shouldPlaceTopBlock),
            new InstantCommand(arm::stop),
            new WaitCommand(0.2),
            new InstantCommand(claw::open),
            new WaitCommand(0.5),
            new ParallelCommandGroup(
                new WaitCommand(1.25).andThen(claw::rotateDown),
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
                            new DriveCommand(
                                swervedrive,
                                () -> { return -0.5; },
                                () -> { return 0; },
                                () -> { return 0; },
                                () -> { return true; })
                        )
                    ),
                    new WaitCommand(2.9)
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
                    new DriveCommand(
                                swervedrive,
                                () -> { return 0.5; },
                                () -> { return 0; },
                                () -> { return 0; },
                                () -> { return true; })
                ),
                new WaitCommand(1)
            ),
            new WaitCommand(1),
            new ToggleBalancingCommand(drivetrain.swervedrive, balancer)
        ).schedule();
        finished = true;
    }

    void outer() {
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
                            new DriveCommand(
                                swervedrive,
                                () -> { return -0.5; },
                                () -> { return 0; },
                                () -> { return 0; },
                                () -> { return true; })
                        )
                    ),
                    new WaitCommand(1.5)
                )
            ),
            new ParallelCommandGroup(
                new InstantCommand(arm::stop),
                new InstantCommand(elevator::stop),
                new InstantCommand(drivetrain.tankdrive::stop)
            )
        ).schedule();
        finished = true;
    }

    void rec() {
        double[] axes = instructionsAxes[instructionIndex];
        boolean[] buttons = instructionsButtons[instructionIndex];
        if (instructionIndex > 0) {
            prevButtonStates = instructionsButtons[instructionIndex - 1];
        } else {
            prevButtonStates = buttons;
        }

        double translationVal = MathUtil.applyDeadband(axes[0], 0.15);
        double strafeVal = MathUtil.applyDeadband(axes[1], 0.15);
        double rotationVal = MathUtil.applyDeadband(axes[2], 0.15);

        swervedrive.drive(
            new Translation2d(translationVal, strafeVal).times(SwerveConstants.maxSpeed), 
            rotationVal * SwerveConstants.maxAngularVelocity, 
            !buttons[0], 
            true);
        
        double elevatorSpeed = axes[3];
        double elevatorVoltage = elevatorSpeed * MAX_ELEVATOR_VOLTAGE;
        elevatorVoltage = clamp(elevatorVoltage, -MAX_ELEVATOR_VOLTAGE, MAX_ELEVATOR_VOLTAGE);
        if (buttons[1]) {
            elevator.setVoltageUnsafe(elevatorVoltage);
        } else {
            elevator.setVoltage(arm, elevatorVoltage);
        }

        double armSpeed = axes[4];
        double armVoltage = armSpeed * MAX_ARM_VOLTAGE;
        if (Math.abs(armSpeed) > 0) {
            armVoltage += Math.signum(armSpeed) * MIN_ARM_VOLTAGE;
        }
        if (buttons[2]) {
            arm.setVoltageUnsafe(armVoltage);
        } else {
            arm.setVoltage(elevator, armVoltage);
        }

        if (buttons[3] && !prevButtonStates[3]) {
            if (claw.isRotationUninitialized()) {
                claw.rotateUp();
            } else {
                claw.toggleRotation();
            }
        }

        if (buttons[4] && !prevButtonStates[4]) {
            if (claw.isGrabbingUninitialized()) {
                claw.close();
            } else {
                claw.toggleGrab();
            }
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
