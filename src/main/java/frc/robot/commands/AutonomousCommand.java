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
import frc.robot.Robot;
import frc.robot.recordings.OuterAuto;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.BalancingSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ElevatorSubsystem;

import static frc.robot.Constants.*;

import java.util.ArrayList;
import java.util.function.BooleanSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

public class AutonomousCommand extends CommandBase {
    private final DriveTrain drivetrain;
    private final AHRS gyro;
    private final ElevatorSubsystem elevator;
    private final ArmSubsystem arm;
    private final ClawSubsystem claw;
    private final BalancingSubsystem balancer;
    private final ArrayList<WPI_TalonFX> components = Robot.components;
    private final double[][] instructions = OuterAuto.voltages;
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
        double[] voltages = instructions[instructionIndex];
        for (int i = 0; i < voltages.length; i++) {
            //components.get(i).setVoltage(voltages[i]);
            components.get(i).set(ControlMode.PercentOutput, voltages[i]);
        }
        drivetrain.swervedrive.alignMotors(FORWARDS);
        instructionIndex++;
        if (instructionIndex >= instructions.length) {
            finished = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        instructionIndex = 0;
        finished = false;
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}
