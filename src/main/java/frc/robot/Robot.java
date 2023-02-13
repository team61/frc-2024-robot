// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.BalancingSubsystem;
import frc.robot.subsystems.DatabaseSubsystem;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ElevatorSubsystem;
import lib.components.LogitechJoystick;

import static frc.robot.Constants.*;
import static frc.robot.Globals.*;

import com.kauailabs.navx.frc.AHRS;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    private RobotContainer robotContainer;
    private DriveTrain drivetrain;
    private ElevatorSubsystem elevator;
    private ArmSubsystem arm;
    private AHRS gyro;
    private BalancingSubsystem balancingSubsystem;
    private DatabaseSubsystem db;
    private SequentialCommandGroup autoCommand;

    /**
     * This function is run when the robot is first started up and should be used
     * for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        robotContainer = new RobotContainer();
        drivetrain = robotContainer.getDriveTrain();
        elevator = robotContainer.getElevator();
        arm = robotContainer.getArm();
        gyro = robotContainer.getGyro();
        balancingSubsystem = robotContainer.getBalancer();
        db = robotContainer.getDatabase();
        autoCommand = robotContainer.getAutonomousCommand();

        gyro.zeroYaw();
    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items
     * like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow
     * and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    /**
     * This autonomous (along with the chooser code above) shows how to select
     * between differentd
     * autonomous modes using the dashboard. The sendable chooser code works with
     * the Java
     * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the
     * chooser code and
     * uncomment the getString line to get the auto name from the text box below the
     * Gyro
     *
     * <p>
     * You can add additional auto modes by adding additional comparisons to the
     * switch structure
     * below with additional strings. If using the SendableChooser make sure to add
     * them to the
     * chooser code above as well.
     */
    @Override
    public void autonomousInit() {
        drivetrain.swervedrive.disableBreaks();

        autoCommand.schedule();
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
    }

    /** This function is called once when teleop is enabled. */
    @Override
    public void teleopInit() {
        drivetrain.swervedrive.enableBreaks();
        elevator.enableBreaks();
        robotContainer.pneumaticHub.disableCompressor();
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
        LogitechJoystick joystick1 = robotContainer.joystick1;
        LogitechJoystick joystick2 = robotContainer.joystick2;
        LogitechJoystick joystick3 = robotContainer.joystick3;
        LogitechJoystick joystick4 = robotContainer.joystick4;

        if (CURRENT_DRIVE_MODE == SWERVE_DRIVE) {
            double speed = joystick1.getYAxis() * Math.abs(joystick1.getYAxis());
            double rotationVoltage = -joystick2.getZAxis(0.05) * MAX_ROTATION_VOLTAGE;
            
            if (joystick1.btn_2.getAsBoolean()) {
                speed *= SLOWDOWN_COEFFICIENT;
            }

            if (joystick2.btn_2.getAsBoolean()) {
                rotationVoltage *= SLOWDOWN_COEFFICIENT;
            }

            drivetrain.swervedrive.driveSpeed(speed);
            drivetrain.swervedrive.setRotationVoltage(rotationVoltage);
        } else {
            double lSpeed = joystick1.getYAxis() * Math.abs(joystick1.getYAxis());
            double rSpeed = joystick2.getYAxis() * Math.abs(joystick2.getYAxis());

            if (joystick1.btn_2.getAsBoolean() || joystick2.btn_2.getAsBoolean()) {
                lSpeed *= SLOWDOWN_COEFFICIENT;
                rSpeed *= SLOWDOWN_COEFFICIENT;
            }

            drivetrain.tankdrive.driveSpeed(lSpeed, rSpeed);
        }

        double elevatorSpeed = joystick3.getYAxis(0.05);
        double elevatorVoltage = elevatorSpeed * MAX_ELEVATOR_VOLTAGE;
        if (elevatorSpeed > 0) {
            elevatorVoltage = Math.max(elevatorVoltage, MIN_ELEVATOR_VOLTAGE);
        }
        if (elevatorVoltage < 0) {
            elevatorVoltage = clamp(elevatorVoltage, ELEVATOR_NEGATIVE_LIMIT, 0);
        }
        elevator.setVoltage(elevatorVoltage);

        double armSpeed = joystick4.getYAxis(0.05);
        double armVoltage = armSpeed * MAX_ARM_VOLTAGE;
        if (Math.abs(armSpeed) > 0) {
            armVoltage += Math.signum(armSpeed) * MIN_ARM_VOLTAGE;
        }
        arm.setVoltage(armVoltage);

        // System.out.print("tags: ");
        // for (long tag : db.getTags()) {
        //     System.out.print(tag + "   ");
        // }
        // System.out.println();
        
        // System.out.print("cone: ");
        // for (long tag : db.getConePos()) {
        //     System.out.print(tag + "   ");
        // }
        // System.out.println();
    }

    /** This function is called once when the robot is disabled. */
    @Override
    public void disabledInit() {
        drivetrain.swervedrive.disableBreaks();
        elevator.disableBreaks();
        balancingSubsystem.disable();
    }

    /** This function is called periodically when disabled. */
    @Override
    public void disabledPeriodic() {
        System.out.print(gyro.getAngle() + "   ");
        System.out.print(gyro.getRate() + "   ");
        System.out.print(gyro.getRoll() + "   ");
        System.out.print(gyro.getPitch() + "   ");
        System.out.print(gyro.getYaw() + "   ");
        System.out.print(gyro.getCompassHeading() + "   ");
        System.out.println();
    }

    /** This function is called once when test mode is enabled. */
    @Override
    public void testInit() {
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {
    }

    /** This function is called once when the robot is first started up. */
    @Override
    public void simulationInit() {
    }

    /** This function is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {
    }
}
