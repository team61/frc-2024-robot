// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.BalancingSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DatabaseSubsystem;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LEDStripSubsystem;
import lib.components.LogitechJoystick;

import static frc.robot.Constants.*;
import static frc.robot.Globals.*;

import java.util.ArrayList;

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
    private final int[][] colors = {
        { 255, 0, 0 },
        { 255, 32, 0 },
        { 255, 128, 0 },
        { 0, 255, 0 },
        { 0, 0, 255 },
        { 255, 0, 255 },
    };
    private final int[][] reds = {
        { 255, 0, 0 },
        { 255, 64, 0 },
        { 255, 128, 0 },
        { 255, 196, 0 },
        { 255, 255, 0 },
        { 255, 255, 255 },
    };
    private final int[][] blues = {
        { 0, 0, 255 },
        { 0, 64, 255 },
        { 0, 128, 255 },
        { 0, 196, 255 },
        { 0, 255, 255 },
        { 255, 255, 255 },
    };
    private int[][] teamColors = colors;
    private double colorOffset = 0;
    private RobotContainer robotContainer;
    private DriveTrain drivetrain;
    private ElevatorSubsystem elevator;
    private ArmSubsystem arm;
    private ClawSubsystem claw;
    private AHRS gyro;
    private BalancingSubsystem balancingSubsystem;
    private LEDStripSubsystem ledStrip;
    private DatabaseSubsystem db;
    private Command autoCommand;
    private SendableChooser<String> autoChooser;
    private long teleopStartTime;
    private final ArrayList<Object> components = new ArrayList<>();

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
        claw = robotContainer.getClaw();
        gyro = robotContainer.getGyro();
        balancingSubsystem = robotContainer.getBalancer();
        ledStrip = robotContainer.getLEDStrip();
        db = robotContainer.getDatabase();
        autoCommand = robotContainer.getAutonomousCommand();

        gyro.zeroYaw();
        CameraServer.startAutomaticCapture();

        autoChooser = new SendableChooser<>();
        autoChooser.setDefaultOption(MIDDLE, MIDDLE);
        autoChooser.addOption(OUTER, OUTER);
        SmartDashboard.putData("Auto Selector", autoChooser);

        for (var swerveUnit : drivetrain.swervedrive.swerveMotors) {
            components.add(swerveUnit.directionMotor);
            components.add(swerveUnit.wheelMotor);
        }
        components.add(elevator.elevatorMotor);
        components.add(arm.armMotor);
        components.add(claw);
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
        SmartDashboard.putNumber("Gyro", gyro.getFusedHeading());

        // System.out.println(gyro.getRate());
        // System.out.println("elevator: " + elevator.getPosition() + ", arm: " + arm.getPosition() + ", limit: " + arm.limitSwitch.get());
        // System.out.print("18,19 " + drivetrain.swervedrive.swerveMotors[0].getRotationPosition() + ", ");
        // System.out.print("10,11 " + drivetrain.swervedrive.swerveMotors[1].getRotationPosition() + ", ");
        // System.out.print("8,9 " + drivetrain.swervedrive.swerveMotors[2].getRotationPosition() + ", ");
        // System.out.println("0,1 " + drivetrain.swervedrive.swerveMotors[3].getRotationPosition());
        System.out.println("Roll: " + gyro.getRoll() + ", Pitch: " + gyro.getPitch() + ", Yaw: " + gyro.getYaw());
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
        drivetrain.swervedrive.disableWheelBreaks();

        teamColors = DriverStation.getAlliance() == Alliance.Red ? reds : blues;
        autoCommand.schedule();
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
        for (int i = 0; i < ledStrip.getLength(); i++) {
            int[] color = teamColors[(int)(i + colorOffset) % teamColors.length];
            ledStrip.setRGB(i, color[0], color[1], color[2]);
        }
        colorOffset += 0.2;
    }

    /** This function is called once when teleop is enabled. */
    @Override
    public void teleopInit() {
        autoCommand.cancel();
        drivetrain.disableWheelBreaks();
        robotContainer.pneumaticHub.enableCompressorDigital();
        teleopStartTime = System.currentTimeMillis();
        balancingSubsystem.disable();
        CURRENT_DIRECTIONS = new String[] { FORWARDS, FORWARDS, FORWARDS, FORWARDS };
        IS_ROTATING = false;
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
        LogitechJoystick joystick1 = robotContainer.joystick1;
        LogitechJoystick joystick2 = robotContainer.joystick2;
        LogitechJoystick joystick3 = robotContainer.joystick3;
        LogitechJoystick joystick4 = robotContainer.joystick4;

        if (CURRENT_DRIVE_MODE == SWERVE_DRIVE) {
            double speed = joystick1.getYAxis() * Math.abs(joystick1.getYAxis(0.15));
            double rotationVoltage = -joystick2.getZAxis(0.05) * MAX_ROTATION_VOLTAGE;
            
            if (joystick1.btn_2.getAsBoolean()) {
                speed *= SLOWDOWN_COEFFICIENT;
            }

            if (joystick2.btn_2.getAsBoolean()) {
                rotationVoltage *= SLOWDOWN_COEFFICIENT;
            }

            double error = 0;
            if (CURRENT_DIRECTIONS[0].equals(FORWARDS)) {
                error = gyro.getRate();
            }
            drivetrain.tankdrive.driveSpeed(speed + error * Math.abs(error / 4), speed - error * Math.abs(error));
            drivetrain.swervedrive.setRotationVoltage(rotationVoltage);
            if (!IS_ROTATING && -joystick2.getZAxis(0.05) == 0) {
                drivetrain.swervedrive.alignMotors(MIDDLE);
            } else if (IS_ROTATING) {
                drivetrain.swervedrive.alignMotors(DIAGONAL);
            }
        } else {
            double lSpeed = joystick1.getYAxis(0.15) * Math.abs(joystick1.getYAxis(0.15));
            double rSpeed = joystick2.getYAxis(0.15) * Math.abs(joystick2.getYAxis(0.15));

            if (joystick1.btn_2.getAsBoolean() || joystick2.btn_2.getAsBoolean()) {
                lSpeed *= SLOWDOWN_COEFFICIENT;
                rSpeed *= SLOWDOWN_COEFFICIENT;
            }

            if (Math.abs(lSpeed) > 0.2 && Math.abs(rSpeed) > 0.2) {
                double error = gyro.getRate();
                lSpeed = lSpeed + error * Math.abs(error / 4);
                rSpeed = rSpeed - error * Math.abs(error);
            }
            drivetrain.tankdrive.driveSpeed(lSpeed, rSpeed);
            drivetrain.swervedrive.alignMotors(CURRENT_DIRECTIONS);
        }

        double elevatorSpeed = joystick3.getYAxis(0.15);
        double elevatorVoltage = elevatorSpeed * MAX_ELEVATOR_VOLTAGE;
        elevatorVoltage = clamp(elevatorVoltage, -MAX_ELEVATOR_VOLTAGE, MAX_ELEVATOR_VOLTAGE);
        if (joystick3.btn_2.getAsBoolean()) {
            elevator.setVoltageUnsafe(elevatorVoltage);
        } else {
            elevator.setVoltage(arm, elevatorVoltage);
        }

        double armSpeed = joystick4.getYAxis(0.15);
        double armVoltage = armSpeed * MAX_ARM_VOLTAGE;
        if (Math.abs(armSpeed) > 0) {
            armVoltage += Math.signum(armSpeed) * MIN_ARM_VOLTAGE;
        }
        if (joystick4.btn_2.getAsBoolean()) {
            arm.setVoltageUnsafe(armVoltage);
        } else {
            arm.setVoltage(elevator, armVoltage);
        }

        double timeElapsed = (System.currentTimeMillis() - teleopStartTime) / 1000.0;
        if (timeElapsed > 105) {
            if (Math.round(timeElapsed * 2) == Math.floor(timeElapsed * 2)) {
                ledStrip.setStripRGB(255, 0, 0);
            } else {
                ledStrip.off();
            }
        } else {
            if (joystick3.getRawAxis(3) > 0) {
                ledStrip.setStripRGB(255, 0, 128);
            } else {
                ledStrip.setStripRGB(255, 128, 0);
            }
        }
    }

    /** This function is called once when the robot is disabled. */
    @Override
    public void disabledInit() {
        ledStrip.off();
        drivetrain.swervedrive.disableWheelBreaks();
        balancingSubsystem.disable();
    }
    
    /** This function is called periodically when disabled. */
    @Override
    public void disabledPeriodic() {
        if (robotContainer.joystick3.getRawAxis(3) > 0) {
            for (int i = 0; i < ledStrip.getLength(); i++) {
                if (Math.floor((i + colorOffset) / 7) % 2 == 0) {
                    ledStrip.setRGB(i, 255, 128, 0);
                } else {
                    ledStrip.setRGB(i, 255, 0, 128);
                }
            }
            colorOffset += 0.3;
        } else {
            ledStrip.off();
        }
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
