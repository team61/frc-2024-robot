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
import lib.components.LogitechJoystick;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;

import static frc.robot.Constants.*;
import static frc.robot.Globals.*;

import java.io.File;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

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
    private Alliance team = Alliance.Invalid;
    private double colorOffset = 0;
    private RobotContainer robotContainer;
    private AHRS gyro;
    private Command autoCommand;
    private SendableChooser<String> autoChooser;
    private long teleopStartTime;
    public static final ArrayList<DoubleSupplier> joystickAxes = new ArrayList<>();
    public static final ArrayList<BooleanSupplier> joystickButtons = new ArrayList<>();
    private ArrayList<double[]> recordingAxes = new ArrayList<>();
    private ArrayList<boolean[]> recordingButtons = new ArrayList<>();

    /**
     * This function is run when the robot is first started up and should be used
     * for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        robotContainer = new RobotContainer();
        gyro = robotContainer.getGyro();
        autoCommand = robotContainer.getAutonomousCommand();

        gyro.zeroYaw();
        CameraServer.startAutomaticCapture(0);
        CameraServer.startAutomaticCapture(1);

        autoChooser = new SendableChooser<>();
        autoChooser.setDefaultOption(MIDDLE, MIDDLE);
        autoChooser.addOption(OUTER, OUTER);
        autoChooser.addOption(REC, REC);
        autoChooser.addOption(TEST, TEST);
        SmartDashboard.putData("Auto Selector", autoChooser);

        LogitechJoystick joystick1 = robotContainer.joystick1;
        LogitechJoystick joystick2 = robotContainer.joystick2;
        LogitechJoystick joystick3 = robotContainer.joystick3;
        LogitechJoystick joystick4 = robotContainer.joystick4;
        joystickAxes.add(() -> -joystick1.getYAxis());
        joystickAxes.add(() -> -joystick1.getXAxis());
        joystickAxes.add(() -> -joystick2.getXAxis());
        joystickAxes.add(() -> joystick3.getYAxis(0.15));
        joystickAxes.add(() -> joystick4.getYAxis(0.15));
        joystickButtons.add(joystick1.btn_2);
        joystickButtons.add(joystick3.btn_2);
        joystickButtons.add(joystick4.btn_2);
        joystickButtons.add(joystick3.btn_1);
        joystickButtons.add(joystick4.btn_1);
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
        System.out.println("front left " + robotContainer.swerve.mSwerveMods[0].getAngle().getRadians() + ", ");
        System.out.println("front right " + robotContainer.swerve.mSwerveMods[1].getAngle().getRadians() + ", ");
        System.out.println("back right " + robotContainer.swerve.mSwerveMods[2].getAngle().getRadians() + ", ");
        System.out.println("back left " + robotContainer.swerve.mSwerveMods[3].getAngle().getRadians());
        // drivetrain.swervedrive.alignMotors(FORWARDS);
        // System.out.println("18,19 " + robotContainer.swerve.mSwerveMods[3].getState());
        // BaseTalon base = new BaseTalon(3, "phoenix");

        // TalonFXSensorCollection fx = new TalonFXSensorCollection(base);
        // System.out.println(fx.getIntegratedSensorPosition());
        // robotContainer.swerve.SwerveModuleState[] desiredStates = new SwerveModuleState[4]; // Assuming you have 4 modules

        // // Populate the desiredStates array with the desired states for each module
        // desiredStates[0] = robotContainer.swerve.SwerveModuleState(1.0, Rotation2d.fromDegrees(45.0)); // Example values
        // desiredStates[1] = robotContainer.swerve.SwerveModuleState(1.0, Rotation2d.fromDegrees(135.0)); // Example values
        // desiredStates[2] = robotContainer.swerve.SwerveModuleState(1.0, Rotation2d.fromDegrees(225.0)); // Example values
        // desiredStates[3] = robotContainer.swerve.SwerveModuleState(1.0, Rotation2d.fromDegrees(315.0)); // Example values

// Call the setModuleStates method with the desired states
// swerveDrive.setModuleStates(desiredStates);
        // System.out.println("Roll: " + gyro.getRoll() + ", Pitch: " + gyro.getPitch() + ", Yaw: " + gyro.getYaw());
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
        // drivetrain.swervedrive.disableWheelBreaks();
        team = DriverStation.getAlliance();

        CommandScheduler.getInstance().cancelAll();
        autoCommand.schedule();
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
        
    }

    /** This function is called once when teleop is enabled. */
    @Override
    public void teleopInit() {
        autoCommand.cancel();
        // drivetrain.enableWheelBreaks();
        robotContainer.pneumaticHub.enableCompressorDigital();
        teleopStartTime = System.currentTimeMillis();
        // balancingSubsystem.disable();
        CURRENT_DIRECTIONS = new String[] { FORWARDS, FORWARDS, FORWARDS, FORWARDS };
        IS_ROTATING = false;
        recordingAxes = new ArrayList<>();
        recordingButtons = new ArrayList<>();
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
        LogitechJoystick joystick1 = robotContainer.joystick1;
        LogitechJoystick joystick2 = robotContainer.joystick2;
        LogitechJoystick joystick3 = robotContainer.joystick3;

        

        // var swerve = robotContainer.swerve;
        // var elevator = robotContainer.elevator;
        // var arm = robotContainer.arm;

        // double translationVal = MathUtil.applyDeadband(-joystick1.getYAxis(), 0.15);
        // double strafeVal = MathUtil.applyDeadband(-joystick1.getXAxis(), 0.15);
        // double rotationVal = MathUtil.applyDeadband(-joystick2.getXAxis(), 0.15);

        // swerve.drive(
        //     new Translation2d(translationVal, strafeVal).times(SwerveConstants.maxSpeed), 
        //     rotationVal * SwerveConstants.maxAngularVelocity, 
        //     !joystick1.btn_2.getAsBoolean(),
        //     true);
        
        // double elevatorSpeed = joystick3.getYAxis(0.15);
        // double elevatorVoltage = elevatorSpeed * MAX_ELEVATOR_VOLTAGE;
        // elevatorVoltage = clamp(elevatorVoltage, -MAX_ELEVATOR_VOLTAGE, MAX_ELEVATOR_VOLTAGE);
        // if (joystick3.btn_2.getAsBoolean()) {
        //     elevator.setVoltageUnsafe(elevatorVoltage);
        // } else {
        //     elevator.setVoltage(arm, elevatorVoltage);
        // }

        // double armSpeed = joystick4.getYAxis(0.15);
        // double armVoltage = armSpeed * MAX_ARM_VOLTAGE;
        // if (Math.abs(armSpeed) > 0) {
        //     armVoltage += Math.signum(armSpeed) * MIN_ARM_VOLTAGE;
        // }
        // if (joystick4.btn_2.getAsBoolean()) {
        //     arm.setVoltageUnsafe(armVoltage);
        // } else {
        //     arm.setVoltage(elevator, armVoltage);
        // }

        if (IS_RECORDING) {
            double[] axes = new double[joystickAxes.size()];
            boolean[] buttons = new boolean[joystickButtons.size()];

            for (int i = 0; i < axes.length; i++) {
                axes[i] = joystickAxes.get(i).getAsDouble();
            }
            recordingAxes.add(axes);

            for (int i = 0; i < buttons.length; i++) {
                buttons[i] = joystickButtons.get(i).getAsBoolean();
            }
            recordingButtons.add(buttons);
        }

        // double timeElapsed = (System.currentTimeMillis() - teleopStartTime) / 1000.0;
        // if (timeElapsed > 105) {
        //     if (Math.round(timeElapsed * 2) == Math.floor(timeElapsed * 2)) {
        //         ledStrip.setStripRGB(255, 0, 0);
        //     } else {
        //         ledStrip.off();
        //     }
        // } else {
        //     if (joystick3.getRawAxis(3) > 0) {
        //         ledStrip.setStripRGB(255, 0, 128);
        //     } else {
        //         ledStrip.setStripRGB(255, 128, 0);
        //     }
        // }
    }

    /** This function is called once when the robot is disabled. */
    @Override
    public void disabledInit() {
        // ledStrip.off();
        // drivetrain.swervedrive.enableWheelBreaks();
        // balancingSubsystem.disable();
        IS_RECORDING = false;

        if (recordingAxes.size() > 0) {
            new Thread(() -> {
                File f;
                PrintWriter pw;

                f = new File("/home/lvuser/recording-" + System.currentTimeMillis() + ".json");
                try {
                    if (!f.exists()) {
                        f.createNewFile();
                    } else if (f.delete()) {
                        f.createNewFile();
                    }
                    pw = new PrintWriter(f);
                } catch (IOException e) {
                    e.printStackTrace();
                    return;
                }
                
                pw.print("{\"axes\":[");
                for (int i = 0; i < recordingAxes.size(); i++) {
                    double[] data = recordingAxes.get(i);
                    pw.print("[");
                    for (int j = 0; j < data.length; j++) {
                        double axis = data[j];
                        pw.print(axis + (j < data.length - 1 ? "," : ""));
                    }
                    pw.print("]" + (i < recordingAxes.size() - 1 ? "," : ""));
                }
                pw.print("],\"buttons\":[");
                for (int i = 0; i < recordingButtons.size(); i++) {
                    boolean[] data = recordingButtons.get(i);
                    pw.print("[");
                    for (int j = 0; j < data.length; j++) {
                        boolean btn = data[j];
                        pw.print(btn + (j < data.length - 1 ? "," : ""));
                    }
                    pw.print("]" + (i < recordingButtons.size() - 1 ? "," : ""));
                }
                pw.print("]}");
                pw.close();
            }).start();
        }
    }
    
    /** This function is called periodically when disabled. */
    @Override
    public void disabledPeriodic() {
        // if (robotContainer.joystick3.getRawAxis(3) > 0) {
        //     for (int i = 0; i < ledStrip.getLength(); i++) {
        //         if (Math.floor((i + colorOffset) / 7) % 2 == 0) {
        //             ledStrip.setRGB(i, 255, 128, 0);
        //         } else {
        //             ledStrip.setRGB(i, 255, 0, 128);
        //         }
        //     }
        //     colorOffset += 0.3;
        // } else {
        //     ledStrip.off();
        // }
        // if (Math.abs(gyro.getYaw()) % 360 <= 2) {
        //     ledStrip.setStripRGB(0, 64, 192);
        // } else if (Math.abs(180 - gyro.getYaw()) <= 2) {
        //     ledStrip.setStripRGB(64, 192, 0);
        // } else {
        //     ledStrip.off();
        // }
    }
}
