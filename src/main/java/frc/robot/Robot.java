// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import lib.components.LogitechJoystick;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.Dictionary;
import java.util.Vector;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import javax.sound.midi.InvalidMidiDataException;
import javax.sound.midi.MidiUnavailableException;
import javax.sound.midi.Transmitter;

import lib.math.Conversions;
import edu.wpi.first.wpilibj.Timer;

import com.ctre.phoenix.motion.BufferedTrajectoryPointStream;
import com.ctre.phoenix.motion.TrajectoryPoint;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.music.Orchestra;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.LEDStrategies.BlinkDecorator;
import frc.robot.LEDStrategies.LEDStrategy;
import frc.robot.LEDStrategies.OscillatoryStrategy;
import frc.robot.LEDStrategies.SolidColorStrategy;
import frc.robot.commands.CalibrateAndZeroAnglesCommand;
import frc.robot.commands.FireLauncherCommand;
import frc.robot.commands.MoveForSecondsCommand;
import frc.robot.commands.ReadyLauncherCommand;
import frc.robot.commands.TargetArmSystemCommand;
import frc.robot.enums.AutonMode;
import frc.robot.enums.LimitedMotorCalibrationStatus;
import frc.robot.commands.TargetAngleCommand;
import frc.robot.subsystemHelpers.DriveModule;
import frc.robot.subsystems.ArmSystem;
import frc.robot.subsystems.DriveSystem;
import frc.robot.subsystems.InputSystem;
import frc.robot.subsystems.LEDSystem;
import frc.robot.subsystems.LauncherSystem;
import frc.robot.subsystems.SwerveSystem;
import frc.robot.subsystems.TrackingSystem;

import com.kauailabs.navx.frc.AHRS;

import java.lang.Math;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSource;

public class Robot extends TimedRobot {
    DriveSystem driveSystem = DriveSystem.get();
    InputSystem inputSystem = InputSystem.get();
    SwerveSystem swerveSystem = SwerveSystem.get();
    ArmSystem armSystem = ArmSystem.get();
    TrackingSystem trackingSystem = TrackingSystem.get();
    LauncherSystem launcherSystem = LauncherSystem.get();
    LEDSystem ledSystem = LEDSystem.get();
    CommandScheduler scheduler = CommandScheduler.getInstance();
    
    double targetAngle = 0;
    boolean calibratingRotation = false;
    AutonMode autonMode;
    double autonTimestamp;
    ArrayList<Double> autonCommandTimes;
    ArrayList<Command> autonCommands;
    boolean overrideLauncher;

    WPI_Pigeon2 gyro = new WPI_Pigeon2(15);
    
    int hue = 0;

    @Override
    public void robotInit() {
        CameraServer.startAutomaticCapture(0);
        CameraServer.startAutomaticCapture(1);

        driveSystem.calibrateAngles();
        driveSystem.zero();
        trackingSystem.calibrateGyro(0);
        
        scheduler.enable();
    }
    
    @Override
    public void robotPeriodic() {
        //auton selection
        if (inputSystem.getLeftAutonModeButton() && autonMode != AutonMode.Left) {
            autonMode = AutonMode.Left;
            System.out.println("Set to left auton position!");
        }
        else if (inputSystem.getCenterAutonModeButton() && autonMode != AutonMode.Center) {
            autonMode = AutonMode.Center;
            System.out.println("Set to central auton position!");
        }
        else if (inputSystem.getRightAutonModeButton() && autonMode != AutonMode.Right) {
            autonMode = AutonMode.Right;
            System.out.println("Set to right auton position!");
        }
        
        //leds
        ledSystem.update();
    }
    
    @Override
    public void disabledInit() {
        ledSystem.strategies = Constants.disabledStrategies;
    }
    
    @Override
    public void disabledPeriodic() {
        //leds
        if (inputSystem.getSpiritLedButton()) {
            ledSystem.strategies = Constants.spiritStrategies;
        }
        else if (!inputSystem.getSpiritLedButton()) {
            ledSystem.strategies = Constants.disabledStrategies;
        }
    }

    @Override
    public void teleopInit() {
        driveSystem.calibrateAngles();
        driveSystem.zero();
        
        swerveSystem.forceTargetAngle(trackingSystem.getYaw());
    }

    @Override
    public void teleopPeriodic() {
        //update tracking (only used for apriltags)

        //trackingSystem.update();
        
        //swervedrive
        
        swerveSystem.updateTranslationVector(inputSystem.getTranslationVector(), trackingSystem.getYaw());

        if (inputSystem.getForcedRotationMode()) { //forced rotation mode is disabled in inputsystem
            swerveSystem.forceRotationPower(inputSystem.getRotationPowerLinear());
        }
        else {
            swerveSystem.updateRotationPower(inputSystem.getRotationPowerLinear());
        }

        swerveSystem.update(trackingSystem.getYaw());

        //arm system

        if (inputSystem.getHandPickupButton()) {
            armSystem.pickup();
        }
        else if (inputSystem.getHandReleaseButton()) {
            armSystem.release();
        }
        else {
            armSystem.stopHand();
        }

        armSystem.setElevatorPower(inputSystem.getElevatorPower());
        armSystem.setArmPowerLinear(inputSystem.getArmPower());

        if (inputSystem.getPickupPresetButton()) {
            armSystem.setPreset(Constants.pickupPreset);
        }
        else if (inputSystem.getAmpPresetButton()) {
            armSystem.setPreset(Constants.ampPreset);
        }
        else if (inputSystem.getHomePresetButton()) {
            armSystem.setPreset(Constants.homePreset);
        }
        else if (inputSystem.getStageStartPresetButton()) {
            armSystem.setPreset(Constants.stageStartPreset);
        }
        else {
            armSystem.setPreset(null);
        }

        if (inputSystem.getElevatorOnlyPresetModeButton()) {
            armSystem.armMotor.targetPosition = null;
        }

        armSystem.update();

        //launcher system

        launcherSystem.update();

        if (inputSystem.getLauncherIntakeButton()) {
            launcherSystem.intake();
        }

        if (inputSystem.getLauncherReadyButton()) {
            launcherSystem.ready(Constants.launcherSpeakerPower);
        }

        if (inputSystem.getLauncherFireButton()) {
            launcherSystem.fire();
        }

        if (inputSystem.getLauncherStopButton()) {
            launcherSystem.setIdle();
        }

        if (inputSystem.getLauncherOverrideModeButton()) {
            launcherSystem.setIdle(inputSystem.getLauncherOverridePower());
            overrideLauncher = true;
        }
        else if (overrideLauncher) {
            launcherSystem.setIdle();
            overrideLauncher = false;
        }

        //angle motor callibration
        if (inputSystem.getCalibrateAngleMotorButton()) {
            driveSystem.calibrateAngles();
        }

        //gyro callibration
        if (inputSystem.getCalibrateGyroButton()) {
            if (calibratingRotation == false) {
                trackingSystem.calibrateGyro(0);
                swerveSystem.forceTargetAngle(null);
                calibratingRotation = true;
            }
        }
        else if (calibratingRotation) {
            swerveSystem.forceTargetAngle(trackingSystem.getYaw());
            calibratingRotation = false;
        }

        //arm system calibration
        if (inputSystem.getCancelCalibrateArmSystemButton()) {
            armSystem.elevatorMotor.CancelManualCalibration();
            armSystem.armMotor.CancelManualCalibration();
        }
        else if (inputSystem.getCalibrateArmSystemButton()) {
            armSystem.elevatorMotor.CalibrateManually();
            armSystem.armMotor.CalibrateManually();
        }

        //leds
        if (inputSystem.getSpiritLedButton()) {
            ledSystem.strategies = Constants.spiritStrategies;
        }
        else if (inputSystem.getLauncherLedButton() == inputSystem.getAmpLedButton()) {
            ledSystem.strategies = Constants.defaultStrategies;
        }
        else if (inputSystem.getLauncherLedButton()) {
            ledSystem.strategies = Constants.launcherStrategies;
        }
        else if (inputSystem.getAmpLedButton()) {
            ledSystem.strategies = Constants.ampStrategies;
        }
    }

    @Override
    public void autonomousInit() {
        //general auton setup
        autonTimestamp = Utils.getTime();

        autonCommandTimes = new ArrayList<Double>();
        autonCommands = new ArrayList<Command>();

        ledSystem.strategies = Constants.autonStrategies;

        //specific auton setup for 2024
        double angle = Constants.autonStartAngles[autonMode.i];

        launcherSystem.overrideLoaded();
        trackingSystem.calibrateGyro(angle);
        swerveSystem.forceTargetAngle(null);

        //queueing auton commands
        autonCommandTimes.add(0.2d);
        autonCommands.add(new CalibrateAndZeroAnglesCommand());

        autonCommandTimes.add(0.4d);
        autonCommands.add(new CalibrateAndZeroAnglesCommand());

        autonCommandTimes.add(0.6d);
        autonCommands.add(new CalibrateAndZeroAnglesCommand());

        autonCommandTimes.add(1d);
        autonCommands.add(new TargetAngleCommand(angle));

        autonCommandTimes.add(2d);
        autonCommands.add(ReadyLauncherCommand.readySpeakerCommand);

        autonCommandTimes.add(3d);
        autonCommands.add(new FireLauncherCommand());

        autonCommandTimes.add(5d);
        autonCommands.add(new MoveForSecondsCommand(new Vector2D(0, 0.6), 1));
    }

    @Override
    public void autonomousPeriodic() {
        double time = Utils.getTime() - autonTimestamp;

        System.out.println(trackingSystem.getYaw());

        while (!autonCommandTimes.isEmpty() && autonCommandTimes.get(0) <= time) {
            autonCommands.get(0).schedule();

            autonCommandTimes.remove(0);
            autonCommands.remove(0);
        }

        scheduler.run();

        swerveSystem.update(trackingSystem.getYaw());
        armSystem.update();
        launcherSystem.update();
    }

    @Override
    public void testInit() {
        
    }

    @Override
    public void testPeriodic() {
        
    }
}
