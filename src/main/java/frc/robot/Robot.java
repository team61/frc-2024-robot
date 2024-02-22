// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.commands.CalibrateAndZeroAnglesCommand;
import frc.robot.commands.MoveForSecondsCommand;
import frc.robot.commands.ReadyLauncherCommand;
import frc.robot.commands.TargetArmSystemCommand;
import frc.robot.enums.AutonMode;
import frc.robot.commands.TargetAngleCommand;
import frc.robot.subsystemHelpers.DriveModule;
import frc.robot.subsystems.ArmSystem;
import frc.robot.subsystems.AudioSystem;
import frc.robot.subsystems.DriveSystem;
import frc.robot.subsystems.InputSystem;
import frc.robot.subsystems.LauncherSystem;
import frc.robot.subsystems.SwerveSystem;
import frc.robot.subsystems.TrackingSystem;

import com.kauailabs.navx.frc.AHRS;

import java.lang.Math;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.cscore.UsbCamera;

public class Robot extends TimedRobot {
    DriveSystem driveSystem = DriveSystem.get();
    InputSystem inputSystem = InputSystem.get();
    SwerveSystem swerveSystem = SwerveSystem.get();
    ArmSystem armSystem = ArmSystem.get();
    TrackingSystem trackingSystem = TrackingSystem.get();
    LauncherSystem launcherSystem = LauncherSystem.get();
    CommandScheduler scheduler = CommandScheduler.getInstance();
    
    double targetAngle = 0;
    boolean calibratingRotation = false;
    AutonMode autonMode;
    double autonTimestamp;
    ArrayList<Double> autonCommandTimes;
    ArrayList<Command> autonCommands;

    WPI_Pigeon2 gyro = new WPI_Pigeon2(15);

    @Override
    public void robotInit() {
        CameraServer.startAutomaticCapture();

        driveSystem.calibrateAngles();
        driveSystem.zero();
        trackingSystem.calibrateGyro(0);

        scheduler.enable();
    }

    @Override
    public void robotPeriodic() {}

    @Override
    public void disabledInit() {}
    
    @Override
    public void disabledPeriodic() {
        if (inputSystem.getLeftAutonModeButton()) {
            autonMode = AutonMode.Left;
        }
        else if (inputSystem.getCenterAutonModeButton()) {
            autonMode = AutonMode.Center;
        }
        else if (inputSystem.getRightAutonModeButton()) {
            autonMode = AutonMode.Right;
        }
    }

    @Override
    public void teleopInit() {
        swerveSystem.forceTargetAngle(trackingSystem.getYaw());
    }

    @Override
    public void teleopPeriodic() {
        //update tracking (only used for apriltags)

        //trackingSystem.update();
        
        //swervedrive
        
        swerveSystem.updateTranslationVector(inputSystem.getTranslationVector(), trackingSystem.getYaw());

        if (inputSystem.getForcedRotationMode()) {
            swerveSystem.forceRotationPower(inputSystem.getRotationPowerLinear());
        }
        else {
            swerveSystem.updateRotationPower(inputSystem.getRotationPowerLinear());
        }

        //swerveSystem.update(trackingSystem.getYaw());

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

        if (inputSystem.getArmPickupMacroButton()) {
            armSystem.elevatorMotor.targetPosition = Constants.armPickupMacroElevatorTarget;
            armSystem.armMotor.targetPosition = Constants.armPickupMacroArmTarget;
        }
        else if (inputSystem.getArmAmpMacroButton()) {
            armSystem.elevatorMotor.targetPosition = Constants.armAmpMacroElevatorTarget;
            armSystem.armMotor.targetPosition = Constants.armAmpMacroArmTarget;
        }
        else if (inputSystem.getArmStageStartMacroButton()) {
            armSystem.elevatorMotor.targetPosition = Constants.armStageStartMacroElevatorTarget;
            armSystem.armMotor.targetPosition = Constants.armStageStartMacroArmTarget;
        }
        else {
            armSystem.elevatorMotor.targetPosition = null;
            armSystem.armMotor.targetPosition = null;
        }

        if (inputSystem.getBalancerEngageButton()) {
            armSystem.engageBalancer();
            
        }
        else if (inputSystem.getBalancerDisengageButton()) {
            armSystem.disengageBalancer();
        }
        else {
            armSystem.stopBalancer();
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



        //System.out.println(armSystem.elevatorMotor.getPosition() + ", " + armSystem.armMotor.getPosition());
    }

    @Override
    public void autonomousInit() {
        autonTimestamp = Utils.getTime();

        autonCommandTimes = new ArrayList<Double>();
        autonCommands = new ArrayList<Command>();



        double angle = Constants.autonStartAngles[autonMode.i];

        launcherSystem.overrideLoaded();
        trackingSystem.calibrateGyro(angle);
        swerveSystem.forceTargetAngle(null);



        autonCommandTimes.add(0.2d);
        autonCommands.add(new CalibrateAndZeroAnglesCommand());

        autonCommandTimes.add(0.4d);
        autonCommands.add(new CalibrateAndZeroAnglesCommand());

        autonCommandTimes.add(0.6d);
        autonCommands.add(new CalibrateAndZeroAnglesCommand());

        autonCommandTimes.add(1d);
        autonCommands.add(new TargetAngleCommand(angle));

        autonCommandTimes.add(3d);
        autonCommands.add(ReadyLauncherCommand.speakerCommand);

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
