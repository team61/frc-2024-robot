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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import lib.components.LogitechJoystick;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

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

public class Robot extends TimedRobot {
    DriveSystem driveSystem = DriveSystem.get();
    InputSystem inputSystem = InputSystem.get();
    SwerveSystem swerveSystem = SwerveSystem.get();
    ArmSystem armSystem = ArmSystem.get();
    TrackingSystem trackingSystem = TrackingSystem.get();
    LauncherSystem launcherSystem = LauncherSystem.get();
    
    double targetAngle = 0;

    @Override
    public void robotInit() {
        System.out.println("Callibrating gyro. Please refrain from sending any inputs to the robot for the next several seconds... if else.");
    }

    @Override
    public void robotPeriodic() {
        
    }

    @Override
    public void disabledInit() {
        
    }
    
    @Override
    public void disabledPeriodic() {}

    @Override
    public void teleopInit() {
        driveSystem.callibrateAngles();
        
        for (DriveModule module : driveSystem.modules) {
            module.setAngle(0);
        }
    }

    @Override
    public void teleopPeriodic() {
        //update tracking

        //trackingSystem.update();
        
        //swervedrive
        
        inputSystem.updateThrottle();
        
        swerveSystem.updateTranslationVector(inputSystem.getTranslationVector(), trackingSystem.getYaw());
        
        //(follow apriltag)

        //Vector2D dir = Vector2D.subtract(new Vector2D(0, 1), trackingSystem.getPosition());

        // if (dir.magnitude() < 0.1) {
        //     dir = new Vector2D();
        // }
        // else {
        //     dir = dir.normalized();
        // }

        // swerveSystem.updateTranslationVector(dir, trackingSystem.getYaw());

        swerveSystem.updateTargetAngle(inputSystem.getTargetAngleVector());
        if (inputSystem.getLinearRotationMode()) {
            swerveSystem.updateRotationPowerLinear(inputSystem.getRotationPowerLinear());
        }
        else {
            swerveSystem.updateRotationPowerTargetted(trackingSystem.getYaw());
        }

        swerveSystem.throttleTranslation(inputSystem.getTranslationThrottle());
        swerveSystem.throttleRotation(inputSystem.getRotationThrottle());

        swerveSystem.apply(driveSystem);

        //hand

        if (inputSystem.getHandPickupButton()) {
            armSystem.pickup();
        }
        else if (inputSystem.getHandReleaseButton()) {
            armSystem.release();
        }
        else {
            armSystem.stopHand();
        }

        //arm system

        //armSystem.setElevatorPower(inputSystem.getElevatorPower());
        armSystem.setArmPower(inputSystem.getArmPower());

        //angle motor callibration
        if (inputSystem.getCallibrateButton()) {
            driveSystem.callibrateAngles();
        }

        //gyro callibration
        if (inputSystem.getResetGyroButton()) {
            trackingSystem.resetGyro();
        }
    }

    @Override
    public void autonomousInit() {
        
    }

    @Override
    public void autonomousPeriodic() {
        
    }

    @Override
    public void testInit() {
        
    }

    @Override
    public void testPeriodic() {
        //launcher
        
        // TalonFX motor = new TalonFX(4);
        // TalonFX motor2 = new TalonFX(5);
        // LogitechJoystick joystick = new LogitechJoystick(2);

        // motor.set(TalonFXControlMode.PercentOutput, -joystick.getY());
        // motor2.set(TalonFXControlMode.PercentOutput, joystick.getY());






        //launcher system

        launcherSystem.update();

        if (inputSystem.getLauncherIntakeButton()) {
            launcherSystem.intake();
        }

        if (inputSystem.getLauncherReadyButton()) {
            launcherSystem.ready(0.1);
        }

        if (inputSystem.getLauncherFireButton()) {
            launcherSystem.fire();
        }

        if (inputSystem.getLauncherCancelButton()) {
            launcherSystem.cancel();
        }
    }
    
}
