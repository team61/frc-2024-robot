// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.cameraserver.CameraServer;
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
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.Vector;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
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

import frc.robot.subsystems.DriveModule;
import frc.robot.subsystems.DriveSystem;
import frc.robot.subsystems.InputSystem;

import com.kauailabs.navx.frc.AHRS;

import java.lang.Math;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class Robot extends TimedRobot {
    DriveSystem driveSystem = DriveSystem.get();
    InputSystem inputSystem = InputSystem.get();
    AHRS gyro = new AHRS(Port.kMXP);

    double targetAngle = 0;

    @Override
    public void robotInit() {
        System.out.println("Callibrating gyro. Don't move robot for the next several seconds...");
        gyro.calibrate();
    }

    @Override
    public void robotPeriodic() {
        //gyro testing
        //System.out.println(gyro.isMagnetometerCalibrated() + ", " + gyro.isCalibrating());
        //System.out.println(gyro.getYaw());

        //prints encoder offsets
        // System.out.println("0: " + new CANCoder(Constants.angleEncoderNumbers[0]).getAbsolutePosition());
        // System.out.println("1: " + new CANCoder(Constants.angleEncoderNumbers[1]).getAbsolutePosition());
        // System.out.println("2: " + new CANCoder(Constants.angleEncoderNumbers[2]).getAbsolutePosition());
        // System.out.println("3: " + new CANCoder(Constants.angleEncoderNumbers[3]).getAbsolutePosition());
        
        //prints joystick 0
        // System.out.println(inputSystem.joysticks[0].getVector().toString());
    }

    @Override
    public void disabledInit() {
        driveSystem.disable();
    }
    
    @Override
    public void disabledPeriodic() {}

    @Override
    public void teleopInit() {
        driveSystem.enable();
        driveSystem.callibrateAngles();
        
        for (DriveModule module : driveSystem.modules) {
            module.setAngle(0);
        }
    }

    @Override
    public void teleopPeriodic() {
        //update throttle
        inputSystem.updateMainThrottle();
        
        //get robot-relative translation vector
        Vector2D translationVector = inputSystem.getTranslationVector();
        translationVector = Vector2D.rotateByDegrees(translationVector, gyro.getYaw());

        //clamp translation vector
        double translationMagnitude = translationVector.magnitude();
        if (translationMagnitude > 1) {
            translationVector = Vector2D.scalarDivide(translationVector, translationMagnitude);
        }
        
        //update target angle
        Vector2D targetAngleVector = inputSystem.getTargetAngleVector();
        double newTargetAngle = -targetAngleVector.theta() + 90;
        if (newTargetAngle > 180) {
            newTargetAngle -= 360;
        }
        if (targetAngleVector.magnitude() >= Constants.targetAngleMinMagnitude) {
            // while (gyro.getYaw() - newTargetAngle > 180) {
            //     newTargetAngle += 360;
            // }
            // while (gyro.getYaw() - newTargetAngle < -180) {
            //     newTargetAngle -= 360;
            // }
            targetAngle = newTargetAngle;
        }

        System.out.println(targetAngle + ", " + gyro.getYaw() + ", " + gyro.isMagnetometerCalibrated() + ", " + gyro.isConnected());

        //get rotation power
        double angleOffset = Math.abs(targetAngle - gyro.getYaw());
        double rotationPower = (angleOffset - Constants.rotationZeroThreshold) / (Constants.rotationMaxThreshold - Constants.rotationZeroThreshold);
        rotationPower = Math.min(Math.max(rotationPower, 0), 1);
        if (targetAngle - gyro.getYaw() < 0) {
            rotationPower *= -1;
        }
        if (angleOffset > 180) {
            rotationPower *= -1;
        }
        
        // Set clockwise or counterclockwise
        rotationPower = inputSystem.getRotationPower();

        //throttle all translation and rotation
        double mainThrottle = inputSystem.getMainThrottle();
        translationVector = Vector2D.scalarMultiply(translationVector, mainThrottle);
        rotationPower *= mainThrottle;

        //send request to drive subsystem
        translationMagnitude = translationVector.magnitude();
        driveSystem.setMovement(translationVector, rotationPower);

        //angle motor callibration
        if (inputSystem.getCallibrateButton()) {
            driveSystem.callibrateAngles();
        }

        //gyro callibration
        if (inputSystem.getResetGyroButton()) {
            gyro.reset();
        }
    }

    /*
    @Override
    public void autonomousInit() {
        
    }

    @Override
    public void autonomousPeriodic() {
        
    }

    @Override
    public void practiceInit() {

    }

    @override void practicePeriodic() {
        
    }

    @Override
    public void testInit() {
        
    }

    @Override
    public void testPeriodic() {
        
    }
    */
}
