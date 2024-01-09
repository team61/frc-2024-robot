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

import java.lang.Math;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class Robot extends TimedRobot {
    DriveSystem driveSystem = DriveSystem.get();
    InputSystem inputSystem = InputSystem.get();
    
    TalonFX motor4 = new TalonFX(4);
    TalonFX motor5 = new TalonFX(5);
    LogitechJoystick joystick2 = new LogitechJoystick(Constants.joystickNumbers[2]);


    /*
    @Override
    public void robotInit() {
        
    }

    @Override
    public void robotPeriodic() {
        
    }

    @Override
    public void autonomousInit() {
        
    }

    @Override
    public void autonomousPeriodic() {
        
    }
    */

    @Override
    public void teleopInit() {
        driveSystem.enable();
        driveSystem.config();
        driveSystem.callibrateAngles();
        
        for (DriveModule module : driveSystem.modules) {
            module.setAngle(0);
        }
    }

    @Override
    public void teleopPeriodic() {
        Vector2D translationVector = inputSystem.getTranslationVector();
        double magnitude = Math.min(translationVector.magnitude(), 1);

        for (DriveModule module : driveSystem.modules) {
            if (magnitude != 0) {
                module.setAngle(translationVector.theta());
            }
            
            module.setPower(magnitude * inputSystem.getMainThrottle());

            //System.out.print(Conversions.falconToDegrees(module.angleMotor.getSelectedSensorPosition(), Constants.gearRatio) + ", ");
        }

        if (inputSystem.getCallibrateButton()) {
            driveSystem.callibrateAngles();
        }

        //System.out.println();
    }

    @Override
    public void disabledInit() {
        driveSystem.disable();
        motor4.set(ControlMode.PercentOutput, 0);
        motor5.set(ControlMode.PercentOutput, 0);
    }
    
    @Override
    public void disabledPeriodic() {
        // for (int i = 0; i < 4; i++) {
        //     System.out.print(i + ": " + driveSystem.modules[i].angleEncoder.getAbsolutePosition() + " ");
        // }

        // System.out.println();
    }

    /*
    @Override
    public void practiceInit() {

    }

    @override void practicePeriodic() {
        
    }
    */

    @Override
    public void testInit() {
        // driveSystem.enable();
        // driveSystem.config();
        // driveSystem.callibrateAngles();
        // motor1.set(ControlMode.PercentOutput, .1);
        // motor2.set(ControlMode.PercentOutput, .1);
        

    }

    @Override
    public void testPeriodic() {
        // driveSystem.setMovement(new Vector2D(), inputSystem.getTranslationVector().y * inputSystem.getMainThrottle());
        // int power = Math.min(-joystick2.getYAxis(), 1.0);
        motor4.set(ControlMode.PercentOutput,-joystick2.getYAxis());
        motor5.set(ControlMode.PercentOutput, joystick2.getYAxis());
        System.out.println(-joystick2.getYAxis());
        
    }


}
