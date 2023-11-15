package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import lib.util.COTSFalconSwerveConstants;
import lib.util.SwerveModuleConstants;

public final class SwerveConstants {
    public static final double openLoopRamp = 0;
    public static final double closedLoopRamp = 0;

    public static double driveKS = 0.5;
    public static double driveKV = 0.5;
    public static double driveKA = 0.5;
    public static double driveKD = 0;
    public static double driveKF = 0;
    public static double driveKI = 0;
    public static double driveKP = 0.05;

    public static final boolean driveEnableCurrentLimit = false;
    public static final double driveContinuousCurrentLimit = 0;
    public static final double drivePeakCurrentLimit = 0;
    public static final double drivePeakCurrentDuration = 0;

    public static final COTSFalconSwerveConstants chosenModule = 
        COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L1);
    public static final double angleKP = chosenModule.angleKP;
    public static final double angleKI = chosenModule.angleKI;
    public static final double angleKD = chosenModule.angleKD;
    public static final double angleKF = chosenModule.angleKF;

    public static final double wheelCircumference = Math.PI * chosenModule.wheelDiameter;
    public static final double driveGearRatio = chosenModule.driveGearRatio;
    public static final double angleGearRatio = chosenModule.angleGearRatio;

    public static final double maxSpeed = 4.115;
    public static final double maxAngularVelocity = 7.163; //7.285;

    public static final boolean angleEnableCurrentLimit = false;
    public static final double angleContinuousCurrentLimit = 0;
    public static final double anglePeakCurrentLimit = 0;
    public static final double anglePeakCurrentDuration = 0;

    public static final SwerveModuleConstants frontLeftMod = new SwerveModuleConstants(
        2, 
        3, 
        22, 
        new Rotation2d(0)); //1.87
    public static final SwerveModuleConstants frontRightMod = new SwerveModuleConstants(
        0, 
        1, 
        23, 
        new Rotation2d(0)); //4.73
    public static final SwerveModuleConstants backLeftMod = new SwerveModuleConstants(
        16, 
        17, 
        24, 
        new Rotation2d(0)); //1.94
    public static final SwerveModuleConstants backRightMod = new SwerveModuleConstants(
        18,
        19,
        21,
        new Rotation2d(0)); // 1.69
    

    public static final double wheelBase = 0.546;
    public static final double trackWidth = 0.546;
    public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));
}
