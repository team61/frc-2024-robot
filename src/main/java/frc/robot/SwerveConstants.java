package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import lib.util.COTSFalconSwerveConstants;
import lib.util.SwerveModuleConstants;

public final class SwerveConstants {
    public static final double wheelCircumference = 0.1 * Math.PI;
    public static final double driveGearRatio = 8.14;
    public static final double angleGearRatio = 12.8;

    public static final double maxSpeed = 4.115;
    public static final double maxAngularVelocity = 10.0;

    public static final double openLoopRamp = 0;
    public static final double closedLoopRamp = 0;

    public static final double driveKS = 0.5;
    public static final double driveKV = 0.5;
    public static final double driveKA = 0.5;
    public static final double driveKP = 0.05;
    public static final double driveKI = 0;
    public static final double driveKD = 0;
    public static final double driveKF = 0;

    public static final boolean driveEnableCurrentLimit = false;
    public static final double driveContinuousCurrentLimit = 0;
    public static final double drivePeakCurrentLimit = 0;
    public static final double drivePeakCurrentDuration = 0;

    public static final COTSFalconSwerveConstants chosenModule = 
        COTSFalconSwerveConstants.SDSMK4(COTSFalconSwerveConstants.driveGearRatios.SDSMK4_L1);
    public static final double angleKP = chosenModule.angleKP;
    public static final double angleKI = chosenModule.angleKI;
    public static final double angleKD = chosenModule.angleKD;
    public static final double angleKF = chosenModule.angleKF;

    public static final boolean angleEnableCurrentLimit = false;
    public static final double angleContinuousCurrentLimit = 0;
    public static final double anglePeakCurrentLimit = 0;
    public static final double anglePeakCurrentDuration = 0;

    public static final SwerveModuleConstants mod0 = new SwerveModuleConstants(
        11, 
        10, 
        31, 
        // new Rotation2d(137.375));
        // new Rotation2d(2.40));
        new Rotation2d(0.80));
    public static final SwerveModuleConstants mod1 = new SwerveModuleConstants(
        9, 
        8, 
        32, 
        // new Rotation2d(36.858));
        // new Rotation2d(0.64));
        new Rotation2d(2.45));
    public static final SwerveModuleConstants mod2 = new SwerveModuleConstants(
        19,
        18,
        30,
        //new Rotation2d(162.176));
        // new Rotation2d(2.83));
        new Rotation2d(0.42));
    public static final SwerveModuleConstants mod3 = new SwerveModuleConstants(
        1, 
        0, 
        33, 
        // new Rotation2d(-22.549));
        // new Rotation2d(-0.39));
        new Rotation2d(-2.60));
    

    public static final double wheelBase = 0.47;
    public static final double trackWidth = 0.47;
    public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));
}
