package frc.robot;

import frc.robot.enums.DriveModulePosition;
import frc.robot.subsystemHelpers.ThrottlePreset;

public class Constants {
    //ids
    
    public static final int[] driveMotorNumbers = new int[] {2, 0, 16, 18};
    public static final int[] angleMotorNumbers = new int[] {3, 1, 17 /*R.I.P. 12/5/2023 1:35:2PM We'll miss you. -Soumith and Noah  :'(  :'( ~ Moved 1/5/2024*/, 19};
    public static final int[] audioMotorNumbers = new int[] {2, 0, 16, 18, 3, 1, 17, 19};
    public static final int[] angleEncoderNumbers = new int[] {22, 23, 24, 21};
    public static final int[] joystickNumbers = new int[] {1, 2, 3, 0};
    public static final int armMotorNumber = 7;
    public static final int handMotorNumber = 6;
    public static final int leftElevatorMotorNumber = 9, rightElevatorMotorNumber = 8;
    public static final int upperLauncherMotorNumber = 5, lowerLauncherMotorNumber = 4;
    public static final int leftFeedMotorNumber = 11, rightFeedMotorNumber = 12;
    public static final int launcherLimitSwitchNumber = 1;

    //drive system

    public static final boolean[] driveMotorInverted = new boolean[] {true, false, true, false};
    public static final boolean[] angleMotorInverted = new boolean[] {true, true, true, true};
    public static final DriveModulePosition[] driveModulePositions = new DriveModulePosition[] {
        DriveModulePosition.FrontLeft,
        DriveModulePosition.FrontRight,
        DriveModulePosition.BackLeft,
        DriveModulePosition.BackRight
    };
    public static final double[] encoderAbsoluteOffsets = new double[] {177.7, 185.8, 330.1, 111.8};
    public static final double gearRatio = 150.0/7.0;

    //swerve system

    public static final Vector2D[] rotationVectors = new Vector2D[] {
        new Vector2D(Math.sqrt(2), Math.sqrt(2)),
        new Vector2D(Math.sqrt(2), -Math.sqrt(2)),
        new Vector2D(-Math.sqrt(2), Math.sqrt(2)),
        new Vector2D(-Math.sqrt(2), -Math.sqrt(2))
    };

    public static final double rotationZeroThreshold = 10;
    public static final double rotationMaxThreshold = 40;

    //arm system

    public static final boolean armMotorInverted = false;
    public static final boolean handMotorInverted = true;
    public static final boolean leftElevatorMotorInverted = true;
    public static final boolean rightElevatorMotorInverted = false;

    public static final double armMotorFactor = 0.5;
    public static final double handMotorFactor = 0.2;
    public static final double elevatorFactor = 1;

    //input system

    public static final ThrottlePreset[] throttlePresets = new ThrottlePreset[] {
        new ThrottlePreset(0.05, 0.05),
        new ThrottlePreset(0.2, 0.2),
        new ThrottlePreset(0.8, 0.2),
        new ThrottlePreset(0, 0)
    };
    public static int defaultThrottlePreset = 3;
    public static final double joystickDeadzoneXMin = -0.01;
    public static final double joystickDeadzoneXMax = 0.01;
    public static final double joystickDeadzoneYMin = -0.2;
    public static final double joystickDeadzoneYMax = 0.01;
    
    public static final double targetAngleMinJoystickMagnitude = 0.5;

    //launcher system

    public static final boolean upperLauncherMotorInverted = false, lowerLauncherMotorInverted = true;
    public static final boolean leftFeedMotorInverted = false, rightFeedMotorInverted = true;

    public static final double launcherMotorIntakeFactor = 0.1;
    public static final double feedMotorIntakeFactor = 0.1;
    public static final double feedMotorOuttakeFactor = 0.1;
    public static final double launcherFireTime = 0.5;

    //tracking system

    public static final double trackingLerpFactor = 0.5;
}