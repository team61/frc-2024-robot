package frc.robot;

import frc.robot.enums.DriveModulePosition;
import frc.robot.subsystemHelpers.ThrottlePreset;

public class Constants {
    //ids
    
    public static final int[] driveMotorNumbers = new int[] {2, 0, 16, 18};
    public static final int[] angleMotorNumbers = new int[] {3, 1, 17 /*R.I.P. 12/5/2023 1:35:2PM We'll miss you. -Soumith and Noah  :'(  :'( ~ Moved 1/5/2024*/, 19};
    public static final int[] audioMotorNumbers = new int[] {2, 0, 16, 18, 3, 1, 17, 19};
    public static final int[] angleEncoderNumbers = new int[] {22, 23, 24, 21};
    public static final int armMotorNumber = 7;
    public static final int handMotorNumber = 6;
    public static final int balancerMotorNumber = 13;
    public static final int leftElevatorMotorNumber = 9, rightElevatorMotorNumber = 8;
    public static final int upperLauncherMotorNumber = 5, lowerLauncherMotorNumber = 4;
    public static final int leftFeedMotorNumber = 11, rightFeedMotorNumber = 12;

    public static final int launcherLimitSwitchNumber = 1;
    public static final int elevatorLimitSwitchNumber = 2;
    public static final int armLimitSwitchNumber = 3;

    public static final int pigeonNumber = 15;

    public static final int[] joystickNumbers = new int[] {1, 2, 3, 0};

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
    public static final double angleMotorGearRatio = 150.0/7.0;

    //swerve system

    public static final double translationThrottle = 0.8;
    public static final double rotationThrottle = 0.2;

    public static final Vector2D[] rotationVectors = new Vector2D[] {
        new Vector2D(Math.sqrt(2), Math.sqrt(2)),
        new Vector2D(Math.sqrt(2), -Math.sqrt(2)),
        new Vector2D(-Math.sqrt(2), Math.sqrt(2)),
        new Vector2D(-Math.sqrt(2), -Math.sqrt(2))
    };

    public static final double rotationZeroThreshold = 0;
    public static final double rotationMaxThreshold = 50;

    public static final double translationLerpFactor = 0.05;
    public static final double rotationLerpFactor = 1;

    public static final double rotationTargetShiftFactor = 15;

    //arm system

    public static final boolean leftElevatorMotorInverted = true;
    public static final boolean rightElevatorMotorInverted = false;
    public static final boolean armMotorInverted = false;
    public static final boolean handMotorInverted = true;
    public static final boolean balancerMotorInverted = false;

    public static final double elevatorFactor = 1.0;
    public static final double armMotorFactor = 0.8;
    public static final double handMotorFactor = 0.2;
    public static final double balancerMotorFactor = 0.8;

    public static final double elevatorLerpFactor = 1.0;
    public static final double armLerpFactor = 1.0;
    public static final double handLerpFactor = 1.0;
    public static final double balancerLerpFactor = 0.5;

    public static final double armGearRatio = 300.0;
    public static final double elevatorGearRatio = 4050;

    public static final double armZeroThreshold = 0.1;
    public static final double armMaxThreshold = 5;
    public static final double elevatorZeroThreshold = 0.1;
    public static final double elevatorMaxThreshold = 1.0;

    //input system

    // public static final ThrottlePreset[] throttlePresets = new ThrottlePreset[] { //remove eventually
    //     new ThrottlePreset(0.05, 0.05),
    //     new ThrottlePreset(0.2, 0.2),
    //     new ThrottlePreset(0.8, 0.2),
    //     new ThrottlePreset(0, 0)
    // };
    // public static int defaultThrottlePreset = 2;
    public static final double joystickDeadzoneXMin = -0.01;
    public static final double joystickDeadzoneXMax = 0.01;
    public static final double joystickDeadzoneYMin = -0.2;
    public static final double joystickDeadzoneYMax = 0.01;
    
    public static final double targetAngleMinJoystickMagnitude = 0.5;

    //launcher system

    public static final boolean upperLauncherMotorInverted = false, lowerLauncherMotorInverted = true;
    public static final boolean leftFeedMotorInverted = false, rightFeedMotorInverted = true;

    public static final double launcherMotorIntakeFactor = 0.15;
    public static final double feedMotorIntakeFactor = 0.1;
    public static final double feedMotorOuttakeFactor = 0.1;
    public static final double launcherFireTime = 1.0;

    //tracking system

    public static final double trackingLerpFactor = 0.5;

    //auton and macros

    public static final double armPickupMacroArmTarget = 0;
    public static final double armPickupMacroElevatorTarget = 1.0;

    public static final double armAmpMacroArmTarget = 72.0;
    public static final double armAmpMacroElevatorTarget = 14.5;

    public static final double armStageStartMacroArmTarget = 21;
    public static final double armStageStartMacroElevatorTarget = 27.5;

    public static final double launcherSpeakerPower = 0.35;

    public static final double[] autonStartAngles = new double[] {124, 0, -124};
}