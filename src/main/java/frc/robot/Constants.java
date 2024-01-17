package frc.robot;

import frc.robot.enums.DriveModulePosition;

public class Constants {
    public static final int[] driveMotorNumbers = new int[] {2, 0, 16, 18};
    public static final int[] angleMotorNumbers = new int[] {3, 1, 17 /*R.I.P. 12/5/2023 1:35:2PM We'll miss you. -Soumith and Noah  :'(  :'( ~ Moved 1/5/2024*/, 19};
    public static final int[] driveEncoderNumbers = new int[] {}; //UNKNOWN
    public static final int[] angleEncoderNumbers = new int[] {22, 23, 24, 21};
    public static final boolean[] driveMotorInverted = new boolean[] {true, false, true, false};
    public static final boolean[] angleMotorInverted = new boolean[] {true, true, true, true};
    public static final DriveModulePosition[] driveModulePositions = new DriveModulePosition[] {
        DriveModulePosition.FrontLeft,
        DriveModulePosition.FrontRight,
        DriveModulePosition.BackLeft,
        DriveModulePosition.BackRight
    };

    //public static final double[] encoderAbsoluteOffsets = new double[] {141, 186, 334, 111};
    public static final double[] encoderAbsoluteOffsets = new double[] {177.7, 185.8, 330.1, 111.8};
    public static final double gearRatio = 150.0/7.0;

    public static final double[] throttlePresets = new double[] { 0.05, 0.3, 1.0 }; //0.41 (1/(rt2+1)) is the highest percentage that never goes above 1.0 with swerve
    public static int defaultThrottlePreset = 0;
    public static final double joystickDeadzoneXMin = -0.01;
    public static final double joystickDeadzoneXMax = 0.01;
    public static final double joystickDeadzoneYMin = -0.2;
    public static final double joystickDeadzoneYMax = 0.01;
    
    public static final double targetAngleMinMagnitude = 0.5;
    public static final double rotationZeroThreshold = 1;
    public static final double rotationMaxThreshold = 5;

    public static final Vector2D[] rotationVectors = new Vector2D[] {
        new Vector2D(Math.sqrt(2), Math.sqrt(2)),
        new Vector2D(Math.sqrt(2), -Math.sqrt(2)),
        new Vector2D(-Math.sqrt(2), Math.sqrt(2)),
        new Vector2D(-Math.sqrt(2), -Math.sqrt(2))
    };

    public static final int[] joystickNumbers = new int[] {1, 2, 3, 0};
}