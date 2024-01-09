package frc.robot.enums;

public enum DriveModulePosition {
    FrontLeft(0),
    FrontRight(1),
    BackLeft(2),
    BackRight(3);

    public final int i;

    DriveModulePosition(int i) {
        this.i = i;
    }
}