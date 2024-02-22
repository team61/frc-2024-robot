package frc.robot.enums;

public enum AutonMode {
    Left(0),
    Center(1),
    Right(2);

    public final int i;

    AutonMode(int i) {
        this.i = i;
    }
}