package frc.robot.enums;

public enum LauncherStatus {
    Idle(0),
    Intake(1),
    Loaded(2),
    Ready(3),
    Firing(4);

    public final int i;

    LauncherStatus(int i) {
        this.i = i;
    }
}