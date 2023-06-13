package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TankDriveSubsystem extends SubsystemBase {
    private final OldSwerveMotorsSubsystem[] driveLeft;
    private final OldSwerveMotorsSubsystem[] driveRight;

    public TankDriveSubsystem(int unitsPerSide, int[] leftIDs, int[] rightIDs) {
        driveLeft = new OldSwerveMotorsSubsystem[unitsPerSide];
        driveRight = new OldSwerveMotorsSubsystem[unitsPerSide];

        for (int i = 0; i < driveLeft.length; i++) {
            driveLeft[i] = new OldSwerveMotorsSubsystem(leftIDs[i * 2], leftIDs[i * 2 + 1]);
        }

        for (int i = 0; i < driveRight.length; i++) {
            driveRight[i] = new OldSwerveMotorsSubsystem(rightIDs[i * 2], rightIDs[i * 2 + 1]);
        }
    }

    public void driveVolts(double lVolts, double rVolts) {
        for (OldSwerveMotorsSubsystem swerveUnit : driveLeft) {
            swerveUnit.setWheelVoltage(lVolts);
        }

        for (OldSwerveMotorsSubsystem swerveUnit : driveRight) {
            swerveUnit.setWheelVoltage(rVolts);
        }
    }

    public double getAverageVoltage() {
        double totalVoltage = 0;
        double totalMotors = 0;

        for (OldSwerveMotorsSubsystem motor : driveLeft) {
            totalVoltage += motor.getWheelVoltage();
            totalMotors++;
        }
        for (OldSwerveMotorsSubsystem motor : driveRight) {
            totalVoltage += motor.getWheelVoltage();
            totalMotors++;
        }

        return totalVoltage / totalMotors;
    }

    public void driveSpeed(double lSpeed, double rSpeed) {
        for (OldSwerveMotorsSubsystem swerveUnit : driveLeft) {
            swerveUnit.setWheelSpeed(lSpeed);
        }

        for (OldSwerveMotorsSubsystem swerveUnit : driveRight) {
            swerveUnit.setWheelSpeed(rSpeed);
        }
    }

    public void stop() {
        for (OldSwerveMotorsSubsystem swerveUnit : driveLeft) {
            swerveUnit.stopWheel();
        }

        for (OldSwerveMotorsSubsystem swerveUnit : driveRight) {
            swerveUnit.stopWheel();
        }
    }

    @Override
    public void periodic() {}
}
