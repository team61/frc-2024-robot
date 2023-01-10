package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrainSubsystem extends SubsystemBase {
    private final SwerveMotorsSubsystem[] swerveMotors;

    public DriveTrainSubsystem(int totalSwerveDriveUnits, int[] ports) {
        swerveMotors = new SwerveMotorsSubsystem[totalSwerveDriveUnits];

        for (int i = 0; i < totalSwerveDriveUnits; i++) {
            swerveMotors[i] = new SwerveMotorsSubsystem(ports[i * 2], ports[i * 2 + 1]);
        }
    }

    public void setRotationVoltage(double volts) {
        for (SwerveMotorsSubsystem swerveUnit : swerveMotors) {
            swerveUnit.setRotationVoltage(volts);
        }
    }

    public void driveVolts(double volts) {
        for (SwerveMotorsSubsystem swerveUnit : swerveMotors) {
            swerveUnit.setWheelVoltage(volts);
        }
    }

    public void driveSpeed(double speed) {
        for (SwerveMotorsSubsystem swerveUnit : swerveMotors) {
            swerveUnit.setWheelSpeed(speed);
        }
    }

    public void enableBreaks() {
        for (SwerveMotorsSubsystem swerveUnit : swerveMotors) {
            swerveUnit.enableBreaks();
        }
    }

    public void disableBreaks() {
        for (SwerveMotorsSubsystem swerveUnit : swerveMotors) {
            swerveUnit.disableBreaks();
        }
    }

    public void alignMotors() {
        double targetPosition = 0;
        double maxPosition = Double.NEGATIVE_INFINITY;
        double minPosition = Double.POSITIVE_INFINITY;
        for (SwerveMotorsSubsystem swerveUnit : swerveMotors) {
            double pos = swerveUnit.getRotationPosition();
            targetPosition += pos;
            maxPosition = Math.max(maxPosition, pos);
            minPosition = Math.min(minPosition, pos);
        }
        targetPosition /= swerveMotors.length;

        for (SwerveMotorsSubsystem swerveUnit : swerveMotors) {
            swerveUnit.rotateTowards(targetPosition, maxPosition, minPosition);
        }
    }

    @Override
    public void periodic() {}

    @Override
    public void simulationPeriodic() {}
}
