package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

public class SwerveDriveSubsystem extends SubsystemBase {
    private final SwerveMotorsSubsystem[] swerveMotors;

    public SwerveDriveSubsystem(int totalSwerveDriveUnits, int[] ports) {
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

    public void zeroOutRotation() {
        for (SwerveMotorsSubsystem swerveUnit : swerveMotors) {
            swerveUnit.zeroOutRotation();
        }
    }

    public boolean alignMotors(String direction) {
        boolean[] results = new boolean[swerveMotors.length];
        double targetPosition = 0;
        if (direction == SIDEWAYS) {
            targetPosition += SWERVE_UNITS_PER_ROTATION / 4;
        } else if (direction == MIDDLE) {
            for (SwerveMotorsSubsystem swerveUnit : swerveMotors) {
                targetPosition += swerveUnit.getRotationPosition();
            }
            targetPosition /= swerveMotors.length;
        }
        for (int i = 0; i < swerveMotors.length; i++) {
            SwerveMotorsSubsystem swerveUnit = swerveMotors[i];
            results[i] = swerveUnit.rotateTowards(targetPosition);
        }

        for (boolean result : results) {
            if (!result) return false;
        }

        return true;
    }

    @Override
    public void periodic() {}

    @Override
    public void simulationPeriodic() {}
}
