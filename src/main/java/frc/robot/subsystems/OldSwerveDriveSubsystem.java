package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

public class OldSwerveDriveSubsystem extends SubsystemBase {
    public final OldSwerveMotorsSubsystem[] swerveMotors;

    public OldSwerveDriveSubsystem(int totalSwerveDriveUnits, int[] motorIDs, int[] encoderIDs, double[] offsets) {
        swerveMotors = new OldSwerveMotorsSubsystem[totalSwerveDriveUnits];

        for (int i = 0; i < totalSwerveDriveUnits; i++) {
            swerveMotors[i] = new OldSwerveMotorsSubsystem(motorIDs[i * 2], motorIDs[i * 2 + 1], encoderIDs[i], offsets[i]);
        }
    }

    public void rotateIndividualWheel(int index, double volts) {
        swerveMotors[index].setRotationVoltageNoRestraint(volts);
    }

    public void setRotationVoltage(double volts) {
        for (OldSwerveMotorsSubsystem swerveUnit : swerveMotors) {
            swerveUnit.setRotationVoltage(volts);
        }
    }

    public void driveVolts(double volts) {
        for (OldSwerveMotorsSubsystem swerveUnit : swerveMotors) {
            swerveUnit.setWheelVoltage(volts);
        }
    }

    public void driveSpeed(double speed) {
        for (OldSwerveMotorsSubsystem swerveUnit : swerveMotors) {
            swerveUnit.setWheelSpeed(speed);
        }
    }

    // public void enableBreaks() {
    //     for (SwerveMotorsSubsystem swerveUnit : swerveMotors) {
    //         swerveUnit.enableBreaks();
    //     }
    // }

    // public void disableBreaks() {
    //     for (SwerveMotorsSubsystem swerveUnit : swerveMotors) {
    //         swerveUnit.disableBreaks();
    //     }
    // }

    public void enableWheelBreaks() {
        for (OldSwerveMotorsSubsystem swerveUnit : swerveMotors) {
            swerveUnit.enableWheelBreaks();
        }
    }

    public void disableWheelBreaks() {
        for (OldSwerveMotorsSubsystem swerveUnit : swerveMotors) {
            swerveUnit.disableWheelBreaks();
        }
    }

    public void zeroOutRotation() {
        for (OldSwerveMotorsSubsystem swerveUnit : swerveMotors) {
            swerveUnit.zeroOutRotation();
        }
    }

    public boolean alignMotors(String[] directions) {
        boolean[] results = new boolean[swerveMotors.length];

        double averagePosition = 0;
        if (directions[0] == MIDDLE) {
            for (int i = 0; i < swerveMotors.length; i++) {
                averagePosition += swerveMotors[i].getRotationPosition();
            }
            averagePosition /= swerveMotors.length;
        }
        
        for (int i = 0; i < swerveMotors.length; i++) {
            double targetPosition = 0;
            String direction = directions[i];
            if (direction.equals(BACKWARDS)) {
                targetPosition += ENCODER_UNITS_PER_ROTATION / 2;
            } else if (direction.equals(SIDEWAYS)) {
                targetPosition += ENCODER_UNITS_PER_ROTATION / 4;
            } else if (direction.equals(DIAGONAL)) {
                if (i == 0) {
                    targetPosition += ENCODER_UNITS_PER_ROTATION / 8;
                } else if (i == 1) {
                    targetPosition += -ENCODER_UNITS_PER_ROTATION / 8;
                } else if (i == 2) {
                    targetPosition += -ENCODER_UNITS_PER_ROTATION / 4 - ENCODER_UNITS_PER_ROTATION / 8;
                } else {
                    targetPosition += ENCODER_UNITS_PER_ROTATION / 4 + ENCODER_UNITS_PER_ROTATION / 8;
                }
            } else if (direction.equals(MIDDLE)) {
                targetPosition = averagePosition;
            }
            OldSwerveMotorsSubsystem swerveUnit = swerveMotors[i];
            results[i] = swerveUnit.rotateTowards(targetPosition);
        }

        for (boolean result : results) {
            if (!result) return false;
        }

        return true;
    }

    public boolean alignMotors(String direction) {
        return alignMotors(new String[] { direction, direction, direction, direction });
    }

    @Override
    public void periodic() {
        // System.out.print("wheels: ");
        // for (SwerveMotorsSubsystem swerveUnit : swerveMotors) {
        //     System.out.print(Math.round(swerveUnit.getRotationPosition()) + ", ");
        // }
        // System.out.println();
    }
}
