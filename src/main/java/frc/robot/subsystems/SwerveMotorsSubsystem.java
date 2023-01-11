package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

public class SwerveMotorsSubsystem extends SubsystemBase {
    private final WPI_TalonFX wheelMotor;
    private final WPI_TalonFX directionMotor;

    public SwerveMotorsSubsystem(int wheelMotorPort, int directionMotorPort) {
        wheelMotor = new WPI_TalonFX(wheelMotorPort);
        directionMotor = new WPI_TalonFX(directionMotorPort);
    }

    public void setRotationVoltage(double volts) {
        double pos = getRotationPosition();
        if (pos > SWERVE_UNITS_PER_ROTATION / 2 - SWERVE_UNITS_PADDING && volts > 0) {
            directionMotor.stopMotor();
        } else if (pos < -SWERVE_UNITS_PER_ROTATION / 2 + SWERVE_UNITS_PADDING && volts < 0) {
            directionMotor.stopMotor();
        } else {
            directionMotor.setVoltage(volts);
        }
    }

    public void stopRotation() {
        directionMotor.setVoltage(0);
    }

    public void setWheelVoltage(double volts) {
        wheelMotor.setVoltage(volts);
    }

    public void setWheelSpeed(double speed) {
        wheelMotor.set(ControlMode.PercentOutput, speed);
    }
    
    public void stopWheel() {
        wheelMotor.setVoltage(0);
    }

    public void enableBreaks() {
        wheelMotor.setNeutralMode(NeutralMode.Brake);
        directionMotor.setNeutralMode(NeutralMode.Brake);
    }

    public void disableBreaks() {
        wheelMotor.setNeutralMode(NeutralMode.Coast);
        directionMotor.setNeutralMode(NeutralMode.Coast);
    }
    
    public void zeroOutRotation() {
        directionMotor.getSensorCollection().setIntegratedSensorPosition(0, 0);
    }

    public double getRotationPosition() {
        return directionMotor.getSelectedSensorPosition();
    }

    public boolean rotateTowards(double target) {
        double pos = getRotationPosition();
        double dist = target - pos;
        double percentage = clamp(dist / SWERVE_UNITS_PER_ROTATION * 3, -0.2, 0.2);

        directionMotor.set(ControlMode.MotionMagic, target, DemandType.ArbitraryFeedForward, percentage);
        return clamp(percentage, -0.035, 0.035) == percentage;
    }
    
    @Override
    public void periodic() {}

    @Override
    public void simulationPeriodic() {}
}
