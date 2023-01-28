package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

public class SwerveMotorsSubsystem extends SubsystemBase {
    private final WPI_TalonFX wheelMotor;
    private final WPI_TalonFX directionMotor;
    private CANCoder encoder;

    public SwerveMotorsSubsystem(int wheelMotorPort, int directionMotorPort, int encoderID) {
        wheelMotor = new WPI_TalonFX(wheelMotorPort);
        directionMotor = new WPI_TalonFX(directionMotorPort);
        encoder = new CANCoder(encoderID);
    }

    public SwerveMotorsSubsystem(int wheelMotorPort, int directionMotorPort) {
        wheelMotor = new WPI_TalonFX(wheelMotorPort);
        directionMotor = new WPI_TalonFX(directionMotorPort);
    }

    public void setRotationVoltage(double volts) {
        double pos = getRotationPosition();
        if (pos > ENCODER_UNITS_PER_ROTATION / 2 - OVER_ROTATION_PADDING && volts > 0) {
            directionMotor.stopMotor();
        } else if (pos < -ENCODER_UNITS_PER_ROTATION / 2 + OVER_ROTATION_PADDING && volts < 0) {
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
        if (encoder != null) {
            encoder.setPosition(0);
        }
    }

    public double getRotationPosition() {
        if (encoder == null) return -Double.NEGATIVE_INFINITY;
        return encoder.getPosition();
    }

    public boolean rotateTowards(double target) {
        double pos = getRotationPosition();
        double dist = target - pos;
        double percentage = clamp(dist / ENCODER_UNITS_PER_ROTATION * 2, -0.3, 0.3);

        directionMotor.set(ControlMode.MotionMagic, target, DemandType.ArbitraryFeedForward, percentage);

        double errorDegrees = Math.abs(getRotationPosition() - target);
        return errorDegrees <= PRECISION_ROTATION_PADDING;
    }
    
    @Override
    public void periodic() {}

    @Override
    public void simulationPeriodic() {}
}
