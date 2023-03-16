package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

public class OldSwerveMotorsSubsystem extends SubsystemBase {
    public final WPI_TalonFX wheelMotor;
    public final WPI_TalonFX directionMotor;
    private CANCoder encoder;

    public OldSwerveMotorsSubsystem(int wheelMotorID, int directionMotorID, int encoderID, double offset) {
        wheelMotor = new WPI_TalonFX(wheelMotorID);
        directionMotor = new WPI_TalonFX(directionMotorID);
        encoder = new CANCoder(encoderID);

        wheelMotor.setNeutralMode(NeutralMode.Coast);
        directionMotor.setNeutralMode(NeutralMode.Coast);
        encoder.configMagnetOffset(offset);
        encoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        
        double pos = getRotationPosition();
        if (Math.abs(ENCODER_UNITS_PER_ROTATION - pos) < ENCODER_UNITS_PER_ROTATION / 4) pos -= ENCODER_UNITS_PER_ROTATION;
        else if (Math.abs(-ENCODER_UNITS_PER_ROTATION - pos) < ENCODER_UNITS_PER_ROTATION / 4) pos += ENCODER_UNITS_PER_ROTATION;
        encoder.setPosition(pos);
    }

    public OldSwerveMotorsSubsystem(int wheelMotorID, int directionMotorID) {
        wheelMotor = new WPI_TalonFX(wheelMotorID);
        directionMotor = new WPI_TalonFX(directionMotorID);
    }

    public void setRotationVoltageNoRestraint(double volts) {
        directionMotor.setVoltage(volts);
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

    public double getWheelVoltage() {
        return wheelMotor.getBusVoltage();
    }

    public void setWheelSpeed(double speed) {
        wheelMotor.set(ControlMode.PercentOutput, speed);
    }
    
    public void stopWheel() {
        wheelMotor.setVoltage(0);
    }

    // public void enableBreaks() {
    //     wheelMotor.setNeutralMode(NeutralMode.Brake);
    //     directionMotor.setNeutralMode(NeutralMode.Brake);
    // }

    // public void disableBreaks() {
    //     wheelMotor.setNeutralMode(NeutralMode.Coast);
    //     directionMotor.setNeutralMode(NeutralMode.Coast);
    // }
    
    public void enableWheelBreaks() {
        wheelMotor.setNeutralMode(NeutralMode.Brake);
    }

    public void disableWheelBreaks() {
        wheelMotor.setNeutralMode(NeutralMode.Coast);
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
        double percentage = clamp(dist / ENCODER_UNITS_PER_ROTATION * 4, -0.25, 0.25);

        double errorDegrees = Math.abs(getRotationPosition() - target);
        boolean isWithinAllowedError = errorDegrees <= PRECISION_ROTATION_PADDING;

        if (!isWithinAllowedError) {
            directionMotor.set(ControlMode.MotionMagic, target, DemandType.ArbitraryFeedForward, percentage);
        }

        return isWithinAllowedError || Math.abs(percentage) < 0.02;
    }
    
    @Override
    public void periodic() {}
}
