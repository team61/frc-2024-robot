package frc.robot.subsystemHelpers;

import lib.math.Conversions;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;

import frc.robot.Constants;
import frc.robot.enums.DriveModulePosition;
import frc.robot.Vector2D;

public class DriveModule {
    public TalonFX driveMotor, angleMotor;
    public CANCoder angleEncoder;

    private DriveModulePosition position;
    private boolean inverted;

    public DriveModule(int driveMotorNumber, int angleMotorNumber, int angleEncoderNumber, DriveModulePosition position) {
        driveMotor = new TalonFX(driveMotorNumber);
        driveMotor.setNeutralMode(NeutralMode.Brake);

        angleMotor = new TalonFX(angleMotorNumber);
        angleMotor.setNeutralMode(NeutralMode.Brake);

        angleEncoder = new CANCoder(angleEncoderNumber);

        this.position = position;
    }

    public void calibrateAngle() {
        angleEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        angleMotor.configIntegratedSensorAbsoluteRange(AbsoluteSensorRange.Unsigned_0_to_360);

        driveMotor.setInverted(Constants.driveMotorInverted[position.i] != inverted);
        angleMotor.setInverted(Constants.angleMotorInverted[position.i]);
        
        double angle = angleEncoder.getAbsolutePosition() - Constants.encoderAbsoluteOffsets[position.i] + (inverted ? 0 : 180);
        
        angleEncoder.setPosition(angle);
        angleMotor.setSelectedSensorPosition(Conversions.degreesToFalcon(angle, Constants.gearRatio));
    }

    public void invertAngle() {
        double angle = angleEncoder.getPosition() + 180;
        
        angleEncoder.setPosition(angle);
        angleMotor.setSelectedSensorPosition(Conversions.degreesToFalcon(angle, Constants.gearRatio));

        driveMotor.setInverted(!driveMotor.getInverted());

        inverted = !inverted;
    }

    public void setPower(double power) {
        driveMotor.set(ControlMode.PercentOutput, power);
    }

    public void setAngle(double angle) {
        double adjustedAngle = angle;

        while (getCalibratedAngle() - adjustedAngle > 180) {
            adjustedAngle += 360;
        }

        while (getCalibratedAngle() - adjustedAngle < -180) {
            adjustedAngle -= 360;
        }

        //inversion

        // if (Math.abs(getCalibratedAngle() - adjustedAngle) > 90) {
        //     invertAngle();
            
        //     if (getCalibratedAngle() - adjustedAngle > 180) {
        //         adjustedAngle += 360;
        //     }
        // }
        
        angleMotor.set(ControlMode.Position, Conversions.degreesToFalcon(adjustedAngle, Constants.gearRatio));
    }

    public void setToVector(Vector2D vector) {
        double magnitude = vector.magnitude();

        if (magnitude != 0) {
            setAngle(vector.theta());
        }
        
        setPower(vector.magnitude());
    }

    public void setMovement(Vector2D translationVector, double rotationPower) {
        Vector2D rotationVector = Vector2D.scalarMultiply(Constants.rotationVectors[position.i], rotationPower);
        Vector2D vector = Vector2D.add(translationVector, rotationVector);

        setToVector(vector);
    }

    public double getAngle() {
        return angleEncoder.getPosition();
    }

    public double getCalibratedAngle() {
        return Conversions.falconToDegrees(angleMotor.getSelectedSensorPosition(), Constants.gearRatio);
    }
}
