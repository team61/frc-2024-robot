package frc.robot.subsystems;

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
    private boolean enabled;

    public DriveModule(int driveMotorNumber, int angleMotorNumber, int angleEncoderNumber, DriveModulePosition position) {
        driveMotor = new TalonFX(driveMotorNumber);
        angleMotor = new TalonFX(angleMotorNumber);
        angleEncoder = new CANCoder(angleEncoderNumber);
        this.position = position;
        
        disable();
    }

    public void enable() {
        driveMotor.setNeutralMode(NeutralMode.Brake);
        angleMotor.setNeutralMode(NeutralMode.Brake);

        enabled = true;
    }

    public void disable() {
        driveMotor.set(ControlMode.PercentOutput, 0);
        angleMotor.set(ControlMode.PercentOutput, 0);
        
        driveMotor.setNeutralMode(NeutralMode.Coast);
        angleMotor.setNeutralMode(NeutralMode.Coast);
        
        enabled = false;
    }

    public void callibrateAngle() {
        double angle = angleEncoder.getAbsolutePosition() - Constants.encoderAbsoluteOffsets[position.i];

        angleEncoder.setPosition(angle);
        angleMotor.setSelectedSensorPosition(Conversions.degreesToFalcon(angle, Constants.gearRatio));
    }

    public void config() {
        angleEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        angleMotor.configIntegratedSensorAbsoluteRange(AbsoluteSensorRange.Unsigned_0_to_360);

        driveMotor.setInverted(Constants.driveMotorInverted[position.i]);
        angleMotor.setInverted(Constants.angleMotorInverted[position.i]);
    }

    public void setPower(double power) {
        driveMotor.set(ControlMode.PercentOutput, power);
    }

    public void setAngle(double angle) {
        angleMotor.set(ControlMode.Position, Conversions.degreesToFalcon(angle, Constants.gearRatio));
    }

    public void setToVector(Vector2D vector) {
        setPower(vector.magnitude());
        setAngle(vector.theta());
    }

    public void setMovement(Vector2D translationVector, double rotationPower) {
        Vector2D rotationVector = Vector2D.scalarMultiply(Constants.rotationVectors[position.i], rotationPower);
        Vector2D vector = Vector2D.add(translationVector, rotationVector);

        setToVector(vector);
    }

    public double getAngle() {
        return angleEncoder.getPosition();
    }
}
