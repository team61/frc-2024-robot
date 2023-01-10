package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveMotorsSubsystem extends SubsystemBase {
    private final WPI_TalonFX wheelMotor;
    private final WPI_TalonFX directionMotor;

    public SwerveMotorsSubsystem(int wheelMotorPort, int directionMotorPort) {
        wheelMotor = new WPI_TalonFX(wheelMotorPort);
        directionMotor = new WPI_TalonFX(directionMotorPort);
    }

    public void setRotationVoltage(double volts) {
        directionMotor.setVoltage(volts);
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

    public double getRotationPosition() {
        return directionMotor.getSelectedSensorPosition();
    }

    public void zeroOutRotation() {
        directionMotor.getSensorCollection().setIntegratedSensorPosition(0, 0);
    }

    public void rotateTowards(double target, double max, double min) {
        double pos = getRotationPosition();
        double dist = target - pos;
        double percentage = Math.max(Math.min(dist / 2500, 0.7), 0);

        directionMotor.set(ControlMode.PercentOutput, percentage);
    }
    
    @Override
    public void periodic() {}

    @Override
    public void simulationPeriodic() {}
}
