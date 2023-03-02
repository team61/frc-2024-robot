package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

public class ElevatorSubsystem extends SubsystemBase {
    private final WPI_TalonFX elevatorMotor;

    public ElevatorSubsystem(int motorID) {
        elevatorMotor = new WPI_TalonFX(motorID);

        elevatorMotor.setNeutralMode(NeutralMode.Brake);
        elevatorMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        elevatorMotor.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
    }

    public void zero() {
        elevatorMotor.setSelectedSensorPosition(0);
    }

    public double getPosition() {
        return elevatorMotor.getSelectedSensorPosition();
    }

    public void setVoltage(ArmSubsystem arm, double volts) {
        double pos = getPosition();
        
        if (pos >= ELEVATOR_MAX_UNEXTENDED_POSITION && volts > 0) {
            if (arm.getPosition() > ARM_EXTENSION_THRESHOLD) {
                volts = 0;
            } else if (pos >= ELEVATOR_MAX_POSITION) {
                volts = 0;
            }
        }
        
        if (pos <= ELEVATOR_MIN_POSITION && volts < 0) {
            volts = 0;
        }
        
        elevatorMotor.setVoltage(volts);
    }

    public void setVoltageUnsafe(double volts) {
        elevatorMotor.setVoltage(volts);
    }

    @Override
    public void periodic() {}
}
