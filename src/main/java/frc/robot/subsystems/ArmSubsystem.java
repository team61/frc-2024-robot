package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

public class ArmSubsystem extends SubsystemBase {
    private final WPI_TalonFX armMotor;
    public final DigitalInput limitSwitch;

    public ArmSubsystem(int motorID, int limitSwitchPort) {
        armMotor = new WPI_TalonFX(motorID);
        limitSwitch = new DigitalInput(limitSwitchPort);

        armMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
        armMotor.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
    }

    public void zero() {
        armMotor.setSelectedSensorPosition(0);
    }

    public double getPosition() {
        return armMotor.getSelectedSensorPosition();
    }

    public void setVoltage(ElevatorSubsystem elevator, double volts) {
        double pos = getPosition();
        
        if ((pos >= ARM_MAX_POSITION || !limitSwitch.get()) && volts > 0) {
            volts = 0;
        }
        
        if (pos <= ARM_MIN_POSITION && volts < 0) {
            volts = 0;
        }

        if (elevator.getPosition() > ELEVATOR_MAX_UNEXTENDED_POSITION && pos > ARM_MIN_LOWER_EXTENSION && volts > 0) {
            volts = 0;
        }
        
        armMotor.setVoltage(volts);
    }

    public void setVoltageUnsafe(double volts) {
        if (!limitSwitch.get() && volts > 0) {
            volts = 0;
        }
        armMotor.setVoltage(volts);
    }

    @Override
    public void periodic() {}
}
