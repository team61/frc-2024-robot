package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

public class ArmSubsystem extends SubsystemBase {
    public final WPI_TalonFX armMotor;
    public final DigitalInput limitSwitch;

    public ArmSubsystem(int motorID, int limitSwitchPort) {
        armMotor = new WPI_TalonFX(motorID);
        limitSwitch = new DigitalInput(limitSwitchPort);

        armMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        armMotor.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        armMotor.setNeutralMode(NeutralMode.Brake);
    }

    public void zero() {
        armMotor.setSelectedSensorPosition(0);
    }

    public double getPosition() {
        return armMotor.getSelectedSensorPosition();
    }

    public boolean isFullyRetracted() {
        return getPosition() >= ARM_MAX_POSITION || !limitSwitch.get();
    }

    public boolean isFullyRetracted(double pos) {
        return pos >= ARM_MAX_POSITION || !limitSwitch.get();
    }

    public boolean isFullyExtended() {
        return getPosition() <= ARM_MIN_POSITION;
    }

    public boolean isFullyExtended(double pos) {
        return pos <= ARM_MIN_POSITION;
    }

    public boolean shouldPlaceTopBlock() {
        return getPosition() <= ARM_PLACE_TOP_PIECE_POSITION;
    }

    public void setVoltage(ElevatorSubsystem elevator, double volts) {
        double pos = getPosition();
        
        if (isFullyRetracted(pos) && volts > 0) {
            volts = 0;
        }
        
        if (isFullyExtended(pos) && volts < 0) {
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

    public void stop() {
        setVoltageUnsafe(0);
    }
}
