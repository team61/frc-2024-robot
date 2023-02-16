package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

public class ElevatorSubsystem extends SubsystemBase {
    private final WPI_TalonFX elevatorMotor;
    private double startPos = -1;
    private double endPos = -1;

    public ElevatorSubsystem(int motorID) {
        elevatorMotor = new WPI_TalonFX(motorID);
        elevatorMotor.setNeutralMode(NeutralMode.Brake);
    }

    public double getPosition() {
        return elevatorMotor.getSelectedSensorPosition();
    }

    public void setVoltage(double volts) {
        double pos = getPosition();
        
        if (pos >= ELEVATOR_MAX_POSITION && volts > 0) {
            elevatorMotor.setVoltage(0);
            volts = 0;
        }
        
        if (pos <= ELEVATOR_MIN_POSITION && volts < 0) {
            elevatorMotor.setVoltage(0);
            volts = 0;
        }
        
        System.out.println(pos + ", " + volts);
        elevatorMotor.setVoltage(volts);
    }

    @Override
    public void periodic() {}
}
