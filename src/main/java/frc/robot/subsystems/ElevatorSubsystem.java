package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
    private final WPI_TalonFX elevatorMotor;

    public ElevatorSubsystem(int motorID) {
        elevatorMotor = new WPI_TalonFX(motorID);
    }

    public void setVoltage(double volts) {
        elevatorMotor.setVoltage(volts);
    }

    @Override
    public void periodic() {}

    @Override
    public void simulationPeriodic() {}
}
