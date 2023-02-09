package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
    private final WPI_TalonFX armMotor;

    public ArmSubsystem(int motorID) {
        armMotor = new WPI_TalonFX(motorID);
    }

    public void setVoltage(double volts) {
        armMotor.setVoltage(volts);
    }

    @Override
    public void periodic() {}
}
