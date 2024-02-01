package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import frc.robot.Constants;

public class ArmSystem {
    private static ArmSystem system;

    private TalonFX armMotor, handMotor, leftElevatorMotor, rightElevatorMotor;

    private ArmSystem() {
        armMotor = new TalonFX(Constants.armMotorNumber);
        armMotor.setNeutralMode(NeutralMode.Brake);
        armMotor.setInverted(Constants.armMotorInverted);
        
        handMotor = new TalonFX(Constants.handMotorNumber);
        handMotor.setNeutralMode(NeutralMode.Coast);
        handMotor.setInverted(Constants.handMotorInverted);

        leftElevatorMotor = new TalonFX(Constants.leftElevatorMotorNumber);
        leftElevatorMotor.setNeutralMode(NeutralMode.Brake);
        leftElevatorMotor.setInverted(Constants.leftElevatorMotorInverted);

        rightElevatorMotor = new TalonFX(Constants.rightElevatorMotorNumber);
        rightElevatorMotor.setNeutralMode(NeutralMode.Brake);
        rightElevatorMotor.setInverted(Constants.rightElevatorMotorInverted);
    }

    public static ArmSystem get() {
        if (system == null) {
            system = new ArmSystem();
        }

        return system;
    }

    public void pickup() {
        handMotor.set(ControlMode.PercentOutput, Constants.handMotorFactor);
    }

    public void release() {
        handMotor.set(ControlMode.PercentOutput, -Constants.handMotorFactor);
    }

    public void stopHand() {
        handMotor.set(ControlMode.PercentOutput, 0);
    }

    public void setArmPower(double power) {
        armMotor.set(ControlMode.PercentOutput, power * Constants.armMotorFactor);
    }

    public void setElevatorPower(double power) {
        leftElevatorMotor.set(ControlMode.PercentOutput, power * Constants.elevatorFactor);
        rightElevatorMotor.set(ControlMode.PercentOutput, power * Constants.elevatorFactor);
    }
}
