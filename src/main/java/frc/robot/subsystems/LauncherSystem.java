package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.Utils;
import frc.robot.enums.LauncherStatus;

public class LauncherSystem {
    private static LauncherSystem system;

    public TalonFX upperLauncherMotor, lowerLauncherMotor;
    public TalonFX leftFeedMotor, rightFeedMotor;
    public DigitalInput limitSwitch;

    private LauncherStatus status;
    private double fireTime;
    private boolean bufferedIntake = false;
    private double bufferedPower;

    private LauncherSystem() {
        upperLauncherMotor = new TalonFX(Constants.upperLauncherMotorNumber);
        upperLauncherMotor.setNeutralMode(NeutralMode.Coast);
        upperLauncherMotor.setInverted(Constants.upperLauncherMotorInverted);

        lowerLauncherMotor = new TalonFX(Constants.lowerLauncherMotorNumber);
        lowerLauncherMotor.setNeutralMode(NeutralMode.Coast);
        lowerLauncherMotor.setInverted(Constants.lowerLauncherMotorInverted);

        leftFeedMotor = new TalonFX(Constants.leftFeedMotorNumber);
        leftFeedMotor.setNeutralMode(NeutralMode.Brake);
        leftFeedMotor.setInverted(Constants.leftFeedMotorInverted);

        rightFeedMotor = new TalonFX(Constants.rightFeedMotorNumber);
        rightFeedMotor.setNeutralMode(NeutralMode.Brake);
        rightFeedMotor.setInverted(Constants.rightFeedMotorInverted);

        limitSwitch = new DigitalInput(Constants.launcherLimitSwitchNumber);

        status = LauncherStatus.Idle;
    }

    public static LauncherSystem get() {
        if (system == null) {
            system = new LauncherSystem();
        }

        return system;
    }

    public void intake() {
        if (status == LauncherStatus.Idle) {
            upperLauncherMotor.set(ControlMode.PercentOutput, -Constants.launcherMotorIntakeFactor);
            lowerLauncherMotor.set(ControlMode.PercentOutput, -Constants.launcherMotorIntakeFactor);
    
            leftFeedMotor.set(ControlMode.PercentOutput, -Constants.feedMotorIntakeFactor);
            rightFeedMotor.set(ControlMode.PercentOutput, -Constants.feedMotorIntakeFactor);
    
            status = LauncherStatus.Intake;
        }
        else if (status == LauncherStatus.Firing) {
            bufferedIntake = true;
        }
    }

    public void update() {
        if (status == LauncherStatus.Intake && !limitSwitch.get()) {
            upperLauncherMotor.set(ControlMode.PercentOutput, 0);
            lowerLauncherMotor.set(ControlMode.PercentOutput, 0);
        
            leftFeedMotor.set(ControlMode.PercentOutput, 0);
            rightFeedMotor.set(ControlMode.PercentOutput, 0);

            status = LauncherStatus.Loaded;

            if (bufferedPower != 0) {
                ready(bufferedPower);
                bufferedPower = 0;
            }
        }
        else
        if (status == LauncherStatus.Firing && Utils.getTime() - fireTime >= Constants.launcherFireTime) {
            upperLauncherMotor.set(ControlMode.PercentOutput, 0);
            lowerLauncherMotor.set(ControlMode.PercentOutput, 0);
            
            leftFeedMotor.set(ControlMode.PercentOutput, 0);
            rightFeedMotor.set(ControlMode.PercentOutput, 0);
            
            status = LauncherStatus.Idle;

            if (bufferedIntake) {
                intake();
                bufferedIntake = false;
            }
        }
    }

    public void ready(double power) {
        if (status == LauncherStatus.Loaded || status == LauncherStatus.Ready) {
            upperLauncherMotor.set(ControlMode.PercentOutput, power);
            lowerLauncherMotor.set(ControlMode.PercentOutput, power);

            status = LauncherStatus.Ready;
        }
        else if (status == status.Intake) {
            bufferedPower = power;
        }
    }

    public void fire() {
        if (status == LauncherStatus.Ready) {
            leftFeedMotor.set(ControlMode.PercentOutput, Constants.feedMotorOuttakeFactor);
            rightFeedMotor.set(ControlMode.PercentOutput, Constants.feedMotorOuttakeFactor);

            fireTime = Utils.getTime();
            
            status = LauncherStatus.Firing;
        }
    }

    public void cancel() {
        if (status == LauncherStatus.Ready) {
            upperLauncherMotor.set(ControlMode.PercentOutput, 0);
            lowerLauncherMotor.set(ControlMode.PercentOutput, 0);

            status = LauncherStatus.Loaded;
        }
        else if (status == LauncherStatus.Intake) {
            upperLauncherMotor.set(ControlMode.PercentOutput, 0);
            lowerLauncherMotor.set(ControlMode.PercentOutput, 0);
            
            leftFeedMotor.set(ControlMode.PercentOutput, 0);
            rightFeedMotor.set(ControlMode.PercentOutput, 0);

            status = LauncherStatus.Loaded;

            if (bufferedPower != 0) {
                ready(bufferedPower);
                bufferedPower = 0;
            }
        }
    }
}
