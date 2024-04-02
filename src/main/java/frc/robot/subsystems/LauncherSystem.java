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
    public LauncherStatus status;

    private double timestamp;

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

    public void update() {
        if (status == LauncherStatus.Intake && !limitSwitch.get()) {
            loaded();
        }
        else if (status == LauncherStatus.Firing && Utils.getTime() - timestamp >= Constants.launcherFireTime) {
            setIdle();
        }
    }

    public void intake() {
        if (status == LauncherStatus.Idle || status == LauncherStatus.Loaded) {
            setLauncherMotors(-Constants.launcherMotorIntakeFactor);
            setFeedMotors(-Constants.feedMotorIntakeFactor);
    
            status = LauncherStatus.Intake;
        }
    }

    public void loaded() {
        if (status == LauncherStatus.Intake) {
            overrideLoaded();
        }
    }

    public void overrideLoaded() {
        setLauncherMotors(0);
        setFeedMotors(0);

        status = LauncherStatus.Loaded;
    }

    public void ready(double power) {
        if (status == LauncherStatus.Loaded) { //add auto override while idle?
            setLauncherMotors(power);
            setFeedMotors(0);

            status = LauncherStatus.Ready;
        }
    }

    public void fire() {
        if (status == LauncherStatus.Ready) {
            setFeedMotors(Constants.feedMotorOuttakeFactor);

            timestamp = Utils.getTime();
            
            status = LauncherStatus.Firing;
        }
    }

    public void setIdle() {
        setIdle(0);
    }

    public void setIdle(double overridePower) {
        setLauncherMotors(overridePower * Constants.overrideLauncherMotorFactor);
        setFeedMotors(overridePower * Constants.overrideFeedMotorFactor);
        
        if (!limitSwitch.get()) {
            status = LauncherStatus.Loaded;
        }
        else {
            status = LauncherStatus.Idle;
        }
    }

    private void setLauncherMotors(double power) {
        upperLauncherMotor.set(ControlMode.PercentOutput, power);
        lowerLauncherMotor.set(ControlMode.PercentOutput, power);
    }

    private void setFeedMotors(double power) {
        leftFeedMotor.set(ControlMode.PercentOutput, power);
        rightFeedMotor.set(ControlMode.PercentOutput, power);
    }
}
