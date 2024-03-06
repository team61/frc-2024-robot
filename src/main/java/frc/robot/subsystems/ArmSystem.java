package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.Utils;
import frc.robot.LEDStrategies.LEDStrategy;
import frc.robot.enums.LimitedMotorCalibrationStatus;
import frc.robot.subsystemHelpers.LimitedMotorWrapper;
import frc.robot.subsystemHelpers.MotorWrapper;
import lib.math.Conversions;

public class ArmSystem {
    private static ArmSystem system;

    private LEDSystem ledSystem = LEDSystem.get();

    public DigitalInput armSwitch, elevatorSwitch;

    public LimitedMotorWrapper elevatorMotor;
    public LimitedMotorWrapper armMotor;
    public MotorWrapper handMotor;
    //public MotorWrapper balancerMotor;

    private boolean stopped = false;

    private ArmSystem() {
        // armMotor = new TalonFX(Constants.armMotorNumber);
        // armMotor.setNeutralMode(NeutralMode.Brake);
        // armMotor.setInverted(Constants.armMotorInverted);
        
        // handMotor = new TalonFX(Constants.handMotorNumber);
        // handMotor.setNeutralMode(NeutralMode.Coast);
        // handMotor.setInverted(Constants.handMotorInverted);

        handMotor = new MotorWrapper(new int[] {Constants.handMotorNumber},
            new boolean[] {Constants.handMotorInverted});
        handMotor.factor = Constants.handMotorFactor;
        handMotor.lerpFactor = Constants.handLerpFactor;
        handMotor.minPower = 0.3;

        armMotor = new LimitedMotorWrapper(new int[] {Constants.armMotorNumber},
            new boolean[] {Constants.armMotorInverted},
            Constants.armLimitSwitchNumber);
        armMotor.factor = Constants.armMotorFactor;
        armMotor.lerpFactor = Constants.armLerpFactor;
        armMotor.gearRatio = Constants.armGearRatio;
        armMotor.zeroThreshold = Constants.armZeroThreshold;
        armMotor.maxThreshold = Constants.armMaxThreshold;
        armMotor.limitPosition = 103.5;
        armMotor.stayBelowLimit = true;
        armMotor.status = LimitedMotorCalibrationStatus.NotCalibrated;
        armMotor.calibrationTriggerPower = 0.2;
        armMotor.calibrationUntriggerPower = 0.1;

        elevatorMotor = new LimitedMotorWrapper(new int[] { Constants.leftElevatorMotorNumber, Constants.rightElevatorMotorNumber },
            new boolean[] { Constants.leftElevatorMotorInverted, Constants.rightElevatorMotorInverted },
            Constants.elevatorLimitSwitchNumber);
        elevatorMotor.factor = Constants.elevatorFactor;
        elevatorMotor.lerpFactor = Constants.elevatorLerpFactor;
        elevatorMotor.gearRatio = Constants.elevatorGearRatio;
        elevatorMotor.zeroThreshold = Constants.elevatorZeroThreshold;
        elevatorMotor.maxThreshold = Constants.elevatorMaxThreshold;
        elevatorMotor.limitPosition = 0;
        elevatorMotor.stayBelowLimit = false;
        elevatorMotor.status = LimitedMotorCalibrationStatus.NotCalibrated;
        elevatorMotor.calibrationTriggerPower = 0.2;
        elevatorMotor.calibrationUntriggerPower = 0.1;

        // balancerMotor = new MotorWrapper(new int[] {Constants.balancerMotorNumber},
        //     new boolean[] {Constants.balancerMotorInverted});
        // balancerMotor.factor = Constants.balancerMotorFactor;
        // balancerMotor.lerpFactor = Constants.balancerLerpFactor;
    }

    public static ArmSystem get() {
        if (system == null) {
            system = new ArmSystem();
        }

        return system;
    }

    public void pickup() {
        handMotor.targetPower = 1;

        if (stopped) {
            ledSystem.strategies = Constants.pickupStrategies;
            stopped = false;
        }
    }

    public void release() {
        handMotor.targetPower = -1;
    }

    public void stopHand() {
        handMotor.targetPower = 0;

        if (!stopped) {
            ledSystem.strategies = Constants.defaultStrategies;
            stopped = true;
        }
    }

    // public void engageBalancer() {
    //     balancerMotor.targetPower = 1;
    // }

    // public void disengageBalancer() {
    //     balancerMotor.targetPower = -1;
    // }

    // public void stopBalancer() {
    //     balancerMotor.targetPower = 0;
    // }

    public void setArmPowerLinear(double power) {
        armMotor.targetPower = power;
    }

    // public void setArmTarget(double angle) {
    //     armTarget = angle;
    // }

    // public void seekArmTarget() {
    //     double angleOffset = Math.abs(armTarget - getArmAngle());
    //     double newArmPower = (angleOffset - Constants.armZeroThreshold) / (Constants.armMaxThreshold - Constants.armZeroThreshold);
    //     newArmPower = Math.min(Math.max(newArmPower, 0), 1);
    //     if (armTarget - getArmAngle() < 0) {
    //         newArmPower *= -1;
    //     }

    //     armPower = Utils.lerp(armPower, newArmPower, Constants.armLerpFactor);
    // }

    public void setElevatorPower(double power) {
        elevatorMotor.targetPower = power;
    }

    // public double getArmAngle() {
    //     return armMotor.getPosition();
    // }

    public void update() {
        elevatorMotor.update();
        armMotor.update();
        handMotor.update();
        //balancerMotor.update();
    }

    public void calibrateArmAngle() {
        armMotor.motors[0].setSelectedSensorPosition(0);
    }
}
