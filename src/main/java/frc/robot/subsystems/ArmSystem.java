package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.Utils;
import frc.robot.enums.LimitedMotorCalibrationStatus;
import frc.robot.subsystemHelpers.ArmPreset;
import frc.robot.subsystemHelpers.LimitedMotorWrapper;
import frc.robot.subsystemHelpers.MotorWrapper;
import lib.math.Conversions;

public class ArmSystem {
    private static ArmSystem system;

    public DigitalInput armSwitch, elevatorSwitch;

    public LimitedMotorWrapper elevatorMotor;
    public LimitedMotorWrapper armMotor;
    public MotorWrapper handMotor;

    private boolean stopped = false;
    private Double overridenArmTarget = null;

    private ArmSystem() {
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
            stopped = false;
        }
    }

    public void release() {
        handMotor.targetPower = -1;
    }

    public void stopHand() {
        handMotor.targetPower = 0;

        if (!stopped) {
            stopped = true;
        }
    }

    public void setArmPowerLinear(double power) {
        armMotor.targetPower = power;
    }

    public void setElevatorPower(double power) {
        elevatorMotor.targetPower = power;
    }

    public void setElevatorTarget(Double target) {
        elevatorMotor.targetPosition = target;
    }

    public void setArmTarget(Double target) {
        if (overridenArmTarget != null) {
            overridenArmTarget = target;
        }
        else {
            armMotor.targetPosition = target;
        }
    }

    public void overrideArmTarget(Double target) {
        if (target == null) {
            if (overridenArmTarget != null) {
                setArmTarget(overridenArmTarget);
                overridenArmTarget = null;
            }
        }
        else {
            if (overridenArmTarget == null) {
                overridenArmTarget = armMotor.targetPosition;
            }

            setArmTarget(target);
        }
    }

    public void setPreset(ArmPreset preset) {
        if (preset == null) {
            setElevatorTarget(null);
            setArmTarget(null);
        }
        else {
            setElevatorTarget(preset.elevatorTarget);
            setArmTarget(preset.armTarget);
        }
    }

    public void update() {
        Double armPosition = armMotor.targetPosition;

        if (overridenArmTarget != null) {
            armPosition = overridenArmTarget;
        }
        else if (armPosition == null) {
            armPosition = armMotor.getPosition() + armMotor.targetPower * 5;
        }
        
        if (elevatorMotor.getPosition() + Constants.armLength * Math.sin(armPosition * Math.PI / 180) > Constants.maxElevatorHeight) {
            overrideArmTarget(Math.asin((Constants.maxElevatorHeight - elevatorMotor.getPosition()) / Constants.armLength) * 180 / Math.PI);
            System.out.println(Math.asin((Constants.maxElevatorHeight - elevatorMotor.getPosition()) / Constants.armLength) * 180 / Math.PI);
        }
        else {
            overrideArmTarget(null);
        }
        
        elevatorMotor.update();
        armMotor.update();
        handMotor.update();
        //balancerMotor.update();

        //System.out.println("e: " + elevatorMotor.getPosition() + ", a: " + armMotor.getPosition() + ", h: " + elevatorMotor.getPosition() + Constants.armLength * Math.cos(armMotor.getPosition() * Math.PI / 180));
    }

    public void calibrateArmAngle() {
        armMotor.motors[0].setSelectedSensorPosition(0);
    }
}
