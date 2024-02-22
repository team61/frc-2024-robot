// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystemHelpers;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.enums.LimitedMotorCalibrationStatus;

/** Add your docs here. */
public class LimitedMotorWrapper extends TargettedMotorWrapper {
    public DigitalInput limitSwitch;
    public double limitPosition;
    public boolean stayBelowLimit;
    public LimitedMotorCalibrationStatus status = LimitedMotorCalibrationStatus.Calibrated;
    public double calibrationTriggerPower = 1;
    public double calibrationUntriggerPower = 1;

    public LimitedMotorWrapper(int[] motorNumbers, boolean[] inverted, int limitSwitchNumber) {
        super(motorNumbers, inverted);

        limitSwitch = new DigitalInput(limitSwitchNumber);
    }

    @Override
    public void update() {
        if (!limitSwitch.get()) {
            if (status == LimitedMotorCalibrationStatus.NotCalibrated) {
                status = LimitedMotorCalibrationStatus.TouchedSwitch;
            }

            if (status == LimitedMotorCalibrationStatus.TouchedSwitch) {
                setPosition(limitPosition);

                targetPower = calibrationUntriggerPower;
            
                if (stayBelowLimit) {
                    targetPower *= -1;
                }
            }

            if (targetPower > 0 == stayBelowLimit) {
                targetPower = 0;
            }

            if (power > 0 == stayBelowLimit) {
                power = 0;
            }
        }
        else if (status == LimitedMotorCalibrationStatus.NotCalibrated) {
            targetPower = calibrationTriggerPower;
            
            if (!stayBelowLimit) {
                targetPower *= -1;
            }
        }
        else if (status == LimitedMotorCalibrationStatus.TouchedSwitch) {
            status = LimitedMotorCalibrationStatus.Calibrated;
            targetPower = 0;
        }

        super.update();
    }
}
