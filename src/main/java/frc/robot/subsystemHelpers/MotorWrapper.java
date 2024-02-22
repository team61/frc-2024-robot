// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystemHelpers;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import frc.robot.Utils;

/** Add your docs here. */
public class MotorWrapper {
    public TalonFX[] motors;
    public double factor, lerpFactor, minPower;

    public double power, targetPower;

    public MotorWrapper(int[] motorNumbers, boolean[] inverted) {
        motors = new TalonFX[motorNumbers.length];

        for (int i = 0; i < motorNumbers.length; i++) {
            motors[i] = new TalonFX(motorNumbers[i]);
        }

        setInverted(inverted);

        factor = 1;
        lerpFactor = 1;
        minPower = 0;
        setNeutralMode(NeutralMode.Brake);
    }

    public void setInverted(boolean[] inverted) {
        for (int i = 0; i < motors.length; i++) {
            motors[i].setInverted(inverted[i]);
        }
    }

    public void setNeutralMode(NeutralMode neutralMode) {
        for (TalonFX motor : motors) {
            motor.setNeutralMode(neutralMode);
        }
    }

    public void update() {
        power = Utils.lerp(power, targetPower, lerpFactor);

        double newPower = 0;
        double absPower = Math.abs(power);

        if (absPower > 0.0001) {
            newPower = absPower * factor;
            newPower = (newPower * (1 - minPower)) + minPower;

            if (power < 0) {
                newPower *= -1;
            }
        }

        for (TalonFX motor : motors) {
            motor.set(ControlMode.PercentOutput, newPower);
        }
    }
}
