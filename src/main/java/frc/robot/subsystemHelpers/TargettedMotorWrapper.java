// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystemHelpers;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import frc.robot.Utils;
import lib.math.Conversions;

/** Add your docs here. */
public class TargettedMotorWrapper extends MotorWrapper {
    
    
    public double gearRatio;
    //public boolean positionIsAngle;
    public double zeroThreshold, maxThreshold;

    public Double targetPosition = null; //assumes it doesnt loop rn

    public TargettedMotorWrapper(int[] motorNumbers, boolean[] inverted) {
        super(motorNumbers, inverted);

        gearRatio = 1;
        //positionIsAngle = false;
        targetPosition = null;
        zeroThreshold = 1;
        maxThreshold = 1;
    }

    public double getPosition() {
        return Conversions.falconToDegrees(motors[0].getSelectedSensorPosition(), gearRatio);
    }

    public void setPosition(double position) {
        motors[0].setSelectedSensorPosition(Conversions.degreesToFalcon(position, gearRatio));
    }

    @Override
    public void update() {
        if (targetPosition != null) {
            double offset = Math.abs(getPosition() - targetPosition);

            targetPower = (offset - zeroThreshold) / (maxThreshold - zeroThreshold);
            targetPower = Math.min(Math.max(targetPower, 0), 1);
            if (targetPosition - getPosition() < 0) {
                targetPower *= -1;
            }
        }
        
        super.update();
    }
}
