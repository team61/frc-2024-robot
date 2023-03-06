package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

import com.kauailabs.navx.frc.AHRS;

public class BalancingSubsystem extends SubsystemBase {
    private final AHRS gyro;
    private final DriveTrain drivetrain;
    private boolean enabled = false;
    private double previousRoll = 0;
    private double currentRoll = 0;

    public BalancingSubsystem(AHRS g, DriveTrain dt) {
        gyro = g;
        drivetrain = dt;
    }

    public void enable() {
        enabled = true;
        drivetrain.disableBreaks();
    }

    public void disable() {
        enabled = false;
        drivetrain.enableBreaks();
    }

    public boolean isEnabled() {
        return enabled;
    }

    @Override
    public void periodic() {
        if (!enabled) return;

        // double newVoltage = -gyro.getRate() * 10;
        // double volts = clamp(newVoltage, -1, 1);
        // if (volts < 0.1 && volts > -0.1) {
        //     drivetrain.tankdrive.stop();
        // } else {
        //     drivetrain.tankdrive.driveVolts(volts, volts);
        // }
        
        previousRoll = currentRoll;
        currentRoll = gyro.getRoll();
        double rollRate = currentRoll - previousRoll;
        double speed = -rollRate / 4;
        double error = gyro.getRate();
        drivetrain.tankdrive.driveSpeed(speed + error * Math.abs(error / 4), speed - error * Math.abs(error));
    }
}
