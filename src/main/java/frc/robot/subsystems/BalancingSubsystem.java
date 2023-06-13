package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.kauailabs.navx.frc.AHRS;

import static frc.robot.Globals.*;

public class BalancingSubsystem extends SubsystemBase {
    private final AHRS gyro;
    private final SwerveDriveSubsystem swervedrive;
    private boolean enabled = false;

    public BalancingSubsystem(AHRS g, SwerveDriveSubsystem sd) {
        gyro = g;
        swervedrive = sd;
    }

    public void enable() {
        enabled = true;
        IS_BALANCING = true;
    }

    public void disable() {
        enabled = false;
        IS_BALANCING = false;
    }

    public boolean isEnabled() {
        return enabled;
    }

    @Override
    public void periodic() {
        if (!enabled) return;
        
        double pitch = Math.round(gyro.getPitch() / 5) * 5;
        double speed = -pitch * 0.0085;
        swervedrive.drive(new Translation2d(Math.signum(speed) * 0.42, 0), 0, true, true);
    }
}
