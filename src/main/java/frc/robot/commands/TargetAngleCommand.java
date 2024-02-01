package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Utils;
import frc.robot.Vector2D;
import frc.robot.subsystems.DriveSystem;
import frc.robot.subsystems.SwerveSystem;

public class TargetAngleCommand extends CommandBase {    
    private DriveSystem driveSystem = DriveSystem.get();
    private SwerveSystem swerveSystem = SwerveSystem.get();
    private AHRS gyro = new AHRS(Port.kMXP);

    private Vector2D target;
    private double duration;

    private double startTime;

    public TargetAngleCommand(Vector2D target, double duration) {
        this.target = target;
        this.duration = duration;
    }

    @Override
    public void initialize() {
        startTime = Utils.getTime();
    }

    @Override
    public void execute() {
        swerveSystem.updateTargetAngle(target);
        swerveSystem.updateRotationPowerTargetted(gyro.getYaw());
        swerveSystem.apply(driveSystem, true);
    }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        return Utils.getTime() - startTime > duration;
    }
}
