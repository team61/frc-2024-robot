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
    private SwerveSystem swerveSystem = SwerveSystem.get();

    private double target;

    public TargetAngleCommand(double target) {
        this.target = target;
    }

    @Override
    public void initialize() {
        swerveSystem.forceTargetAngle(target);
    }

    @Override
    public void execute() {
        
    }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
