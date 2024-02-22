package frc.robot.commands;

import java.sql.Time;
import java.time.Instant;
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

public class MoveForSecondsCommand extends CommandBase {    
    private SwerveSystem swerveSystem = SwerveSystem.get();
    private AHRS gyro = new AHRS(Port.kMXP);
    
    private Vector2D translation;
    private double rotationPower;
    private double duration;

    private double startTime;

    public MoveForSecondsCommand(Vector2D translation, double duration) {
        this.translation = translation;
        this.rotationPower = rotationPower;
        this.duration = duration;
    }

    @Override
    public void initialize() {
        startTime = Utils.getTime();

        swerveSystem.updateTranslationVector(translation, gyro.getYaw());
    }

    @Override
    public void execute() {
        
    }

    @Override
    public void end(boolean interrupted) {
        swerveSystem.updateTranslationVector(Vector2D.zero, gyro.getYaw());
    }

    @Override
    public boolean isFinished() {
        return Utils.getTime() - startTime >= duration;
    }
}
