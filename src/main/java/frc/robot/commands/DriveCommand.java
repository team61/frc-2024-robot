package frc.robot.commands;

import frc.robot.SwerveConstants;
import frc.robot.subsystems.SwerveDriveSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.Globals.*;

public class DriveCommand extends CommandBase {    
    private SwerveDriveSubsystem swervedrive;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;

    public DriveCommand(SwerveDriveSubsystem sd, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup) {
        swervedrive = sd;
        addRequirements(sd);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
    }

    @Override
    public void execute() {
        if (USE_OLD_SWERVE_DRIVE || IS_BALANCING) return;
        
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), 0.15);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), 0.15);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), 0.15);

        swervedrive.drive(
            new Translation2d(translationVal, strafeVal).times(SwerveConstants.maxSpeed), 
            rotationVal * SwerveConstants.maxAngularVelocity, 
            !robotCentricSup.getAsBoolean(), 
            true
        );
    }
}
