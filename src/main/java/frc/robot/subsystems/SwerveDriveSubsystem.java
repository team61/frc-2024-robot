package frc.robot.subsystems;

import frc.robot.SwerveConstants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Globals.*;

public class SwerveDriveSubsystem extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public AHRS gyro;

    public SwerveDriveSubsystem(AHRS g) {
        gyro = g;
        zeroGyro();
        
        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, SwerveConstants.mod0),
            new SwerveModule(1, SwerveConstants.mod1),
            new SwerveModule(2, SwerveConstants.mod2),
            new SwerveModule(3, SwerveConstants.mod3),
        };

        Timer.delay(1.0);
        resetModulesToAbsolute();

        swerveOdometry = new SwerveDriveOdometry(SwerveConstants.kinematics, getYaw(), getModulePositions());
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        double forward = translation.getX();
        double strafe = -translation.getY();

        if (fieldRelative) {
            double gyroDegrees = getYaw().getDegrees();
            double gyroRadians = gyroDegrees * Math.PI / 180; 
            double temp = forward * Math.cos(gyroRadians) + strafe * Math.sin(gyroRadians);
            strafe = forward * Math.sin(gyroRadians) + -strafe * Math.cos(gyroRadians);
            forward = temp;
        }

        SwerveModuleState[] swerveModuleStates =
            SwerveConstants.kinematics.toSwerveModuleStates(
                new ChassisSpeeds(
                    forward, 
                    strafe, 
                    rotation)
                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }    

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.maxSpeed);
        
        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }    

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : mSwerveMods) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : mSwerveMods) {
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public void zeroGyro() {
        gyro.zeroYaw();
    }
    
    public Rotation2d getYaw() {
        double yaw = gyro.getYaw();
        if (yaw < 0) {
            yaw += 360;
        }
        return Rotation2d.fromDegrees(yaw);
    }

    public void resetModulesToAbsolute() {
        for (SwerveModule mod : mSwerveMods) {
            mod.resetToAbsolute();
        }
    }

    @Override
    public void periodic() {
        if (USE_OLD_SWERVE_DRIVE) return;
        
        swerveOdometry.update(getYaw(), getModulePositions());
    }
}
