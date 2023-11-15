package frc.robot.subsystems;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.CTREConfigs;
import frc.robot.SwerveConstants;
import lib.math.Conversions;
import lib.util.CTREModuleState;
import lib.util.SwerveModuleConstants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
// import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

public class SwerveModule {
    public int moduleNumber;
    private Rotation2d angleOffset;
    private Rotation2d lastAngle;

    private TalonFX mAngleMotor;
    private TalonFX mDriveMotor;
    public CANCoder angleEncoder;

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(SwerveConstants.driveKS, SwerveConstants.driveKV, SwerveConstants.driveKA);
    CTREConfigs configs = new CTREConfigs();

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;
        
        angleEncoder = new CANCoder(moduleConstants.cancoderID);
        configAngleEncoder();

        mAngleMotor = new TalonFX(moduleConstants.angleMotorID);
        configAngleMotor();

        mDriveMotor = new TalonFX(moduleConstants.driveMotorID);
        configDriveMotor();

        lastAngle = getState().angle;
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        desiredState = CTREModuleState.optimize(desiredState, getState().angle); 
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / SwerveConstants.maxSpeed;
            mDriveMotor.set(ControlMode.PercentOutput, percentOutput);
        }
        else {
            double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond, SwerveConstants.wheelCircumference, SwerveConstants.driveGearRatio);
            mDriveMotor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward, feedforward.calculate(desiredState.speedMetersPerSecond));
        }
    }

    private void setAngle(SwerveModuleState desiredState) {
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (SwerveConstants.maxSpeed * 0.06)) ? lastAngle : desiredState.angle; //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        
        mAngleMotor.set(ControlMode.Position, Conversions.degreesToFalcon(angle.getDegrees(), SwerveConstants.angleGearRatio));
        lastAngle = angle;
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(Conversions.falconToDegrees(mAngleMotor.getSelectedSensorPosition(), SwerveConstants.angleGearRatio));
    }

    public Rotation2d getCanCoder() {
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
    }

    public void resetToAbsolute() {
        double absolutePosition = Conversions.degreesToFalcon(getCanCoder().getDegrees() - angleOffset.getDegrees(), SwerveConstants.angleGearRatio);
        // double absolutePosition = 0;
        mAngleMotor.setSelectedSensorPosition(absolutePosition);
    }

    private void configAngleEncoder() {        
        angleEncoder.configFactoryDefault();
        angleEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        angleEncoder.configAllSettings(configs.swerveCanCoderConfig);
        angleEncoder.setPositionToAbsolute();
        angleEncoder.getPosition();
    }

    private void configAngleMotor() {
        mAngleMotor.configFactoryDefault();
        mAngleMotor.setNeutralMode(NeutralMode.Coast);
        mAngleMotor.configAllSettings(configs.swerveAngleFXConfig);
        resetToAbsolute();
    }

    private void configDriveMotor() {        
        mDriveMotor.configFactoryDefault();
        mDriveMotor.setNeutralMode(NeutralMode.Coast);
        mDriveMotor.setSelectedSensorPosition(0);
        mDriveMotor.configAllSettings(configs.swerveDriveFXConfig);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
            Conversions.falconToMPS(mDriveMotor.getSelectedSensorVelocity(), SwerveConstants.wheelCircumference, SwerveConstants.driveGearRatio), 
            getAngle()
        ); 
    }
    
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            Conversions.falconToMeters(mDriveMotor.getSelectedSensorPosition(), SwerveConstants.wheelCircumference, SwerveConstants.driveGearRatio), 
            getAngle()
        );
    }
}
