package com.team2073.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.team2073.common.util.ConversionUtil;
import com.team2073.robot.Subsystems.Drive.DrivetrainSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import static com.team2073.robot.AppConstants.DriveConstants.*;

public class SwerveModule {

    private static final double kDriveP = 15.0;
    private static final double kDriveI = 0.01;
    private static final double kDriveD = 0.1;
    private static final double kDriveF = 0.2;

    private static final double kAngleP = 1.0;
    private static final double kAngleI = 0.0;
    private static final double kAngleD = 0.0;

    private static double encoderTicksPerRotation = 4096;

    private TalonFX driveMotor;
    private TalonFX steerMotor;
    private CANCoder canCoder;

    private Rotation2d offset;

    public SwerveModule(TalonFX driveMotor, TalonFX steerMotor, CANCoder canCoder, Rotation2d offset){
        this.driveMotor = driveMotor;
        this.steerMotor = steerMotor;
        this.canCoder = canCoder;
        this.offset = offset;
        System.out.println("auduisyad");

        TalonFXConfiguration angleTalonFXConfiguration = new TalonFXConfiguration();

        angleTalonFXConfiguration.slot0.kP = kAngleP;
        angleTalonFXConfiguration.slot0.kI = kAngleI;
        angleTalonFXConfiguration.slot0.kD = kAngleD;

        angleTalonFXConfiguration.remoteFilter0.remoteSensorDeviceID = canCoder.getDeviceID();
        angleTalonFXConfiguration.remoteFilter0.remoteSensorSource = RemoteSensorSource.CANCoder;

        angleTalonFXConfiguration.primaryPID.selectedFeedbackSensor = FeedbackDevice.RemoteSensor0;
        steerMotor.configAllSettings(angleTalonFXConfiguration);
        steerMotor.setNeutralMode(NeutralMode.Brake);

        TalonFXConfiguration driveTalonFXConfiguration = new TalonFXConfiguration();

        driveTalonFXConfiguration.slot0.kP = kDriveP;
        driveTalonFXConfiguration.slot0.kI = kDriveI;
        driveTalonFXConfiguration.slot0.kD = kDriveD;
        driveTalonFXConfiguration.slot0.kF = kDriveF;

        driveMotor.configAllSettings(driveTalonFXConfiguration);
        driveMotor.setNeutralMode(NeutralMode.Brake);

        CANCoderConfiguration canCoderConfiguration = new CANCoderConfiguration();
        canCoderConfiguration.magnetOffsetDegrees = offset.getDegrees();
        canCoder.configAllSettings(canCoderConfiguration);
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(canCoder.getAbsolutePosition());
    }

    public double getRawAngle() {
        return canCoder.getAbsolutePosition();
    }

    public static SwerveModuleState optimize(
            SwerveModuleState desiredState, Rotation2d currentAngle) {
        Rotation2d delta = desiredState.angle.minus(currentAngle);
        if (Math.abs(delta.getDegrees()) > 90.0) {
            return new SwerveModuleState(
                    -desiredState.speedMetersPerSecond,
                    desiredState.angle.rotateBy(Rotation2d.fromDegrees(180.0)));
        } else {
            return new SwerveModuleState(desiredState.speedMetersPerSecond, desiredState.angle);
        }
    }

    public void setState(SwerveModuleState desiredState) {
        Rotation2d currentAngle = getAngle();
        SwerveModuleState state = optimize(desiredState, currentAngle);

        Rotation2d rotationDelta = state.angle.minus(currentAngle);

        double positionDelta = (rotationDelta.getDegrees()/360) * encoderTicksPerRotation;
        double currentPosition = canCoder.getPosition() /canCoder.configGetFeedbackCoefficient();
        double desiredPosition = currentPosition + positionDelta;

        steerMotor.set(TalonFXControlMode.Position, desiredPosition);

        double feetPerSecond = Units.metersToFeet(state.speedMetersPerSecond);

        driveMotor.set(TalonFXControlMode.PercentOutput, feetPerSecond/ DrivetrainSubsystem.maxSpeed);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(((driveMotor.getSelectedSensorVelocity() * GEAR_RATIO * 10)/ 2048 * ConversionUtil.inchesToMeters(4 * Math.PI)), getAngle());
    }
}
