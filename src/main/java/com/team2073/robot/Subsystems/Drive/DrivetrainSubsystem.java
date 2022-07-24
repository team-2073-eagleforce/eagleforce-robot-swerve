package com.team2073.robot.Subsystems.Drive;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import com.team2073.common.periodic.AsyncPeriodicRunnable;
import com.team2073.robot.ApplicationContext;
import com.team2073.robot.Subsystems.SwerveModule;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import javax.crypto.spec.PSource;
import java.nio.file.Path;
import java.util.List;

import static com.team2073.robot.AppConstants.DriveConstants.*;
import static com.team2073.robot.AppConstants.AutoConstants.*;

public class DrivetrainSubsystem extends SubsystemBase implements AsyncPeriodicRunnable  {
    ApplicationContext appCTX = ApplicationContext.getInstance();

    public static double maxSpeedMeters = Units.feetToMeters(16.2);
    public static double maxSpeedFeet = 13.6;
    public static double maxAngularSpeed = 2*Math.PI;

    private Field2d field2d = appCTX.getField2d();

    private ShuffleboardTab tuning = Shuffleboard.getTab("tuning");

    public static double frontLeftOffset = 269.25;
    public static double frontRightOffset = 310;
    public static double backLeftOffset = 260;
    public static double backRightOffset = 319;

    public PigeonIMU gyro = appCTX.getGyro();


    private SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(kinematics, Rotation2d.fromDegrees(getHeading()));

    private SwerveModule[] modules = new SwerveModule[] {
            new SwerveModule(appCTX.getFrontLeftDriveMotor(), appCTX.getFrontLeftSteerMotor(), appCTX.getFrontLeftEncoder(), frontLeftOffset),
            new SwerveModule(appCTX.getFrontRightDriveMotor(), appCTX.getFrontRightSteerMotor(), appCTX.getFrontRightEncoder(), frontRightOffset),
            new SwerveModule(appCTX.getBackLeftDriveMotor(), appCTX.getBackLeftSteerMotor(), appCTX.getBackLeftEncoder(), backLeftOffset),
            new SwerveModule(appCTX.getBackRightDriveMotor(), appCTX.getBackRightSteerMotor(), appCTX.getBackRightEncoder(), backRightOffset)
    };

    public DrivetrainSubsystem(){
        autoRegisterWithPeriodicRunner();
        SmartDashboard.putData("Field2d", field2d);
//        gyro.setYaw(0);
    }


    @Override
    public void onPeriodicAsync() {
        m_odometry.update(Rotation2d.fromDegrees(gyro.getYaw()),modules[0].getState(),modules[1].getState(),modules[2].getState(),modules[3].getState());
        SmartDashboard.putNumber("Pose rot", getPose().getRotation().getDegrees());
        SmartDashboard.putNumber("Yaw", gyro.getYaw());

        field2d.setRobotPose(getPose());

        for ( int i = 0; i < modules.length; i++) {

            var modulePositionFromChassis = kModulePositions[i]
                    .rotateBy(Rotation2d.fromDegrees(getHeading()))
                    .plus(getPose().getTranslation());
            m_modulePose[i] = new Pose2d(modulePositionFromChassis,modules[i].getAngle());
        }
        field2d.getObject("Swerve Modules").setPoses(m_modulePose);
    }

    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean calibrateGyro) {
        if (calibrateGyro) {
        }
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(
                fieldRelative
                    ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, Rotation2d.fromDegrees(gyro.getYaw() + 180))
                    : new ChassisSpeeds(xSpeed, ySpeed, rot));
        SwerveDriveKinematics.desaturateWheelSpeeds(states, maxSpeedMeters);
        for (int i = 0; i < modules.length; i++) {
            SwerveModule module = modules[i];
            SwerveModuleState state = states[i];
            module.setState(state);
        }
    }

    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose, PathPlannerTrajectory trajectory) {
        SmartDashboard.putNumber("init Pose rot", trajectory.getInitialPose().getRotation().getDegrees());
        gyro.setYaw(trajectory.getInitialState().holonomicRotation.getDegrees());
        m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
    }

    public double getHeading() {
        return Math.IEEEremainder(gyro.getYaw(),360);
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
                desiredStates, kMaxSpeedMetersPerSecond);
        for (int i = 0; i < modules.length; i++) {
            SwerveModule module = modules[i];
            module.setState(desiredStates[i]);
        }
    }

    Pose2d[] m_modulePose = {
            new Pose2d(),
            new Pose2d(),
            new Pose2d(),
            new Pose2d()
    };

    @Override
    public void simulationPeriodic() {
        field2d.setRobotPose(getPose());

        for ( int i = 0; i < modules.length; i++) {

            var modulePositionFromChassis = kModulePositions[i]
                    .rotateBy(Rotation2d.fromDegrees(getHeading()))
                    .plus(getPose().getTranslation());
            m_modulePose[i] = new Pose2d(modulePositionFromChassis,modules[i].getAngle());
        }
        field2d.getObject("Swerve Modules").setPoses(m_modulePose);
    }


}
