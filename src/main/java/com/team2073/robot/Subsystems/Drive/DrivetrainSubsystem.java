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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import javax.crypto.spec.PSource;
import java.nio.file.Path;
import java.util.List;

import static com.team2073.robot.AppConstants.DriveConstants.*;
import static com.team2073.robot.AppConstants.AutoConstants.*;

public class DrivetrainSubsystem extends SubsystemBase implements AsyncPeriodicRunnable {
    ApplicationContext appCTX = ApplicationContext.getInstance();

    public static double maxSpeedMeters = Units.feetToMeters(16.2);
    public static double maxSpeedFeet = 13.6;
    public static double maxAngularSpeed = 2*Math.PI;

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
        gyro.setYaw(0);
    }


    @Override
    public void onPeriodicAsync() {
        m_odometry.update(Rotation2d.fromDegrees(getHeading()),modules[0].getState(),modules[1].getState(),modules[2].getState(),modules[3].getState());
    }

    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean calibrateGyro) {
        if (calibrateGyro) {
        }
        SmartDashboard.putNumber("gyro Angle", gyro.getFusedHeading());
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(
                fieldRelative
                    ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, Rotation2d.fromDegrees(gyro.getYaw()))
                    : new ChassisSpeeds(xSpeed, ySpeed, rot));
        SmartDashboard.putNumber("Chassis Speed", kinematics.toChassisSpeeds(states[0], states[1], states[2], states[3]).vxMetersPerSecond/maxSpeedMeters);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, maxSpeedMeters);
        for (int i = 0; i < modules.length; i++) {
            SwerveModule module = modules[i];
            SwerveModuleState state = states[i];
            SmartDashboard.putNumber(String.valueOf(i), state.speedMetersPerSecond);
            SmartDashboard.putNumber("state: " + String.valueOf(i), state.angle.getDegrees());
            module.setState(state);
        }
    }

    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
//        gyro.setYaw(pose.getRotation().getDegrees());
        m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
    }

    public void zeroHeading() {
//        gyro.setFusedHeading(0);
    }

    public double getHeading() {
        return Math.IEEEremainder(gyro.getFusedHeading(),360);
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
                desiredStates, kMaxSpeedMetersPerSecond);
        for (int i = 0; i < modules.length; i++) {
            SwerveModule module = modules[i];
            module.setState(desiredStates[i]);
        }
    }
}
