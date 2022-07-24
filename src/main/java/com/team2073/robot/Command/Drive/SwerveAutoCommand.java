// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team2073.robot.Command.Drive;

import static edu.wpi.first.wpilibj.util.ErrorMessages.requireNonNullParam;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.team2073.robot.AppConstants;
import com.team2073.robot.ApplicationContext;
import com.team2073.robot.Subsystems.Drive.DrivetrainSubsystem;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

import static com.team2073.robot.AppConstants.DriveConstants.*;
import static com.team2073.robot.AppConstants.AutoConstants.*;

import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * A command that uses two PID controllers ({@link PIDController}) and a ProfiledPIDController
 * ({@link ProfiledPIDController}) to follow a trajectory {@link Trajectory} with a swerve drive.
 *
 * <p>This command outputs the raw desired Swerve Module States ({@link SwerveModuleState}) in an
 * array. The desired wheel and module rotation velocities should be taken from those and used in
 * velocity PIDs.
 *
 * <p>The robot angle controller does not follow the angle given by the trajectory but rather goes
 * to the angle given in the final state of the trajectory.
 *
 * <p>This class is provided by the NewCommands VendorDep
 */
@SuppressWarnings("MemberName")
public class SwerveAutoCommand extends Command {
    private final Timer m_timer = new Timer();
    private final PathPlannerTrajectory m_trajectory;
    private final Supplier<Pose2d> m_pose;
    private final SwerveDriveKinematics m_kinematics;
    private final HolonomicDriveController m_controller;
    private final Consumer<SwerveModuleState[]> m_outputModuleStates;
    private final ProfiledPIDController thetaController;
//        private final Supplier<Rotation2d> m_desiredRotation;
    private final PigeonIMU gyro = ApplicationContext.getInstance().getGyro();

    public SwerveAutoCommand(PathPlannerTrajectory traj, DrivetrainSubsystem drivetrainSubsystem) {
        m_trajectory = requireNonNullParam(traj, "trajectory", "SwerveControllerCommand");
        m_pose = requireNonNullParam(drivetrainSubsystem::getPose, "pose", "SwerveControllerCommand");
        m_kinematics = requireNonNullParam(kinematics, "kinematics", "SwerveControllerCommand");
        thetaController = new ProfiledPIDController(kPThetaController, 0, 0, kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        m_controller =
                new HolonomicDriveController(
                        requireNonNullParam(new PIDController(kPXController, 0, 0.005), "xController", "SwerveControllerCommand"),
                        requireNonNullParam(new PIDController(kPYController, 0, 0.005), "xController", "SwerveControllerCommand"),
                        requireNonNullParam(thetaController, "thetaController", "SwerveControllerCommand"));
        m_controller.setTolerance(new Pose2d(0.25,0.25,new Rotation2d(0.10)));
//        m_desiredRotation = () -> traj.getState(traj.getStates().size() - 1).holonomicRotation;
        m_outputModuleStates = requireNonNullParam(drivetrainSubsystem::setModuleStates, "frontLeftOutput", "SwerveControllerCommand");
    }

    @Override
    public void initialize() {
        m_timer.reset();
        m_timer.start();
    }

    @Override
    public void execute() {
        double curTime = m_timer.get();
        PathPlannerTrajectory.PathPlannerState desiredState = (PathPlannerTrajectory.PathPlannerState) m_trajectory.sample(curTime);
        var targetChassisSpeeds = m_controller.calculate(m_pose.get(), desiredState, desiredState.holonomicRotation);
        var targetModuleStates = m_kinematics.toSwerveModuleStates(targetChassisSpeeds);

        m_outputModuleStates.accept(targetModuleStates);
    }

    @Override
    public void end() {
        m_timer.stop();
    }

    @Override
    public boolean isFinished() {
        return m_timer.hasElapsed(m_trajectory.getTotalTimeSeconds());
    }
}
