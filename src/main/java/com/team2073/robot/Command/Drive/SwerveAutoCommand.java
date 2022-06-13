package com.team2073.robot.Command.Drive;

import com.team2073.common.command.AbstractLoggingCommand;
import com.team2073.robot.AppConstants;
import com.team2073.robot.Subsystems.Drive.DrivetrainSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

import java.util.function.Supplier;

import static edu.wpi.first.wpilibj.util.ErrorMessages.requireNonNullParam;
import static com.team2073.robot.AppConstants.AutoConstants.*;

public class SwerveAutoCommand extends AbstractLoggingCommand {
    private final Trajectory m_trajectory;
    private final Supplier<Pose2d> m_pose;
    private final SwerveDriveKinematics m_kinematics;
    private final PIDController m_xController;
    private final PIDController m_yController;
    private final ProfiledPIDController m_thetaController;
    private final DrivetrainSubsystem m_robotDrive;
    private SwerveControllerCommand swerveControllerCommand;

    public SwerveAutoCommand(Trajectory traj, DrivetrainSubsystem drivetrainSubsystem) {
        m_trajectory = requireNonNullParam(traj, "trajectory", "SwerveAutoCommand");
        m_pose = requireNonNullParam(drivetrainSubsystem::getPose, "pose", "SwerveAutoCommand");
        m_kinematics = requireNonNullParam(AppConstants.DriveConstants.kinematics, "kinematics", "SwerveAutoCommand");
        m_xController = new PIDController(kPXController,0,0);
        m_yController = new PIDController(kPYController,0,0);
        m_thetaController = new ProfiledPIDController(kPThetaController,0,0, kThetaControllerConstraints);
        m_robotDrive = drivetrainSubsystem;
    }

    @Override
    protected void initializeDelegate() {
        m_thetaController.enableContinuousInput(-Math.PI, Math.PI);
        swerveControllerCommand = new SwerveControllerCommand(m_trajectory,m_pose, m_kinematics,m_xController,m_yController,m_thetaController,m_robotDrive::setModuleStates,m_robotDrive);
        m_robotDrive.resetOdometry(m_trajectory.getInitialPose());
        swerveControllerCommand.schedule();
//        swerveControllerCommand.initialize();
    }

    @Override
    protected void executeDelegate() {
//        swerveControllerCommand.execute();
    }

    @Override
    protected void endDelegate() {
        swerveControllerCommand.cancel();
//        swerveControllerCommand.end(false);
    }

    @Override
    protected boolean isFinishedDelegate() {
        return swerveControllerCommand.isFinished();
    }
}
