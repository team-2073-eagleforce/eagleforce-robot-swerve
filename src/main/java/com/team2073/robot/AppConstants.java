package com.team2073.robot;

import com.team2073.common.util.ConversionUtil;
import com.team2073.robot.Subsystems.Drive.DrivetrainSubsystem;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class AppConstants {
    private static AppConstants instance;

    //Front Left Module
    public int FRONT_LEFT_DRIVE_PORT = 11;
    public int FRONT_LEFT_STEER_PORT = 12;
    public int FRONT_LEFT_ENCODER_PORT = 1;

    //Front Right Module
    public int FRONT_RIGHT_DRIVE_PORT = 41;
    public int FRONT_RIGHT_STEER_PORT = 42;
    public int FRONT_RIGHT_ENCODER_PORT = 4;

    //Back Left Module
    public int BACK_LEFT_DRIVE_PORT = 21;
    public int BACK_LEFT_STEER_PORT = 22;
    public int BACK_LEFT_ENCODER_PORT = 2;

    //Back Right Module
    public int BACK_RIGHT_DRIVE_PORT = 31;
    public int BACK_RIGHT_STEER_PORT = 32;
    public int BACK_RIGHT_ENCODER_PORT = 3;


    private AppConstants(){}

    public static AppConstants getInstance(){
        if (instance == null){
            instance = new AppConstants();
        }
        return instance;
    }

    public static final class DriveConstants {

        public static final DCMotor kDriveMotorGearbox = DCMotor.getFalcon500(1);
        public static final DCMotor kTurnMotorGearbox = DCMotor.getFalcon500(1);
        public static final double kDriveGearRatio = 6.86;
        public static final double kTurnGearRatio = 12.8;



        public static final double kDriveEncoderDistancePerPulse =
                (ConversionUtil.inchesToMeters(4) * Math.PI) / ((double) 2048 * 6.89);

        public static final double kDriveSimEncoderDistancePerPulse = kDriveEncoderDistancePerPulse / 2;

        public static final double kCANCoderCPR = 4096;
        public static final double kTurningEncoderDistancePerPulse =
                // Assumes the encoders are on a 1:1 reduction with the module shaft.
                (360.0) / kCANCoderCPR;

        public static final double kTurningSimEncoderDistancePerPulse = kTurningEncoderDistancePerPulse;

        public static final boolean kGyroReversed = true;
        //Change later
//        public static final double GEAR_RATIO = 6.86;
        public static final double GEAR_RATIO = 1;

        //Change later
        public static final double ksVolts = 1;
        //Change later
        public static final double kvVoltSecondsPerMeter = 0.8;
        //Change later
        public static final double kaVoltSecondsSquaredPerMeter = 0.15;

        public static SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
                new Translation2d(
                        Units.inchesToMeters(10),
                        Units.inchesToMeters(10)
                ),
                new Translation2d(
                        Units.inchesToMeters(10),
                        Units.inchesToMeters(-10)
                ),
                new Translation2d(
                        Units.inchesToMeters(-10),
                        Units.inchesToMeters(10)
                ),
                new Translation2d(
                        Units.inchesToMeters(-10),
                        Units.inchesToMeters(-10)
                )
        );

        // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
        // These characterization values MUST be determined either experimentally or theoretically
        // for *your* robot's drive.
        // The RobotPy Characterization Toolsuite provides a convenient tool for obtaining these
        // values for your robot.
        public static final double kvVoltSecondsPerRadian = 0.16;
        public static final double kaVoltSecondsSquaredPerRadian = 0.0348;

        public static TrajectoryConfig config =
                new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
                        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                        .setKinematics(kinematics);

        public static final double kTrackWidth = ConversionUtil.inchesToMeters(24);
        // Distance between centers of right and left wheels on robot
        public static final double kWheelBase = ConversionUtil.inchesToMeters(24);;
        // Distance between front and back wheels on robot

        private static final Translation2d kFrontLeftModulePosition = new Translation2d(kWheelBase / 2, kTrackWidth / 2);
        private static final Translation2d kFrontRightModulePosition = new Translation2d(kWheelBase / 2, -kTrackWidth / 2);
        private static final Translation2d kBackLeftModulePosition = new Translation2d(-kWheelBase / 2, kTrackWidth / 2);
        private static final Translation2d kBackRightModulePosition = new Translation2d(-kWheelBase / 2, -kTrackWidth / 2);

        public static final Translation2d[] kModulePositions = {
                kFrontLeftModulePosition,
                kFrontRightModulePosition,
                kBackLeftModulePosition,
                kBackRightModulePosition
        };

    }

    public static final class AutoConstants {
        //Change later
        public static final double kMaxSpeedMetersPerSecond = 3;
        //Change later
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        //Change later
        public static final double kMaxAngularSpeedRadiansPerSecond =  2* Math.PI;
        //Change later
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        //Change later
        public static final double kPXController = 0.8;
        //Change later
        public static final double kPYController = 0.8;
        //Change later
        public static final double kPThetaController = .8;

        // Constraint for the motion profiled robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
                new TrapezoidProfile.Constraints(kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }
}
