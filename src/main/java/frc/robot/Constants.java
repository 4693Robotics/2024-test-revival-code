// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class DriveConstants {
        // Driving Parameters - Note that these are not the maximum capable speeds of
        // the robot, rather the allowed maximum speeds
        public static final double kMaxSpeedMetersPerSecond = 4.8;
        public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

        public static final double kDirectionSlewRate = 2.0; // radians per second
        public static final double kMagnitudeSlewRate = 3.0; // percent per second (1 = 100%)
        public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

        // Chassis configuration
        public static final double kTrackWidth = Units.inchesToMeters(25.5);
        // Distance between centers of right and left wheels on robot
        public static final double kWheelBase = Units.inchesToMeters(25.5);
        // Distance between front and back wheels on robot
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        // Angular offsets of the modules relative to the chassis in radians
        public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
        public static final double kFrontRightChassisAngularOffset = 0;
        public static final double kBackLeftChassisAngularOffset = Math.PI;
        public static final double kBackRightChassisAngularOffset = Math.PI / 2;

        // SPARK MAX CAN IDs
        public static final int kFrontLeftDrivingCanId = 3;
        public static final int kRearLeftDrivingCanId = 2;
        public static final int kFrontRightDrivingCanId = 4;
        public static final int kRearRightDrivingCanId = 1;

        public static final int kFrontLeftTurningCanId = 7;
        public static final int kRearLeftTurningCanId = 6;
        public static final int kFrontRightTurningCanId = 8;
        public static final int kRearRightTurningCanId = 5;

        public static final boolean kGyroReversed = false;
    }

    public static final class ModuleConstants {
        // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
        // This changes the drive speed of the module (a pinion gear with more teeth will result in a
        // robot that drives faster).
        public static final int kDrivingMotorPinionTeeth = 14;

        // Invert the turning encoder, since the output shaft rotates in the opposite direction of
        // the steering motor in the MAXSwerve Module.
        public static final boolean kTurningEncoderInverted = true;

        // Calculations required for driving motor conversion factors and feed forward
        public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
        public static final double kWheelDiameterMeters = 0.0762;
        public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
        // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
        public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
        public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters) /
            kDrivingMotorReduction;

        public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI) /
            kDrivingMotorReduction; // meters
        public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI) /
            kDrivingMotorReduction) / 60.0; // meters per second

        public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
        public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

        public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
        public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

        public static final double kDrivingP = 0.04;
        public static final double kDrivingI = 0;
        public static final double kDrivingD = 0;
        public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
        public static final double kDrivingMinOutput = -1;
        public static final double kDrivingMaxOutput = 1;

        public static final double kTurningP = 1;
        public static final double kTurningI = 0;
        public static final double kTurningD = 0;
        public static final double kTurningFF = 0;
        public static final double kTurningMinOutput = -1;
        public static final double kTurningMaxOutput = 1;

        public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
        public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

        public static final int kDrivingMotorCurrentLimit = 50; // amps
        public static final int kTurningMotorCurrentLimit = 20; // amps
    }
    public static final class IntakeConstants {
        public static final int kIntakeArmCanId = 9;
        public static final int kIntakeTopRollerCanId = 10;
        public static final int kIntakeBottomRollerCanId = 11;

        public static final IdleMode kIntakeArmIdleMode = IdleMode.kBrake;
        public static final IdleMode kIntakeTopRollerIdleMode = IdleMode.kBrake;
        public static final IdleMode kIntakeBottomRollerIdleMode = IdleMode.kBrake;

        public static final boolean kIntakeArmInverted = false;
        public static final boolean kIntakeTopRollerInverted = true;
        public static final boolean kIntakeBottomRollerInverted = true;

        public static final int kIntakeArmCurrentLimit = 50; // amps
        public static final int kIntakeTopRollerCurrentLimit = 20; // amps
        public static final int kIntakeBottomRollerCurrentLimit = 20; // amps

        public static final SoftLimitDirection kIntakeArmSoftLimit = SoftLimitDirection.kReverse;
    }

    public static final class ShooterConstants {
        public static final int kShooterTopCanId = 13;
        public static final int kShooterBottomCanId = 12;
        public static final int kFeederRightCanId = 16;
        public static final int kShooterAngleCanId = 15;

        public static final IdleMode kShooterTopIdleMode = IdleMode.kCoast;
        public static final IdleMode kShooterBottomIdleMode = IdleMode.kCoast;
        public static final IdleMode kFeederRightIdleMode = IdleMode.kBrake;
        public static final IdleMode kShooterAngleIdleMode = IdleMode.kBrake;

        public static final boolean kShooterTopInverted = true;
        public static final boolean kShooterBottomInverted = true;
        public static final boolean kFeederRightInverted = true;
        public static final boolean kShooterAngleInverted = true;

        public static final int kShooterTopCurrentLimit = 50; // amps
        public static final int kShooterBottomCurrentLimit = 50; // amps
        public static final int kFeederRightCurrentLimit = 50; // amps
        public static final int kShooterAngleCurrentLimit = 50; // amps

        public static final int kMaxWheelRPM = 4200;
        public static final int kWheelDiameter = 4; // inches
        public static final double kShooterEfficiency = 1;
        public static final double InitialNoteVelocity = (kMaxWheelRPM * kWheelDiameter * Math.PI / 60 / 12) * kShooterEfficiency;
    }

    public static final class HangerConstants {
        public static final int kRightHangerCanId = 18;
        public static final int kLeftHangerCanId = 17;

        public static final IdleMode kRightHangerIdleMode = IdleMode.kBrake;
        public static final IdleMode kLeftHangerIdleMode = IdleMode.kBrake;

        public static final boolean kRightHangerInverted = true;
        public static final boolean kLeftHangerInverted = false;

        public static final int kRightHangerCurrentLimit = 20; // amps
        public static final int kLeftHangerCurrentLimit = 20; // amps

        public static final SoftLimitDirection kRightHangerSoftLimit = SoftLimitDirection.kReverse;
        public static final SoftLimitDirection kLeftHangerSoftLimit = SoftLimitDirection.kReverse;
    }

    public static final class VisionConstants {
        public static final double kCameraDistanceFromGround = 0.36576; // meters
        public static final Transform3d kcameraPose = new Transform3d(new Translation3d(0, 0, 0.45), new Rotation3d(0, 0, 0));
    }

    public static final class LightConstants {
        public static final double kPWMColorRed = 0.61;
        public static final double kPWMColorOrange = 0.65;
        public static final double kPWMColorYellow = 0.69;
        public static final double kPWMColorGreen = 0.77;
        public static final double kPWMColorBlue = 0.87;
        public static final double kPWMColorViolet = 0.91;
    }

    public static final class ShuffleboardConstants {
      public static final String kPreGameTabName = "Pre Game";
      public static final String kAutoTabName = "Auto";
      public static final String kTeleopTabName = "Teleop";
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kSubsystemsControllerPort = 1;
        public static final double kDriveDeadband = 0.1;
        public static final double KSubsystemsDeadband = 0.1;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 1.68;
        public static final double kMaxAccelerationMetersPerSecondSquared = 1;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
        public static final double kDistanceToFarthestModuleMeters = 0.458;

        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;

        // Constraint for the motion profiled robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static final class NeoMotorConstants {
        public static final double kFreeSpeedRpm = 5676;
    }

    public static final class AprilTag2024Constants {
        public static final int kBlueSourceRight = 1;
        public static final int kBlueSourceLeft = 2;
        public static final int kRedSpeakerSide = 3;
        public static final int kRedSpeakerCenter = 4;
        public static final int kRedAmp = 5;
        public static final int kBlueAmp = 6;
        public static final int kBlueSpeakerCenter = 7;
        public static final int kBlueSpeakerSide = 8;
        public static final int kRedSourceRight = 9;
        public static final int kRedSourceLeft = 10;
        public static final int kRedStageLeft = 11;
        public static final int kRedStageRight = 12;
        public static final int kRedStageCenter = 13;
        public static final int kBlueStageCenter = 14;
        public static final int kBlueStageLeft = 15;
        public static final int kBlueStageRight = 16;
    }
}