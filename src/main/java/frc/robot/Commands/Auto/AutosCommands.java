package frc.robot.Commands.Auto;

import java.util.List;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Commands.IntakeOut;
import frc.robot.Commands.IntakeIn;
import frc.robot.Commands.RotateToAngle;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutosCommands {
    
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

        
    ProfiledPIDController thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    public AutosCommands() {
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public SequentialCommandGroup RightRedAuto(DriveSubsystem DriveSubsystem, IntakeSubsystem IntakeSubsystem, ShooterSubsystem ShooterSubsystem) {
         Trajectory Trajectory1 = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(new Translation2d(0, 0)),
            new Pose2d(0, 0, new Rotation2d(0)),
             config);
            
        return new SequentialCommandGroup(
        );
    }

        public SequentialCommandGroup MoveForward(DriveSubsystem DriveSubsystem, IntakeSubsystem IntakeSubsystem, ShooterSubsystem ShooterSubsystem) {

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(Units.degreesToRadians(0))),
        List.of(new Translation2d(1, 0)),
        new Pose2d(2, 0, new Rotation2d(Units.degreesToRadians(0))),
        config);

    SwerveControllerCommand swerveControllerCommand = this.getCommandWithTrajectory(DriveSubsystem, exampleTrajectory);

    // Reset odometry to the starting pose of the trajectory.
    DriveSubsystem.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return new ShootNoteAuto(IntakeSubsystem, ShooterSubsystem)
    .andThen(new WaitCommand(1)
    .andThen(swerveControllerCommand
    .andThen(() -> DriveSubsystem.drive(0, 0, 0, false, false))));
    }

    private SwerveControllerCommand getCommandWithTrajectory(DriveSubsystem DriveSubsystem, Trajectory Trajectory) {
        return new SwerveControllerCommand(
            Trajectory,
            DriveSubsystem::getPose, // Functional interface to feed supplier
            DriveConstants.kDriveKinematics,
    
            // Position controllers
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            thetaController,
            DriveSubsystem::setModuleStates,
            DriveSubsystem);
    }
}

    


