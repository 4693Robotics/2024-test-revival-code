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

    public SequentialCommandGroup Auto2NoteMiddle(DriveSubsystem DriveSubsystem, IntakeSubsystem IntakeSubsystem, ShooterSubsystem ShooterSubsystem) {
        return new SequentialCommandGroup(
           new InstantCommand(() -> ShooterSubsystem.setShooterSpeed(1)),
           new InstantCommand(() -> IntakeSubsystem.setIntakeRollerSpeed(-1)), //Ejects loaded piece
           new WaitCommand(0.7),
           new InstantCommand(() -> IntakeSubsystem.setIntakeRollerSpeed(0)),
           new WaitCommand(0.4),
           new InstantCommand(() -> ShooterSubsystem.setShooterSpeed(0)), //Stops the Ejection
           new IntakeOut(IntakeSubsystem), //Moves the Intake out
           new WaitCommand(0.7),
           new InstantCommand(() -> IntakeSubsystem.setIntakeRollerSpeed(0.5)), //Turns on the roller 
           new RotateToAngle(DriveSubsystem, 0, 0), //Needs a rotate angle to not break things don't know why
           new InstantCommand(() -> DriveSubsystem.drive(-0.7, 0, 0, false, false)), //Move robot backwards for 0.6 secconds
           new WaitCommand(0.6),
           new InstantCommand(() -> DriveSubsystem.drive(0, 0, 0, false, false)), //Stops the robot
           new InstantCommand(() -> IntakeSubsystem.setIntakeRollerSpeed(0)),
           new IntakeIn(IntakeSubsystem), //Move the intake inwards
           new WaitCommand(0.7),
           new InstantCommand(() -> DriveSubsystem.drive(0.7, 0, 0, false, false)), //Moves Robot back to get ready to shoot 
           new WaitCommand(0.4),
           new InstantCommand(() -> DriveSubsystem.drive(0, 0, 0, false, false)), //Stops the robot
           new InstantCommand(() -> IntakeSubsystem.setIntakeRollerSpeed(-1)),
           new InstantCommand(() -> ShooterSubsystem.setShooterSpeed(1)), //Shoots the note
           new WaitCommand(0.4),
           new InstantCommand(() -> IntakeSubsystem.setIntakeRollerSpeed(0)), //Stops the shooter and the intake
           new InstantCommand(() -> ShooterSubsystem.setShooterSpeed(0))
        ); 
    }

    public SequentialCommandGroup Auto1NoteMove(DriveSubsystem DriveSubsystem, IntakeSubsystem IntakeSubsystem, ShooterSubsystem ShooterSubsystem) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> ShooterSubsystem.setShooterSpeed(1)),
            new InstantCommand(() -> IntakeSubsystem.setIntakeRollerSpeed(-1)), //Ejects loaded piece
            new WaitCommand(1),
            new InstantCommand(() -> IntakeSubsystem.setIntakeRollerSpeed(0)),
            new WaitCommand(0.4),
            new InstantCommand(() -> ShooterSubsystem.setShooterSpeed(0)), //Stops the Ejection
            new WaitCommand(0.7),
            new InstantCommand(() -> IntakeSubsystem.setIntakeRollerSpeed(0.5)), //Turns on the roller 
            new RotateToAngle(DriveSubsystem, 0, 0), //Needs a rotate angle to not break things don't know why
            new InstantCommand(() -> DriveSubsystem.drive(-0.7, 0, 0, false, false)), //Move robot backwards for 0.6 secconds
            new WaitCommand(0.6),
            new InstantCommand(() -> DriveSubsystem.drive(0, 0, 0, false, false)) //Stops the robot
        );
    }

    public SequentialCommandGroup DulthAuto(DriveSubsystem DriveSubsystem, IntakeSubsystem IntakeSubsystem, ShooterSubsystem ShooterSubsystem) {
        return new SequentialCommandGroup(
            new ShootNoteAuto(IntakeSubsystem, ShooterSubsystem),
            new IntakeOut(IntakeSubsystem),
            new WaitCommand(.5),
            new InstantCommand(() -> IntakeSubsystem.setIntakeRollerSpeed(0.5)),
            new InstantCommand(() -> DriveSubsystem.drive(0, 0, 0, false, false)),
            new WaitCommand(0.8),
            new InstantCommand(() -> DriveSubsystem.drive(0, 0, 0, false, false)),
            new IntakeIn(IntakeSubsystem),
            new InstantCommand(() -> DriveSubsystem.drive(0, 0, 0, false, false)),
            new WaitCommand(0.7),
            new InstantCommand(() -> DriveSubsystem.drive(0, 0, 0, false, false)),
            new ShootNoteAuto(IntakeSubsystem, ShooterSubsystem)
        );
    }
    public SequentialCommandGroup TestAuto(DriveSubsystem DriveSubsystem, IntakeSubsystem IntakeSubsystem, ShooterSubsystem ShooterSubsystem) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> DriveSubsystem.drive(-0.4, 0, 0, false, false)),
                new WaitCommand(0.8),
                new RotateToAngle(DriveSubsystem, 50, 0.)

        );
    }

    public SequentialCommandGroup BackUpAuto(DriveSubsystem DriveSubsystem, IntakeSubsystem IntakeSubsystem, ShooterSubsystem ShooterSubsystem) {
        return new SequentialCommandGroup(
            new ShootNoteAuto(IntakeSubsystem, ShooterSubsystem),
            new InstantCommand(() -> DriveSubsystem.drive(-0.4, 0, 0, false, false)),
            new WaitCommand(2),
            new InstantCommand(() -> DriveSubsystem.drive(0, 0, 0, false, false))
        );
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

    public SequentialCommandGroup Middle2Note(DriveSubsystem DriveSubsystem, IntakeSubsystem IntakeSubsystem, ShooterSubsystem ShooterSubsystem) {

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(Units.degreesToRadians(0))),
        List.of(new Translation2d(0.8, 0)),
        new Pose2d(0.8, 0, new Rotation2d(Units.degreesToRadians(0))),
        config); 


    SwerveControllerCommand swerveControllerCommand = this.getCommandWithTrajectory(DriveSubsystem, exampleTrajectory);

    DriveSubsystem.setGyroPosition(180);

    // Reset odometry to the starting pose of the trajectory.
    DriveSubsystem.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return new ShootNoteAuto(IntakeSubsystem, ShooterSubsystem)
    .andThen(new IntakeOut(IntakeSubsystem))
    .andThen(new RotateToAngle(DriveSubsystem, 0, 0))
    .andThen(swerveControllerCommand);
   // .andThen(() -> DriveSubsystem.drive(0, 0, 0, false, false));
    }

    public SequentialCommandGroup MoveForward(DriveSubsystem DriveSubsystem) {

        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(Units.degreesToRadians(0))),
            List.of(new Translation2d(1, 0)),
            new Pose2d(1, 0, new Rotation2d(Units.degreesToRadians(0))),
            config); 

        SwerveControllerCommand trajectoryCommand = this.getCommandWithTrajectory(DriveSubsystem, trajectory);

        return new SequentialCommandGroup(
            trajectoryCommand);
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

    


