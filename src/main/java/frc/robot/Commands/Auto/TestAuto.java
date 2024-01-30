package frc.robot.Commands.Auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Commands.RotateToAngle;
import frc.robot.subsystems.DriveSubsystem;

public class TestAuto extends Command {
    
    DriveSubsystem DriveSystem;
    double GyroAngle;

    public TestAuto(DriveSubsystem DriveSubsystem) {
        DriveSystem = DriveSubsystem;
        GyroAngle = DriveSubsystem.getHeading();

        addRequirements(DriveSubsystem);
    }

    @Override
    public void execute() {
        DriveSystem.zeroHeading();
        DriveSystem.drive(0.1, 0, 0, true, false); //moves robot forward for .2 seconds
        Timer.delay(0.2);
        new RotateToAngle(0.5, 90, DriveSystem); // Rotates the robot to face 90 degrees on the gyro at half speed
        DriveSystem.drive(0, 0, 0, true, false); //stops the robot
    }

    @Override
    public void end(boolean interrupted) {
        DriveSystem.drive(0, 0, 0, true, false); //stop robot when command ends
        DriveSystem.setX(); //set robot to xStance to prevent slipping
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
