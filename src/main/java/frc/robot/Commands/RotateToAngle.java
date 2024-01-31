package frc.robot.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.DriveSubsystem;

public class RotateToAngle extends Command {

    private DriveSubsystem DriveSubsystem;
    private double targetAngle;
    private PIDController pidController;

    public RotateToAngle(DriveSubsystem DriveSubsystem, double targetAngle) {
        this.DriveSubsystem = DriveSubsystem;
        this.targetAngle = targetAngle;
        
        // Tune these PID constants based on your robot and testing
        this.pidController = new PIDController(0.03, 0.0, 0.0);
        
        // Add the DriveSubsystem as a requirement to ensure it is not used elsewhere
        addRequirements(DriveSubsystem);
    }

    @Override
    public void initialize() {
        // Reset the gyro and set the setpoint for the PID controller
        DriveSubsystem.zeroHeading();
        pidController.setSetpoint(targetAngle);
    }

    @Override
    public void execute() {
        // Calculate the output from the PID controller
        double output = pidController.calculate(DriveSubsystem.getHeading());

        // Apply the output to rotate the robot
        DriveSubsystem.drive(0, 0, output, false, false);
    }

    @Override
    public boolean isFinished() {
        // This command is finished when the robot's angle is close to the target angle
        return pidController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the rotation when the command ends
        DriveSubsystem.drive(0, 0, 0, true, false);
    }

}