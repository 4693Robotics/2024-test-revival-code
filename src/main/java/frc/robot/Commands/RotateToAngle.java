package frc.robot.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.DriveSubsystem;

public class RotateToAngle extends Command {

    private DriveSubsystem DriveSubsystem;
    private double targetAngle;
    private PIDController pidController;
    private double maxRotationSpeed;

    public RotateToAngle(DriveSubsystem DriveSubsystem, double targetAngle, double maximumSpeed) {
        this.DriveSubsystem = DriveSubsystem;
        this.targetAngle = targetAngle;
        this.maxRotationSpeed = maximumSpeed;
        
        // Tune these PID constants based on your robot and testing
        this.pidController = new PIDController(0.1, 0, 0);
        
        pidController.setTolerance(2);
        // Add the DriveSubsystem as a requirement to ensure it is not used elsewhere
        addRequirements(DriveSubsystem);
    }

    @Override
    public void initialize() {
        // Reset the gyro and set the setpoint for the PID controller
        DriveSubsystem.zeroHeading();
        pidController.setSetpoint(-targetAngle);
        pidController.setTolerance(3);
    }

    @Override
    public void execute() {
        // Calculate the output from the PID controller
        double output = pidController.calculate(DriveSubsystem.invertGyro_Angle());

        double limitedOutput = Math.copySign(Math.min(Math.abs(output), maxRotationSpeed), output);

        // Apply the output to rotate the robot
        DriveSubsystem.drive(0, 0, limitedOutput, false, false);
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the rotation when the command ends
        DriveSubsystem.drive(0, 0, 0, true, false);
    }

    @Override
    public boolean isFinished() {
        // This command is finished when the robot's angle is close to the target angle
        return pidController.atSetpoint();
    }
}