package frc.robot.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class DriveTranslation extends Command{

    DriveSubsystem drivesystem;

    PIDController xPIDController;
    PIDController yPIDController;

    double xdistance;
    double ydistance;
    double maxSpeed;

    boolean isDisplaced;

    public DriveTranslation(DriveSubsystem DriveSubsystem, double xDistanceMeters, double yDistanceMeters, double maximimSpeed) {

        this.drivesystem = DriveSubsystem;

        this.xPIDController = new PIDController(0.1, 0, 0);
        this.yPIDController = new PIDController(0.1, 0, 0);

        this.xdistance = xDistanceMeters;
        this.ydistance = yDistanceMeters;
        this.maxSpeed = maximimSpeed;

        addRequirements(DriveSubsystem);

    }

    public void initialize() {

        drivesystem.resetDisplacement();

        xPIDController.setSetpoint(xdistance);
        yPIDController.setSetpoint(ydistance);

        xPIDController.setTolerance(0.05);
        yPIDController.setTolerance(0.05);
    }
    
    public void execute() {

        double xOutput = xPIDController.calculate(drivesystem.getXDisplacement());
        double yOutput = yPIDController.calculate(drivesystem.getyDisplacement());
        double xOutputLimit = Math.copySign(Math.min(Math.abs(xOutput), maxSpeed), xOutput);
        double yOutputLimit = Math.copySign(Math.min(Math.abs(yOutput), maxSpeed), yOutput);

        drivesystem.drive(xOutputLimit, yOutputLimit, 0, false, false);

        isDisplaced = xPIDController.atSetpoint() && yPIDController.atSetpoint();
    }

    public void end(){

        drivesystem.drive(0, 0, 0, false, false);
    }

    public boolean isFinished(boolean interrupted) {
        return isDisplaced;
    } 
    
}
