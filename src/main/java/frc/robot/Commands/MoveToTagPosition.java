package frc.robot.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class MoveToTagPosition extends Command {

    private DriveSubsystem drivesystem;
    private CameraSubsystem camerasystem;
    private PIDController xPIDController;
    private PIDController yPIDController;
    private PIDController rotPIDController;

    private double maxSpeed;
    private double xdistance;
    private double ydistance;
    private double rotdistance;

    private boolean inPosition;

    public MoveToTagPosition(DriveSubsystem DriveSubsystem, CameraSubsystem CameraSubsystem, double xdistance, double ydistance, double maxSpeed) {

        this.drivesystem = DriveSubsystem;
        this.camerasystem = CameraSubsystem;
        xPIDController = new PIDController(10, 0, 0);
        yPIDController = new PIDController(10, 0, 0);
        rotPIDController = new PIDController(5, 0, 0.5);

        this.maxSpeed = maxSpeed;
        this.xdistance = xdistance;
        this.ydistance = ydistance;
        this.rotdistance = 3.15  ; 

        addRequirements(DriveSubsystem, CameraSubsystem);
    }

    public void initialize() {

        xPIDController.setSetpoint(xdistance);
        yPIDController.setSetpoint(ydistance);
        rotPIDController.setSetpoint(rotdistance);

        rotPIDController.setTolerance(0.1);
    }

    public void execute() {
        if (camerasystem.getBestTagXDistance() != -1 && camerasystem.getBestTagYDistance() != -1) {
        double xOutput = xPIDController.calculate(camerasystem.getBestTagXDistance());
        double xOutputLimit = Math.copySign(Math.min(Math.abs(xOutput), maxSpeed), xOutput);

        double yOutput = yPIDController.calculate(camerasystem.getBestTagYDistance());
        double yOutputLimit = Math.copySign(Math.min(Math.abs(yOutput), maxSpeed), yOutput);

        double rotOutput = rotPIDController.calculate(camerasystem.get1BestTagYaw());
        double rotOutputLimit = Math.copySign(Math.min(Math.abs(rotOutput), maxSpeed), rotOutput);

        drivesystem.drive(-xOutputLimit, -yOutputLimit, rotOutputLimit, false, true);

        } else {
            drivesystem.drive(0, 0, 0, false, false);
        }

        inPosition = xPIDController.atSetpoint() && yPIDController.atSetpoint() && rotPIDController.atSetpoint();
    }

    public void end(boolean interrupted) {
        drivesystem.drive(0, 0, 0, false, false);
    }

    public boolean isFinished() {
        return inPosition;
    }
}