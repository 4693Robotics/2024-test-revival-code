package frc.robot.Commands;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CameraSubsystem;

public class MoveToTagPosition extends Command {
    
    private PhotonTrackedTarget selectedTarget;
    private PIDController xPidController;
    private double xdistance;

    public MoveToTagPosition(CameraSubsystem CameraSubsystem, double xdistance) {

        this.selectedTarget = CameraSubsystem.getTarget();
        this.xdistance = xdistance;

        xPidController = new PIDController(0.1, 0, 0);
    }

    public void initialize() {

    }

    public void execute() {

    }

    public void end(boolean interrupted) {
        
    }
    public boolean isFinished() {
        return false;
    }
}