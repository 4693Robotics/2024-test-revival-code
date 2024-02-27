package frc.robot.Commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HangerSubsystem;

public class HangerMove extends Command {

    HangerSubsystem hangersystem;
    XboxController controller;

    double rightTrigger;
    double leftTrigger;

    public HangerMove(HangerSubsystem HangerSubsystem, XboxController controller) {

        this.hangersystem = HangerSubsystem;
        this.controller = controller;

        this.rightTrigger = controller.getRawAxis(2);
        this.leftTrigger = controller.getRawAxis(3);

        addRequirements(HangerSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        hangersystem.setRightHangerSpeed(rightTrigger);
        hangersystem.setLeftHangerSpeed(leftTrigger);

        SmartDashboard.putNumber("Right Trigger", rightTrigger);
        SmartDashboard.putNumber("Left Trigger", leftTrigger);
        
    }

    @Override
    public void end(boolean interrupted) {
        hangersystem.setHangerSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
