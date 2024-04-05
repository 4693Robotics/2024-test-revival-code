package frc.robot.Commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HangerSubsystem;

public class HangerMove extends Command {

    HangerSubsystem hangersystem;
    XboxController controller;

    public HangerMove(HangerSubsystem HangerSubsystem, XboxController controller) {

        this.hangersystem = HangerSubsystem;
        this.controller = controller;

        addRequirements(HangerSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if (controller.getRightBumper()){
            hangersystem.setRightHangerSpeed(1);
        } else {
            hangersystem.setHangerSpeed(0);
        }

        if (controller.getLeftBumper()) {
            hangersystem.setLeftHangerSpeed(1);
        } else {
            hangersystem.setLeftHangerSpeed(0);
        }
        
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
