package frc.robot.Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeMove extends Command {

    IntakeSubsystem intakeSubsystem;
    XboxController controller;

     public IntakeMove(IntakeSubsystem IntakeSubsystem, XboxController controller) {
        intakeSubsystem = IntakeSubsystem;
        this.controller = controller;

        addRequirements(IntakeSubsystem);
     }

     @Override
     public void execute() {
      double YRightspeed = MathUtil.applyDeadband(controller.getRightY(), OIConstants.KSubsystemsDeadband);
      intakeSubsystem.setIntakeRollerSpeed(YRightspeed);
     }

     @Override
     public void end(boolean interrupted) {
        intakeSubsystem.setIntakeRollerSpeed(0);
     }

     @Override
     public boolean isFinished() {
       return false;
     }
    
}
