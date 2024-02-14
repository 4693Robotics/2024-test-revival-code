package frc.robot.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeNote extends Command {

    IntakeSubsystem intakesystem;
    PIDController armPIDController;
    Timer timer;

    double intakeRollerSpeed;
    double armPosition;


    public IntakeNote(IntakeSubsystem IntakeSubsystem) {
        this.intakesystem = IntakeSubsystem;
        this.timer = new Timer();

        addRequirements(IntakeSubsystem);
    }

    public void initialize() {
        timer.start();
        intakesystem.moveIntakeArm(0.1);
    }

    @Override
    public void execute() {

         
        if (timer.get() > 1.25) {
            intakesystem.moveIntakeArm(0);
        }
    }

    @Override
    public void end(boolean interrupted) {
     intakesystem.moveIntakeRoller(0);
    }

    @Override
    public boolean isFinished() {
      return false;
    }
}