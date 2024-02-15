package frc.robot.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class LoadNote extends Command {

    IntakeSubsystem intakesystem;
    PIDController armPIDController;
    Timer timer;

    double intakeRollerSpeed;
    double armPosition;


    public LoadNote(IntakeSubsystem IntakeSubsystem) {
        this.intakesystem = IntakeSubsystem;
        this.timer = new Timer();

        addRequirements(IntakeSubsystem);
    }

    public void initialize() {
        timer.reset();
        timer.start();
        intakesystem.moveIntakeArm(-0.1);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
     timer.stop();
     intakesystem.moveIntakeArm(0);
    }

    @Override
    public boolean isFinished() {
      return timer.get() > 1.75;
    }
}
