package frc.robot.Commands.Auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeStop extends Command {

    IntakeSubsystem intakesystem;
    Timer timer;
    
    public IntakeStop(IntakeSubsystem IntakeSubsystem) {

        this.intakesystem = IntakeSubsystem;
        
        this.timer = new Timer();

        addRequirements(IntakeSubsystem);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();

        intakesystem.setArmPosition(0);
        intakesystem.setIntakeRollerSpeed(0);
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
    }

    @Override
    public boolean isFinished() {
        return timer.get() > 0.1;
        
    }
}
