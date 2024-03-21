package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class EncoderIntakeOut extends Command{

    private IntakeSubsystem intakesystem;

    EncoderIntakeOut(IntakeSubsystem intakeSubsystem) {

        this.intakesystem = intakeSubsystem; 
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        
    }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
