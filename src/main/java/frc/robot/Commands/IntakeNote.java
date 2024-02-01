package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeNote extends Command {

    double intakeSpeed;
    double time;
    IntakeSubsystem intakeSubsystem;

    public IntakeNote(double durration, IntakeSubsystem intake) {
        this.time = durration;
        this.intakeSubsystem = intake;

        intakeSpeed = 1;
    }

    @Override
    public void execute() {
        intakeSubsystem.moveIntakeRoller(intakeSpeed);
        Timer.delay(time);
    }

    @Override
    public void end(boolean interrupted) {
       intakeSubsystem.moveIntakeArm(0);
       intakeSubsystem.moveIntakeRoller(0);
    }

    @Override
    public boolean isFinished() {
      return false;
    }
}