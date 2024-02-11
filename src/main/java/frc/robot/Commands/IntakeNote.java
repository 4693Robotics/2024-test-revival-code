package frc.robot.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeNote extends Command {

    IntakeSubsystem intakesystem;
    PIDController armPIDController;

    double intakeRollerSpeed;
    double armPosition;


    public IntakeNote(IntakeSubsystem IntakeSubsystem) {
        this.intakesystem = IntakeSubsystem;

        this.armPIDController = new PIDController(0.1, 0, 0);

        intakeRollerSpeed = 1;

        addRequirements(IntakeSubsystem);
    }

    @Override
    public void execute() {

        if (intakesystem.getAtLimit()) {
            intakesystem.setArmPosition(0);
        }

        

        new SequentialCommandGroup(
            new InstantCommand(() -> intakesystem.moveIntakeRoller(intakeRollerSpeed))

        );
    }

    @Override
    public void end(boolean interrupted) {
     intakesystem.moveIntakeArm(0);
     intakesystem.moveIntakeRoller(0);
    }

    @Override
    public boolean isFinished() {
      return false;
    }
}