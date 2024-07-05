package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;

public class CommandGroupFactory {
    public static final ShooterSubsystem shooter = ShooterSubsystem.getInstance();    
    public static final IntakeSubsystem intake = IntakeSubsystem.getInstance();

    public static Command getShootCommand(){
        return new ParallelDeadlineGroup(Commands.waitSeconds(0.02)
        .andThen(Commands.waitUntil(() -> shooter.isAtVelocity()).andThen(() -> intake.feedShooterCommand()))
        , shooter.setShooterSpeed(50));
    }
}
