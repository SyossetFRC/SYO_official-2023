package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class CloseIntakeCommand extends CommandBase {
    private final IntakeSubsystem m_intakeSubsystem;
    
    public CloseIntakeCommand(IntakeSubsystem intakeSubsystem) {
        m_intakeSubsystem = intakeSubsystem;

        addRequirements(m_intakeSubsystem);
    }

    @Override
    public void initialize() {
        m_intakeSubsystem.close();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}