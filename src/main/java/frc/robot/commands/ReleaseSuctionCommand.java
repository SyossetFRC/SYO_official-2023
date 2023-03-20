package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class ReleaseSuctionCommand extends CommandBase {
    private final IntakeSubsystem m_intakeSubsystem;
    
    public ReleaseSuctionCommand(IntakeSubsystem intakeSubsystem) {
        m_intakeSubsystem = intakeSubsystem;

        addRequirements(m_intakeSubsystem);
    }

    @Override
    public void initialize() {
        m_intakeSubsystem.releaseSuction();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}