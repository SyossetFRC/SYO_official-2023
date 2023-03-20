package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorPositionCommand extends CommandBase {
    private final ElevatorSubsystem m_elevatorSubsystem;

    private double m_startPosition;
    private double m_targetPosition;
    private final double m_power;

    public ElevatorPositionCommand(ElevatorSubsystem elevatorSubsystem, String position, double power) {
        m_elevatorSubsystem = elevatorSubsystem;
        if (position.equals("HIGH")) {
            m_targetPosition = 1.10;
        } 
        else if (position.equals("MID")) {
            m_targetPosition = 0.5;
        } 
        else if (position.equals("LOW")) {
            m_targetPosition = 0.15;
        }
        m_power = power;

        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
        m_startPosition = m_elevatorSubsystem.getElevatorAbsPosition();
        m_elevatorSubsystem.extend(Math.copySign(m_power, m_targetPosition - m_startPosition));
    }

    @Override
    public boolean isFinished() {
        if (Math.abs(m_elevatorSubsystem.getElevatorAbsPosition() - m_startPosition) < Math.abs(m_targetPosition - m_startPosition)) {
            return false;
        }
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        m_elevatorSubsystem.extend(0);
    }
}
