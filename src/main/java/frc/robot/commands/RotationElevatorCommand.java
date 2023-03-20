package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.WinchSubsystem;

public class RotationElevatorCommand extends CommandBase {
    private final WinchSubsystem m_winchSubsystem;

    private final double m_elevatorRotationAngle;
    private double m_startingAngle;
    private final double m_power;

    public RotationElevatorCommand(WinchSubsystem winchSubsystem, double elevatorRotationAngle, double power) {
        m_winchSubsystem = winchSubsystem;
        m_elevatorRotationAngle = elevatorRotationAngle;
        m_power = power;

        addRequirements(winchSubsystem);
    }

    @Override
    public void initialize() {
        m_startingAngle = m_winchSubsystem.getWinchAbsPosition();
        m_winchSubsystem.rotate(Math.copySign(m_power, m_elevatorRotationAngle));
    }

    @Override
    public boolean isFinished() {
        if (Math.abs(m_winchSubsystem.getWinchAbsPosition() - m_startingAngle) < Math.abs(m_elevatorRotationAngle)) {
            return false;
        }
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        m_winchSubsystem.rotate(0);
    }
}
