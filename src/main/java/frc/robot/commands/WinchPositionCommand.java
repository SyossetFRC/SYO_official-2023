package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.WinchSubsystem;

public class WinchPositionCommand extends CommandBase {
    private final WinchSubsystem m_winchSubsystem;

    private double m_startAngle;
    private double m_targetAngle;
    private final double m_power;

    public WinchPositionCommand(WinchSubsystem winchSubsystem, String position, double power) {
        m_winchSubsystem = winchSubsystem;
        if (position.equals("DRIVE")) {
            m_targetAngle = 65;
        } else if (position.equals("OUT")) {
            m_targetAngle = 53;
        } else if (position.equals("IN")) {
            m_targetAngle = 22;
        }
        m_power = power;

        addRequirements(winchSubsystem);
    }

    @Override
    public void initialize() {
        m_startAngle = m_winchSubsystem.getWinchAbsPosition();
        m_winchSubsystem.rotate(Math.copySign(m_power, (m_targetAngle - m_startAngle)));
    }

    @Override
    public boolean isFinished() {
        if (Math.abs(m_winchSubsystem.getWinchAbsPosition() - m_startAngle) < Math.abs(m_targetAngle - m_startAngle)) {
            return false;
        }
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        m_winchSubsystem.rotate(0);
    }
}