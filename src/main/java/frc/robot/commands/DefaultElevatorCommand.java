package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.WinchSubsystem;

public class DefaultElevatorCommand extends CommandBase {
    private final ElevatorSubsystem m_elevatorSubsystem;
    private final WinchSubsystem m_winchSubsystem;

    private final DoubleSupplier m_elevatorPulleySpeed;
    private final DoubleSupplier m_winchSpeed;

    public DefaultElevatorCommand(ElevatorSubsystem elevatorSubsystem, WinchSubsystem winchSubsystem, DoubleSupplier elevatorPulleySpeed, DoubleSupplier winchSpeed) {
        m_elevatorSubsystem = elevatorSubsystem;
        m_winchSubsystem = winchSubsystem;
        m_elevatorPulleySpeed = elevatorPulleySpeed;
        m_winchSpeed = winchSpeed;

        addRequirements(elevatorSubsystem, winchSubsystem);
    }

    @Override
    public void execute() {
        m_elevatorSubsystem.extend(m_elevatorPulleySpeed.getAsDouble());
        m_winchSubsystem.rotate(m_winchSpeed.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        m_elevatorSubsystem.extend(0);
        m_winchSubsystem.rotate(0);
    }
}
