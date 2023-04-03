package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class IdleElevatorCommand extends CommandBase {
    private final ElevatorSubsystem m_elevatorSubsystem;

    private long m_waitTime;
    private Timer timer;

    public IdleElevatorCommand(ElevatorSubsystem elevatorSubsystem) {
        this(elevatorSubsystem, (long) Double.POSITIVE_INFINITY);
    }

    public IdleElevatorCommand(ElevatorSubsystem elevatorSubsystem,
                                long msecs) {
        this.m_elevatorSubsystem = elevatorSubsystem;
        this.m_waitTime = msecs;
        timer = new Timer();

        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        m_elevatorSubsystem.extend(0);
    }

    @Override
    public boolean isFinished() {
        if (timer.get() < (m_waitTime / 1000.0)) {
            return false;
        }
        return true;
    }
}