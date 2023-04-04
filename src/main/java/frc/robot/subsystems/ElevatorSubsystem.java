package frc.robot.subsystems;

import static frc.robot.Constants.ELEVATOR_PULLEY_MOTOR_1;
import static frc.robot.Constants.ELEVATOR_PULLEY_MOTOR_2;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
    private CANSparkMax m_elevatorPulley1;
    private CANSparkMax m_elevatorPulley2;
    private double m_elevatorPulleySpeed = 0;

    private RelativeEncoder m_elevatorPulleyEncoder1;
    private RelativeEncoder m_elevatorPulleyEncoder2;

    DigitalInput m_topSwitch;
    DigitalInput m_bottomSwitch;

    public ElevatorSubsystem() {
        m_elevatorPulley1 = new CANSparkMax(ELEVATOR_PULLEY_MOTOR_1, MotorType.kBrushless);
        m_elevatorPulley1.setIdleMode(IdleMode.kBrake);
        m_elevatorPulley1.setInverted(true);

        m_elevatorPulley2 = new CANSparkMax(ELEVATOR_PULLEY_MOTOR_2, MotorType.kBrushless);
        m_elevatorPulley2.setIdleMode(IdleMode.kBrake);
        m_elevatorPulley2.follow(m_elevatorPulley1, true);

        m_elevatorPulleyEncoder1 = m_elevatorPulley1.getEncoder();
        m_elevatorPulleyEncoder1.setPositionConversionFactor(0.01); // Convert to meters

        m_elevatorPulleyEncoder2 = m_elevatorPulley2.getEncoder();
        m_elevatorPulleyEncoder2.setPositionConversionFactor(-0.01); // Convert to meters

        m_topSwitch = new DigitalInput(Constants.ELEVATOR_SWITCH_TOP);
        m_bottomSwitch = new DigitalInput(Constants.ELEVATOR_SWITCH_BOTTOM);
    }

    public void extend(double elevatorPulleySpeed) {
        m_elevatorPulleySpeed = elevatorPulleySpeed;
    }

    public double getElevatorAbsPosition() {
        return (m_elevatorPulleyEncoder1.getPosition() + m_elevatorPulleyEncoder2.getPosition()) / 2;
    }

    // Only resets when a match starts
    public void resetEncoders() {
        m_elevatorPulleyEncoder1.setPosition(0);
        m_elevatorPulleyEncoder2.setPosition(0);
    }

    @Override
    public void periodic() {
        if (!m_topSwitch.get() && m_elevatorPulleySpeed > 0) {
            m_elevatorPulley1.set(0);
            
        }
        else if (!m_bottomSwitch.get() && m_elevatorPulleySpeed < 0) {
            m_elevatorPulley1.set(0);
        }
        else {
            m_elevatorPulley1.set(m_elevatorPulleySpeed);
        }
    }
}
