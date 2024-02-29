package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/*
 * 
 * TODO: modes for shooting; controller switches with 3 diff power modes
 * 
 */



public class ShootingSubsystem extends SubsystemBase {

    private CANSparkMax m_shooter1 = new CANSparkMax(11, MotorType.kBrushless);
    private CANSparkMax m_shooter2 = new CANSparkMax(12, MotorType.kBrushless);

    public ShootingSubsystem() {
        m_shooter1.setInverted(true);
    }

    public void shoot() {
        
        m_shooter1.set(1);
        m_shooter2.set(1);
    }

    public void stop(){

        m_shooter1.stopMotor();
        m_shooter2.stopMotor();
    }
    @Override
    public void periodic() {
    RelativeEncoder shooter1Encoder = m_shooter1.getEncoder();
    RelativeEncoder shooter2Encoder = m_shooter2.getEncoder();



}
}
