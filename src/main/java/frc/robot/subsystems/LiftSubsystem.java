package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/*
 * TODO: reset encoder values
 * TODO: sync the motors based on encoder values
 * TODO: add lift + lower functions
 * TODO: functions set to fixed encoder values (lift + lower)
 * TODO: function to read encoder values (very important)
 */

public class LiftSubsystem extends SubsystemBase {
    private CANSparkMax lifter1 = new CANSparkMax(13, MotorType.kBrushless);
    private CANSparkMax lifter2 = new CANSparkMax(14, MotorType.kBrushless);
    private RelativeEncoder lifter2Encoder, lifter1Encoder;
    private double encoder1Position = 0, encoder2Position = 0;
    private double difference;
    private double encoder1Scale = 1;
    private double encoder2Scale = 1;
    private double maxDeviation = 4;
    private double speed = -0.50;
    

    public void control(double direction) {
        syncMotors(speed * direction, lifter1Encoder.getPosition(), lifter2Encoder.getPosition());
        if(direction != 0)
        {
            //syncMotors(speed * direction, lifter1Encoder.getPosition(), lifter2Encoder.getPosition());
            lifter1.set(speed * direction*encoder1Scale);
            lifter2.set(speed * direction*encoder2Scale);
        }
        else {
            liftersStop();
        }
    }

    public void lift() {
        syncMotors(speed, lifter1Encoder.getPosition(), lifter2Encoder.getPosition());
        lifter1.set(speed*encoder1Scale);
        lifter2.set(speed*encoder2Scale);
    }

    public void lower() {
        syncMotors(-speed, lifter1Encoder.getPosition(), lifter2Encoder.getPosition());
        lifter1.set(-speed*encoder1Scale);
        lifter2.set(-speed*encoder2Scale);
    }
    public void liftersStop() {
        lifter1.stopMotor();
        lifter2.stopMotor();
    }
    public void liftersReset() {
        lifter1Encoder.setPosition(4000);
        lifter2Encoder.setPosition(4000);

    }


    public LiftSubsystem() {
        lifter2Encoder = lifter2.getEncoder();
        lifter1Encoder = lifter1.getEncoder();
    }
 

    public void syncMotors(double velocity, double Encoder1, double Encoder2) {
        difference = Encoder1 - Encoder2;
        if (velocity > 0) {
            if (difference > 0) {
                encoder1Scale = Math.max((maxDeviation - Math.abs(difference))/(maxDeviation), 0);
                encoder2Scale = 1;
            }
            else if(difference < 0) {
                encoder1Scale = 1;
                encoder2Scale = Math.max((maxDeviation - Math.abs(difference))/(maxDeviation), 0);
            }
            else
            {
                encoder1Scale = 1;
                encoder2Scale = 1;
            }
        }
        else if(velocity < 0) {
            if (difference > 0) {
                encoder1Scale = 1;
                encoder2Scale = Math.max((maxDeviation - Math.abs(difference))/(maxDeviation), 0);
            }
            else if(difference < 0) {
                encoder1Scale = Math.max((maxDeviation - Math.abs(difference))/(maxDeviation), 0);
                encoder2Scale = 1;
            }
            else
            {
                encoder1Scale = 1;
                encoder2Scale = 1;
            }
        }
    }

    @Override
    public void periodic() {
        encoder1Position = lifter1Encoder.getPosition();
        encoder2Position = lifter2Encoder.getPosition();
        SmartDashboard.putNumber("Lifter 1 Encoder", encoder1Position);
        SmartDashboard.putNumber("Lifter 2 Encoder", encoder2Position);
        SmartDashboard.putNumber("Encoder1", encoder1Scale);
        SmartDashboard.putNumber("Encoder2", encoder2Scale);
        SmartDashboard.putNumber("Lift Difference", (maxDeviation - (encoder1Position - encoder2Position))/maxDeviation);
    }


}
