package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

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
    private RelativeEncoder lifter2Encoder = lifter2.getEncoder();
    private RelativeEncoder lifter1Encoder = lifter1.getEncoder();
    private double difference;
    private double encoder1Scale = 1;
    private double encoder2Scale = 1;
    private double maxDeviation = 4;
    private double speed = 0.80;

    public void control(double direction) {
        if(direction != 0)
        {
            syncMotors(speed * direction, lifter1Encoder.getPosition(), lifter2Encoder.getPosition());
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
        lifter1Encoder.setPosition(0);
        lifter2Encoder.setPosition(0);
    }

 

    public void syncMotors(double speed, double Encoder1, double Encoder2) {
        difference = Encoder1 - Encoder2;
        if (speed > 0) {
            if (difference > 0) {
                encoder1Scale = (maxDeviation-difference)/(maxDeviation);
                encoder2Scale = 1;
            }
            else if(difference < 0) {
                encoder1Scale = 1;
                encoder2Scale = (maxDeviation+difference)/(maxDeviation);
            }
            else
            {
                encoder1Scale = 1;
                encoder2Scale = 1;
            }
        }
        else if(speed < 0) {
            if (difference < 0) {
                encoder1Scale = (maxDeviation-difference)/(maxDeviation);
                encoder2Scale = 1;
            }
            else if(difference > 0) {
                encoder1Scale = 1;
                encoder2Scale = (maxDeviation+difference)/(maxDeviation);
            }
            else
            {
                encoder1Scale = 1;
                encoder2Scale = 1;
            }
        }
    }


}
