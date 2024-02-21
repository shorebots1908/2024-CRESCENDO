package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class LEDSubsystem extends SubsystemBase {
    private Spark ledStrip = new Spark(0);
    private double yellow = 0.69; 
    private double purple = 0.91; 
    private double green = 0.77; 
    private double red = 0.61;
    private double blue = 0.87;
    private double teamColor;

    
    public LEDSubsystem(NetworkTable FMS){
        if (FMS.getEntry("isRedAlliance").getBoolean(false)) {
            teamColor = 0.61;
        }
        else {
            teamColor = 0.87;
        }
    }
    public void setLEDColor(double pwmColorCode) {
        ledStrip.set(pwmColorCode);
    }
}
