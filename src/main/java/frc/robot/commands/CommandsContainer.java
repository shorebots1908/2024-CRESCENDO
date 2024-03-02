package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LiftSubsystem;
import frc.robot.subsystems.ShootingSubsystem;
public class CommandsContainer {
    public Command defaultDriveCommand(DriveSubsystem drive, Joystick controller) {
        return new RunCommand(
            () -> drive.drive(
                -0.15 *  modifyAxis(controller.getRawAxis(2)),
                0.15 * modifyAxis(controller.getRawAxis(3)),
                0.15 *  modifyAxis(controller.getRawAxis(0)),
                true, true),
            drive);
        
    }


    public Command defaultLiftCommand(LiftSubsystem lift, Joystick controller)  {
        return new RunCommand(
            () -> lift.control(controller.getRawAxis(6)));
    }
    

    
    private static double modifyAxis(double value) {
    // Deadband
    value = -MathUtil.applyDeadband(value, OIConstants.kDriveDeadband);
    
    // Square the axis, keep its sign.
    value = Math.copySign(value * value, value);
    
    return value;

  }
}
