// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/* 
 * TODO: intake controls; deploying intake, indexed rotations
 * TODO: shooting controls; 
 * TODO: apriltags reading, better camera stuff
 * TODO: driving tuning
 * TODO: 2 camera pipelines, rings and apriltags tracking, camera for object detection + driver camera
 * TODO: path planning and path resolution
 */

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.simulation.JoystickSim;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LiftSubsystem;
import frc.robot.subsystems.ShootingSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.utils.JoystickAnalogButton;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;


import java.util.List;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final VisionSubsystem m_vision = new VisionSubsystem(m_robotDrive);
  private final LEDSubsystem m_LedSubsystem;
  private final LiftSubsystem m_LiftSubsystem = new LiftSubsystem();
  private final ShootingSubsystem m_ShootingSubsystem = new ShootingSubsystem();
  private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  Joystick m_driverJoystick = new Joystick(1);

  // The controller buttons being declared, can be used for setting different buttons to certain commands and/or functions
  //XBOX CONTROLLER IDENTIFICATION
    Trigger yButton = new JoystickButton(m_driverController, XboxController.Button.kY.value);
    Trigger xButton = new JoystickButton(m_driverController, XboxController.Button.kX.value);
    Trigger aButton = new JoystickButton(m_driverController, XboxController.Button.kA.value);
    Trigger bButton = new JoystickButton(m_driverController, XboxController.Button.kB.value);
    Trigger leftBumper = new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value);
    Trigger rightBumper = new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value);
    Trigger rightStickPush = new JoystickButton(m_driverController, XboxController.Button.kRightStick.value);
    Trigger leftStickPush = new JoystickButton(m_driverController, XboxController.Button.kLeftStick.value);
  //RADIOMASTER ZORRO CONTROLLER IDENTIFICATION
    Trigger leftBumperPush = new JoystickButton(m_driverJoystick, 1);
    Trigger rightBumperPush = new JoystickButton(m_driverJoystick, 2);
    Trigger leftBackPush = new JoystickButton(m_driverJoystick, 3);
    Trigger rightBackPush = new JoystickButton(m_driverJoystick, 4);
    Trigger leftPot = new JoystickButton(m_driverJoystick, 5);
    Trigger rightPot = new JoystickButton(m_driverJoystick, 6);
    Trigger axisButton1 = new JoystickButton(m_driverJoystick, 7);
    NetworkTable FMS = NetworkTableInstance.getDefault().getTable("FMSInfo");
    Trigger test = new Trigger((() -> m_driverJoystick.getRawAxis(7) > 0.5));
    Trigger button10 = new Trigger((() -> m_driverJoystick.getRawAxis(10) > 0.5));
    Trigger testButton5 = new Trigger((() -> m_driverJoystick.getRawAxis(7) < 0.5));


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_LedSubsystem = new LEDSubsystem(FMS);
    
    // Configure the button bindings
    configureButtonBindings();
    
    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        // new RunCommand(
        //     () -> m_robotDrive.drive(
        //         modifyAxis(m_driverController.getLeftY()),
        //         modifyAxis(m_driverController.getLeftX()),
        //         modifyAxis(m_driverController.getRightX()),
        //         true, true),
        //     m_robotDrive));
        new RunCommand(
            () -> m_robotDrive.drive(
                -0.15 *  modifyAxis(m_driverJoystick.getRawAxis(2)),
                0.15 * modifyAxis(m_driverJoystick.getRawAxis(3)),
                0.15 *  modifyAxis(m_driverJoystick.getRawAxis(0)),
                true, true),
            m_robotDrive));
      m_LiftSubsystem.setDefaultCommand(new RunCommand(
        () -> m_LiftSubsystem.control(m_driverJoystick.getRawAxis(6)),
        m_LiftSubsystem));
  }



  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_driverJoystick, 5)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));
    // new JoystickButton(m_driverJoystick, 6)
    //     .whileTrue(new StartEndCommand(
    //       () -> m_LiftSubsystem.lift(),
    //       () -> m_LiftSubsystem.liftersStop(),
    //      m_LiftSubsystem));
    new JoystickButton(m_driverJoystick, 2)
        .whileTrue(                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         new ParallelCommandGroup(
          new StartEndCommand(() -> m_ShootingSubsystem.shoot(), () -> m_ShootingSubsystem.stop(), m_ShootingSubsystem)
          .alongWith(
            new WaitCommand(0.25)
            .andThen(
              new StartEndCommand(() -> m_IntakeSubsystem.noteFeed(), () -> m_IntakeSubsystem.noteFeedStop(), m_IntakeSubsystem)
            )
          )
        ));
    new JoystickButton(m_driverJoystick, 1)
        .whileTrue(new FunctionalCommand(
          () ->{},
          () -> m_IntakeSubsystem.noteIntake(),
          (x) -> m_IntakeSubsystem.intakeStop(), 
          () -> m_IntakeSubsystem.intakeSensor()
         ));
    new JoystickButton(m_driverJoystick, 3)
        .whileTrue(new StartEndCommand(
          () -> m_IntakeSubsystem.noteUntake(),
          () -> m_IntakeSubsystem.intakeStop(),
          m_IntakeSubsystem
         ));
    new JoystickButton(m_driverJoystick, 4)
        .whileTrue(new StartEndCommand(
          () -> m_IntakeSubsystem.noteFeed(),
          () -> m_IntakeSubsystem.intakeStop()
         ));
         //Trigger defined elsewhere, no need for extra stuff here
    test
        .whileTrue(new StartEndCommand(
          () -> m_IntakeSubsystem.noteFeed(),
          () -> m_IntakeSubsystem.intakeStop()
        ));
    

    
    

    
    // new JoystickButton(m_driverJoystick, 7)
    //     .whileTrue(new RunCommand(
    //       () -> ,
    //      ));
    // uncomment when new commands/functions for controller are needed (the above commands are set for radiomaster zorro, not xbox controller)

  }

  

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false, false));
  }

  public void axisBoolean(Joystick control, int axis, Command action) {
    if (control.getRawAxis(axis) > 0.99) {
      action.execute();
    }
  }






  //functions to smooth controller input
  private static double modifyAxis(double value) {
    // Deadband
    value = -MathUtil.applyDeadband(value, OIConstants.kDriveDeadband);
    
    // Square the axis, keep its sign.
    value = Math.copySign(value * value, value);
    
    return value;

  }
  
}