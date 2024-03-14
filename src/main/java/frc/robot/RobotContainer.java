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

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.simulation.JoystickSim;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.CommandsContainer;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LiftSubsystem;
import frc.robot.subsystems.ShootingSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.utils.JoystickAnalogButton;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import com.choreo.lib.*;

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
  private final CommandsContainer m_CommandsContainer = new CommandsContainer();
  private UsbCamera camera1;
  private UsbCamera camera2;
  Field2d m_field = new Field2d();
  //command container class
  CommandsContainer commands = new CommandsContainer();
  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  Joystick m_driverJoystick = new Joystick(1);
  Joystick m_assistJoystick = new Joystick(2);
  private final SendableChooser<String> autoSelector = new SendableChooser();
  ChoreoTrajectory traj;
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
    Trigger back = new JoystickButton(m_driverController, XboxController.Button.kBack.value);
    
  //RADIOMASTER ZORRO CONTROLLER IDENTIFICATION
    Trigger leftBumperPush = new JoystickButton(m_driverJoystick, 1);
    Trigger rightBumperPush = new JoystickButton(m_driverJoystick, 2);
    Trigger leftBackPush = new JoystickButton(m_driverJoystick, 3);
    Trigger rightBackPush = new JoystickButton(m_driverJoystick, 4);
    Trigger leftPot = new JoystickButton(m_driverJoystick, 5);
    Trigger rightPot = new JoystickButton(m_driverJoystick, 6);
    Trigger axisButton1 = new JoystickButton(m_driverJoystick, 7);
    NetworkTable FMS = NetworkTableInstance.getDefault().getTable("FMSInfo");
    Trigger switch1 = new Trigger((() -> m_driverJoystick.getRawAxis(5) > 0.5));
    Trigger button10 = new Trigger((() -> m_driverJoystick.getRawAxis(6) > 0.5));
    Trigger testButton5 = new Trigger((() -> m_driverJoystick.getRawAxis(4) > 0.5));


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_LedSubsystem = new LEDSubsystem(FMS, m_IntakeSubsystem);
    traj = Choreo.getTrajectory("Trajectory");
    m_field.getObject("traj").setPoses(
      traj.getInitialPose(), traj.getFinalPose()
    );
    m_field.getObject("trajPoses").setPoses(
      traj.getPoses()
    );

    SmartDashboard.putData(m_field);

    // Configure the button bindings
    configureButtonBindings();
    
    //Autonomous options
    autoSelector.setDefaultOption("Amp on Left", "Amp on Left");
    autoSelector.setDefaultOption("Amp on Right", "Amp on Right");

    SmartDashboard.putData("Auto Mode", autoSelector);

    // Configure default commands
    m_robotDrive.setDefaultCommand(commands.defaultDriveCommand(m_robotDrive, m_driverJoystick));
      m_LiftSubsystem.setDefaultCommand(new RunCommand(
        () -> {
          if(m_driverController.getPOV() == 0) {
            m_LiftSubsystem.measuredControl(1);
          }
          else if(m_driverController.getPOV() == 180) {
            m_LiftSubsystem.measuredControl(-1);
          }
          else {
          m_LiftSubsystem.control(m_driverJoystick.getRawAxis(6));
          }
        },
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
    new JoystickButton(m_driverController, 3) 
        .whileTrue(new RunCommand(
            () -> m_robotDrive.zeroHeading(),
            m_robotDrive));
    // new JoystickButton(m_driverJoystick, 6)
    //     .whileTrue(new StartEndCommand(
    //       () -> m_LiftSubsystem.lift(),
    //       () -> m_LiftSubsystem.liftersStop(),
    //      m_LiftSubsystem));

    new JoystickButton(m_driverController, 0);
    new JoystickButton(m_driverJoystick, 2)
        .onTrue(                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         new ParallelCommandGroup(
          new FunctionalCommand(
            () -> {m_ShootingSubsystem.timerInit();},
            () -> m_ShootingSubsystem.shoot(), 
            (x) -> {
              m_ShootingSubsystem.stop();
              m_ShootingSubsystem.timerStop();
            }, 
            () -> {return (m_ShootingSubsystem.getTime() > 3.0);},
            m_ShootingSubsystem)
          .alongWith(
            new WaitCommand(2)
            .andThen(
              new FunctionalCommand(
                () -> {},
                () -> m_IntakeSubsystem.noteFeed(), 
                (x) -> m_IntakeSubsystem.noteFeedStop(), 
                () -> {return (m_ShootingSubsystem.getTime() > 3.0);},
                m_IntakeSubsystem)
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

    // switch1
    //     .whileTrue(new InstantCommand(
    //       () -> {m_CommandsContainer.fieldRelative = false;}
    //       ));
    new JoystickButton(m_driverJoystick, 3)
        .whileTrue(new StartEndCommand(
          () -> m_IntakeSubsystem.noteUntake(),
          () -> m_IntakeSubsystem.intakeStop(),
          m_IntakeSubsystem
         ));
    new JoystickButton(m_driverController, 1)
        .whileTrue(new StartEndCommand(
          () -> m_IntakeSubsystem.noteFeed(),
          () -> m_IntakeSubsystem.intakeStop()
         ));
    new JoystickButton(m_driverController, 4)
        .whileTrue(new RunCommand(
          () -> {m_robotDrive.setX();}
         ));
    new JoystickButton(m_assistJoystick, 2)
        .whileTrue(new StartEndCommand(
          () -> {m_LiftSubsystem.liftersReset();},
          () -> {}
         ));
    // new JoystickButton(m_driverController, 6)
    //     .onTrue(new InstantCommand(
    //       () -> {m_vision.lockNote();}
    //     ));
    new JoystickButton(m_driverController, 7)
        .whileTrue(new RunCommand(
          () -> m_LiftSubsystem.liftersReset()
        ));
    // button10
    //     .whileTrue(new InstantCommand(
    //       () -> {m_ShootingSubsystem.shootReverse();}
    //     ));
    //xbox controller b button slow shoot
    new JoystickButton(m_driverController, 2)
        .whileTrue(new FunctionalCommand(
          () -> {m_ShootingSubsystem.timerInit();}, 
          () -> {
            m_IntakeSubsystem.throttledIntake(0.12);
            m_ShootingSubsystem.throttledShooting(0.2);
          }, 
          (x) -> {
            m_IntakeSubsystem.intakeStop(); 
            m_ShootingSubsystem.stop();
          }, 
          () -> {return (m_ShootingSubsystem.shootSensor());}, 
          m_IntakeSubsystem, 
          m_ShootingSubsystem));
        }
    


    

    
    // new JoystickButton(m_driverJoystick, 7)
    //     .whileTrue(new RunCommand(
    //       () -> ,
    //      ));
    // uncomment when new commands/functions for controller are needed 
    //(the above commands are set for radiomaster zorro, not xbox controller)

  
public void checkFieldColor() {
  // The origin is always blue. When our alliance is red, X and Y need to be inverted
var alliance = DriverStation.getAlliance();
var invert = 1;
if (alliance.isPresent() && alliance.get() == Alliance.Red) {
    invert = -1;
}}
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
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(List.of(new Pose2d(0, 0, new Rotation2d(0)),new Pose2d(2.2, 0, new Rotation2d(0))
      ),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    
    Command swerveCommand = Choreo.choreoSwerveCommand(
        traj,
        m_robotDrive::getPose, // Functional interface to feed supplier

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        (ChassisSpeeds speeds) -> m_robotDrive.drive(
          speeds.vxMetersPerSecond, 
          speeds.vyMetersPerSecond, 
          speeds.omegaRadiansPerSecond, 
          false,
          true),
        true,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    InstantCommand setGyroRight = new InstantCommand(() -> m_robotDrive.setHeading(60));
    InstantCommand setGyroLeft = new InstantCommand(() -> m_robotDrive.setHeading(-60));
    InstantCommand setGyroRegular = new InstantCommand(() -> m_robotDrive.setHeading(0));
    // Run path following command, then stop at the end.

    String selectedAuto = autoSelector.getSelected();
    switch(selectedAuto){
      case "Amp on Left":
        return setGyroLeft
          .andThen(shoot)
          .andThen(new WaitCommand(8))
          .andThen(swerveCommand);
          // .andThen(setGyroRegular);
      case "Amp on Right":
        return setGyroRight
          .andThen(shoot)
          .andThen(new WaitCommand(8))
          .andThen(swerveCommand);
          // .andThen(setGyroRegular);
    }
    return swerveCommand.andThen(shoot).andThen();
  }

  // public void axisBoolean(Joystick control, int axis, Command action) {
  //   if (control.getRawAxis(axis) > 0.99) {
  //     action.execute();
  //   }
  // }

  ParallelCommandGroup shoot = new FunctionalCommand(
    () -> {m_ShootingSubsystem.timerInit();},
    () -> m_ShootingSubsystem.shoot(), 
    (x) -> {
      m_ShootingSubsystem.stop();
      m_ShootingSubsystem.timerStop();
    }, 
    () -> {return (m_ShootingSubsystem.getTime() > 3.0);},
    m_ShootingSubsystem)
  .alongWith(
    new WaitCommand(2)
    .andThen(
      new FunctionalCommand(
        () -> {},
        () -> m_IntakeSubsystem.noteFeed(), 
        (x) -> m_IntakeSubsystem.noteFeedStop(), 
        () -> {return (m_ShootingSubsystem.getTime() > 3.0);},
        m_IntakeSubsystem)
    )
  );








  


public void periodic () {
  
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