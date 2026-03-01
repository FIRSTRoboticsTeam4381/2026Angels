// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.commands.TeleopSwerve;
import frc.lib.controls.JoystickUtils;
import frc.lib.subsystems.PhotonCam;
import frc.robot.commands.Autos;
import frc.robot.subsystems.Agitator;
import frc.robot.subsystems.Hang;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.Intaker;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterHood;
import frc.robot.subsystems.Swerve;

@Logged
public class RobotContainer {
  
  // Controllers
  public final CommandXboxController driver = new CommandXboxController(0);
  public final CommandXboxController specialist = new CommandXboxController(1);
  
  // If you are using a button board, uncomment these and comment out specialist above
  // You may also want to adjust the un-zero'd joystick check in lib/controls/JoystickUtils.java
  public CommandGenericHID buttonBoard1 = new CommandGenericHID(2);
  public CommandGenericHID buttonBoard2 = new CommandGenericHID(3);

  //Auto Chooser
  SendableChooser<Autos.PreviewAuto> autoChooser = new SendableChooser<>();

  // Subsystems
  public final Swerve swerve = new Swerve();
  public final ShooterHood shooterhood = new ShooterHood();
  public final Shooter shooter = new Shooter();
  public final Agitator agitator = new Agitator();
  public final Intaker intaker = new Intaker();
  public final IntakePivot intakePivot = new IntakePivot();
  public final Hang hang = new Hang();
  // TODO set camera names, coordinates, and angles relative to the robot's center
  public final PhotonCam camA = new PhotonCam("FL_Camera", new Transform3d(new Translation3d(Units.inchesToMeters(11.33), Units.inchesToMeters(12.9),  Units.inchesToMeters(20.215)),
   new Rotation3d(0, Units.degreesToRadians(-20), Units.degreesToRadians(-30))));
  public final PhotonCam camB = new PhotonCam("FR_Camera", new Transform3d(new Translation3d(Units.inchesToMeters(11.33), Units.inchesToMeters(-12.9),  Units.inchesToMeters(20.215)),
   new Rotation3d(0, Units.degreesToRadians(-20), Units.degreesToRadians(30))));
  public final PhotonCam camC = new PhotonCam("BL_Camera", new Transform3d(new Translation3d(Units.inchesToMeters(-12.52), Units.inchesToMeters(12.56),  Units.inchesToMeters(10.535)),
   new Rotation3d(0, Units.degreesToRadians(-5), Units.degreesToRadians(45+180))));
  public final PhotonCam camD = new PhotonCam("BR_Camera", new Transform3d(new Translation3d(Units.inchesToMeters(-12.52), Units.inchesToMeters(-12.56),  Units.inchesToMeters(10.535)),
   new Rotation3d(0, Units.degreesToRadians(-5), Units.degreesToRadians(-45-180))));
  //forwardback, leftright, elevation

  // Constructor: set up the robot! 
  public RobotContainer() {
    robotReference = this;
    // Set default commands here
    NamedCommands.registerCommand("autoAimShoot", AutoAim.autoaimspecialist());



    // Set up autonomous picker
    // Add any autos you want to be able to select below
    autoChooser.setDefaultOption("None", Autos.none());
    autoChooser.addOption("Test", Autos.testAuto());
    autoChooser.addOption("Left Score", Autos.leftScoreAuto());
    autoChooser.addOption("Right Score", Autos.rightScoreAuto());
    autoChooser.addOption("Middle Depot", Autos.middleScoreAuto());
    autoChooser.addOption("Middle Depot (Testing Edition)", Autos.middleScoreTest());
    
    // Add auto controls to the dashboard
    SmartDashboard.putData("Choose Auto:", autoChooser);
    SmartDashboard.putData(CommandScheduler.getInstance());
    autoChooser.onChange((listener) -> {
      if(listener!=null)
        listener.showPreview();
      });
    SmartDashboard.putNumber("Start Delay",0);

    
    // Configure button bindings
    configureBindings();
  }

  private void configureBindings() {
    // Button to re-zero swerve drive
    driver.back()
      .onTrue(new InstantCommand(() -> swerve.zeroYaw()).ignoringDisable(true));
    
    // Default teleop swerve command
    swerve.setDefaultCommand(new TeleopSwerve(swerve, 
            driver::getLeftY,
            driver::getLeftX,
            driver::getRightX));

    // Example "slow mode" with 60% modififier on right bumper
    driver.rightBumper().whileTrue(
        new TeleopSwerve(swerve, 
            driver::getLeftY,
            driver::getLeftX,
            driver::getRightX,
            0.6));

    // Set the wheels to an "X" shape to make the robot more difficult to push
    driver.x().whileTrue(swerve.brake());

    // Button to revert to robot-oriented drive in an emergency
    driver.start().toggleOnTrue(
      new TeleopSwerve(swerve, 
            driver::getLeftY,
            driver::getLeftX,
            driver::getRightX,
            1.0,
            true,
            false,
            false));

    // Button to cancel running actions
    // TODO Your Controls Here!
    specialist.back().onTrue(new InstantCommand(() -> CommandScheduler.getInstance().cancelAll()));
    specialist.rightTrigger().toggleOnTrue(AutoAim.autoaimspecialist());
    specialist.leftTrigger().toggleOnTrue(intaker.intake());
    specialist.leftBumper().toggleOnTrue(intaker.outake());
    specialist.povUp().onTrue(intakePivot.up());
    specialist.povDown().onTrue(intakePivot.down());
    specialist.povRight().onTrue(intakePivot.middle());
    specialist.rightBumper().whileTrue(agitator.agitatorFunnelMove());
    specialist.axisMagnitudeGreaterThan(1 , 0.1).onTrue(hang.manualControl(() -> JoystickUtils.interpolateNow(specialist.getLeftY(), 0.1)));
    //specialist.povDownLeft().onTrue(intakePivot.halfopen());

    //Button board controls
    buttonBoard2.axisMagnitudeGreaterThan(0 , 0.1).onTrue(hang.manualControl(() ->JoystickUtils.interpolateNow(buttonBoard2.getRawAxis(0), 0.1)));
    buttonBoard1.button(1).whileTrue(intaker.intake());
    buttonBoard1.button(2).onTrue(intakePivot.down());
    buttonBoard1.button(2).onFalse(intakePivot.up());
    buttonBoard1.button(3).whileTrue(agitator.agitatorFunnelMove());
    buttonBoard1.button(4).whileTrue(AutoAim.autoaimspecialist());
    buttonBoard2.button(1).toggleOnTrue( new ParallelCommandGroup(
    shooterhood.setHoodAngle(() -> (buttonBoard1.getRawAxis(1) + 1) / 2 * 0.228 + 0.1),
    shooter.setVelocity(() -> (buttonBoard1.getRawAxis(0) + 1) / 2 * 6784)));
    buttonBoard2.button(3).whileTrue(intaker.outake());
    buttonBoard2.button(2).whileTrue(agitator.agitatorFunnelMoveReverse());
    // haven't done hang yet




    SmartDashboard.putNumber("autoAim/ManualShootPower", 0);
    SmartDashboard.putNumber("autoAim/ManualHoodAngle", 0);

    SmartDashboard.putData("autoAim/Manual", new ParallelCommandGroup
    (
      shooterhood.setHoodAngle(() -> (SmartDashboard.getNumber("autoAim/ManualHoodAngle", 0))),
      shooter.setVelocity(() -> (SmartDashboard.getNumber("autoAim/ManualShootPower", 0)))
    ));
            
    driver.rightTrigger().whileTrue(AutoAim.autoAimSwerve(driver::getLeftY,  driver::getLeftX));

 
           


  }

  public Command getAutonomousCommand() {
    double startDelay=SmartDashboard.getNumber("Start Delay", 0);
    return new SequentialCommandGroup( 
      new WaitCommand(startDelay), 
      new ScheduleCommand(autoChooser.getSelected().auto)
    ); 
  }


  // Static reference to the robot class
  // Use getRobot() to get the robot object
  private static RobotContainer robotReference;

  /**
   * Get a reference to the RobotContainer object in use
   * @return the active RobotContainer object
   */
  public static RobotContainer getRobot()
  {
    return robotReference;
  }




}
