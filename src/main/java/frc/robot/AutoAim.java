// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
//hi people 
/** Add your docs here. */

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;

import java.text.FieldPosition;
import java.util.Optional;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.lib.commands.SwerveAngle;
import frc.robot.subsystems.ShooterHood;

public class AutoAim {

public static Pose2d blueHubPosition = new Pose2d( 4.626, 4.035, Rotation2d.fromDegrees(0)); 
public static Pose2d redHubPosition = new Pose2d( 11.915, 4.035, Rotation2d.fromDegrees(0)); 

public AutoAim() {NamedCommands.registerCommand("autoAimSpecialist", autoaimspecialist());}


public static Command autoAimSwerve(Supplier<Double> forward, Supplier<Double> leftright)
{
    return new ConditionalCommand
    (
      new ConditionalCommand(
        new SwerveAngle(RobotContainer.getRobot().swerve,
            forward, leftright, () -> angleToHub(redHubPosition)),
        new SwerveAngle(RobotContainer.getRobot().swerve,
          forward, leftright, () -> Rotation2d.kZero),
        () -> redAllianceZone()), 
      new ConditionalCommand(
          new SwerveAngle(RobotContainer.getRobot().swerve,
            forward, leftright, () -> angleToHub(blueHubPosition)), 
          new SwerveAngle(RobotContainer.getRobot().swerve,
            forward, leftright, () -> Rotation2d.k180deg),
          () -> blueAllianceZone()),
          () -> isRed());
          //change later for the rotation2d.KCW_90deg to rotation2d.KCCW_90deg
          //KCCW is counterclockwise KCW is clockwise
}

public static Command autoaimspecialist()
{
  return new ConditionalCommand
  (
    new ConditionalCommand(
    shootingLerp(),
    passingLerp(),
    () -> redAllianceZone()),
    new ConditionalCommand(
    shootingLerp(),
    passingLerp(),
    () -> blueAllianceZone()),
    () -> isRed());





}

public static boolean isRed()
{
   Optional<Alliance> a= DriverStation.getAlliance();

   if(a.isEmpty())
    return true;

    else
        return a.get() == Alliance.Red;
}
 
    public static boolean redAllianceZone()
    {
        return RobotContainer.getRobot().swerve.swerveOdometry.getEstimatedPosition().getMeasureX().in(Meter) > 12;
    }

    public static boolean blueAllianceZone()
    {
        return RobotContainer.getRobot().swerve.swerveOdometry.getEstimatedPosition().getMeasureX().in(Meter) < 4.5;
    }

    public static Rotation2d angleToHub(Pose2d hub)
    {

        return RobotContainer.getRobot().swerve.swerveOdometry.getEstimatedPosition().minus(hub).getTranslation().getAngle().plus(Rotation2d.k180deg);
    }

    public static double distanceToPass()
    {
        double x = RobotContainer.getRobot().swerve.swerveOdometry.getEstimatedPosition().getMeasureX().in(Meters);
        if(isRed())
        {
          return Constants.FIELD_LENGTH - x;
        }
        else
        {
          return x;
        }
    }


    public static double distanceToHub()
    {
      Pose2d hub;
      if (isRed()) 
      {
        hub = redHubPosition;
      }
      else
      {
        hub = blueHubPosition;
      }
        return RobotContainer.getRobot().swerve.swerveOdometry.getEstimatedPosition().minus(hub).getTranslation().getNorm();


    }

    public static Command shootingLerp()
    {
      return new ParallelCommandGroup
      (
        RobotContainer.getRobot().shooter.setShootVelocityFromDistance(),
        RobotContainer.getRobot().shooterhood.setHoodAngle()
      );
    }

    public static Command passingLerp()
    {
      return new ParallelCommandGroup
      (
        RobotContainer.getRobot().shooter.setPassVelocityFromDistance(),
        RobotContainer.getRobot().shooterhood.setHoodAnglePass()
      );
    }
}

