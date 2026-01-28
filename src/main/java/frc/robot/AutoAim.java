// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
//hi people 
/** Add your docs here. */

import static edu.wpi.first.units.Units.Meter;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.lib.commands.SwerveAngle;

public class AutoAim {

public static Command autoAimSwerve(Supplier<Double> forward, Supplier<Double> leftright)
{
    return new ConditionalCommand
    (() -> isRed(),
      new ConditionalCommand(() -> redAllianceZone(),
         null, 
         new SwerveAngle(RobotContainer.getRobot().swerve,
          forward, leftright, () -> Rotation2d.kCCW_90deg)), 
      new ConditionalCommand(() -> blueAllianceZone(),
         null, 
         new SwerveAngle(RobotContainer.getRobot().swerve,
          forward, leftright, () -> Rotation2d.kCW_90deg)))
          //change later for the rotation2d.KCW_90deg to rotation2d.KCCW_90deg
          //KCCW is counterclockwise KCW is clockwise
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
}
