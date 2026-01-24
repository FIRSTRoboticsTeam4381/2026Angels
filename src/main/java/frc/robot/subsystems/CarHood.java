// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.commands.SparkPositionProfiled;

public class CarHood extends SubsystemBase 
{
  public SparkFlex hoodedmotor;




  /** Creates a new CarHood. */
  public CarHood() 
  {
    hoodedmotor = new SparkFlex(1, MotorType.kBrushless);
    SparkFlexConfig hoodedmotorConfig = new SparkFlexConfig();



    hoodedmotor.configure(hoodedmotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

     this.setDefaultCommand
(
new FunctionalCommand
(
() -> hoodedmotor.set(0),
() -> {},
(killed) -> {},
() -> {return false;},
this)
);


}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

public Command hoodtoposition(double target, double range) 
{
  return new SparkPositionProfiled(hoodedmotor, target, range, this);
}

public Command joystickcontrol(Supplier<Double> joystickMove)
  {
    return 
      new InstantCommand(()-> hoodedmotor.set(joystickMove.get()),this).repeatedly()
    ;
  }

}
