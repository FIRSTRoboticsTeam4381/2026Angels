// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.commands.SparkPositionProfiled;
import frc.robot.CanIDs;

public class CarHood extends SubsystemBase 
{
  public SparkFlex hoodedmotor;




  /** Creates a new CarHood. */
  public CarHood() 
  {
    hoodedmotor = new SparkFlex(CanIDs.HOODED_MOTOR_MOTOR_ID, MotorType.kBrushless);
    SparkFlexConfig hoodedmotorConfig = new SparkFlexConfig(){{
      smartCurrentLimit(40);
      closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
      softLimit.forwardSoftLimit(100).reverseSoftLimit(0);
    }};



    hoodedmotor.configure(hoodedmotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

     this.setDefaultCommand
(
new FunctionalCommand
( //default command
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

public Command hoodtoposition(double target, double range) //puts the hood in a position
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
