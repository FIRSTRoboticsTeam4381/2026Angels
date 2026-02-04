// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.commands.SparkPositionProfiled;
import frc.robot.AutoAim;
import frc.robot.CanIDs;

public class ShooterHood extends SubsystemBase 
{
  public SparkFlex hoodedmotor;

  public InterpolatingDoubleTreeMap hoodShootTable;
  public InterpolatingDoubleTreeMap hoodPassTable;

  /** Creates a new CarHood. */
  public ShooterHood() 
  {
    hoodPassTable = new InterpolatingDoubleTreeMap();
    hoodShootTable = new InterpolatingDoubleTreeMap();
    setUp();


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
    private void setAngle(double Angle)
    {
        hoodedmotor.getClosedLoopController().setSetpoint(Angle, ControlType.kPosition);
        // keeps the speed around the same speed 
    }

 

  public void setUp()
    {
    hoodShootTable.put(0.0, 0.0);
    hoodShootTable.put(1.0, 10.0);
    hoodShootTable.put(2.0, 30.0);
    }

     public void stillDontKnowName()
    {
    hoodPassTable.put(0.0, 0.0);
    hoodPassTable.put(1.0, 10.0);
    hoodPassTable.put(2.0, 30.0);
    }

    public double hoodAngle()
    {
      double Distance = AutoAim.distanceToHub();
          return hoodShootTable.get(Distance);
    }

    public Command setHoodAngle()
    {
        return new InstantCommand(() -> setAngle(hoodAngle()),this).repeatedly();
    }



    public double setHoodPass()
    {
      double Distance = AutoAim.distanceToHub(); //change distanceToHub to something else 
          return hoodPassTable.get(Distance);
    }

    public Command setHoodAnglePass()
    {
        return new InstantCommand(() -> setAngle(hoodAngle()),this).repeatedly();
    }
}
