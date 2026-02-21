// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.commands.SparkPosition;
import frc.lib.commands.SparkPositionProfiled;
import frc.lib.commands.SparkSysIDTest;
import frc.robot.AutoAim;
import frc.robot.CanIDs;
@Logged
public class ShooterHood extends SubsystemBase 
{
  public SparkMax hoodedmotor1;
  public SparkMax hoodedmotor2;
  
  public InterpolatingDoubleTreeMap hoodShootTable;
  public InterpolatingDoubleTreeMap hoodPassTable;

  /** Creates a new CarHood. */
  public ShooterHood() 
  {
    hoodPassTable = new InterpolatingDoubleTreeMap();
    hoodShootTable = new InterpolatingDoubleTreeMap();
    setUp();


    hoodedmotor1 = new SparkMax(CanIDs.HOODED_MOTOR_ID, MotorType.kBrushed);
    SparkMaxConfig hoodedmotor1Config = new SparkMaxConfig(){{
      smartCurrentLimit(10);
      closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
      softLimit.forwardSoftLimit(0.328).reverseSoftLimit(0.1)
      .forwardSoftLimitEnabled(true).reverseSoftLimitEnabled(true);
      this.signals.absoluteEncoderPositionAlwaysOn(true);
      this.signals.isAtSetpointAlwaysOn(true);
      this.signals.maxMotionSetpointPositionAlwaysOn(true);
      this.signals.maxMotionSetpointVelocityAlwaysOn(true);
      this.signals.setSetpointAlwaysOn(true);
      this.idleMode(IdleMode.kBrake);
      this.inverted(true);
      closedLoop.feedForward.sv(0.76127, 21.166);
      closedLoop.p(5.3492);
    }};
     hoodedmotor1.configure(hoodedmotor1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    
    hoodedmotor2 = new SparkMax(CanIDs.HOODED_MOTOR_ID_2, MotorType.kBrushed);
    SparkMaxConfig hoodedmotor2Config = new SparkMaxConfig(){{
      smartCurrentLimit(10);
      follow(hoodedmotor1, true);
       this.idleMode(IdleMode.kBrake);
    }};

     hoodedmotor2.configure(hoodedmotor2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);



   

     this.setDefaultCommand (hoodtoposition(0.1, 0.01));

    SmartDashboard.putData("Subsystem/ShooterHood",this);


  SmartDashboard.putData("SysID/ShooterHood",
      new SparkSysIDTest(hoodedmotor1, this, 1, 0.12, 0.32, hoodedmotor1.getAbsoluteEncoder()::getPosition));

    
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooterhood/targetposition", hoodedmotor1.getClosedLoopController().getSetpoint());
  }

public Command hoodtoposition(double target, double range) //puts the hood in a position
{ 
  return new SparkPosition(hoodedmotor1, target, range, this);
}

public Command joystickcontrol(Supplier<Double> joystickMove)
  {
    return 
      new InstantCommand(()-> hoodedmotor1.set(joystickMove.get()),this).repeatedly()
    ;
  }
    private void setAngle(double Angle)
    {
        hoodedmotor1.getClosedLoopController().setSetpoint(Angle, ControlType.kPosition);
        // keeps the speed around the same speed 
    }

 

  public void setUp()
    {
    hoodShootTable.put(3.05, 0.19975);
    hoodShootTable.put(1.28, 0.107125);
    hoodShootTable.put(5.21, 0.298378);
    hoodShootTable.put(5.19, 0.256472);
    hoodShootTable.put(3.43, 0.219386);
    hoodShootTable.put(3.49, 0.222079);
    //key=distance, value=angle
    hoodPassTable.put(0.0, 0.1);
    hoodPassTable.put(1.0, 0.2);
    hoodPassTable.put(2.0, 0.3);
    }

    public double hoodAngle()
    {
      double Distance = AutoAim.distanceToHub();
          return hoodShootTable.get(Distance);
    }

    public Command setHoodAngle()
    {
        return new InstantCommand(() -> setAngle(hoodAngle()),this).repeatedly().withName("setHoodAngle");
    }



    public double setHoodPass()
    {
      double Distance = AutoAim.distanceToPass(); //change distanceToHUb to something else 
          return hoodPassTable.get(Distance);
    }

    public Command setHoodAnglePass()
    {
        return new InstantCommand(() -> setAngle(setHoodPass()),this).repeatedly().withName("setHoodAnglePass");
    }


    public Command setHoodAngle(Supplier<Double> supplier)
      {
        return new InstantCommand(() -> setAngle(supplier.get()), this).repeatedly().withName("setHoodAngle");
      }
}
