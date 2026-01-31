// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.ejml.equation.IntegerSequence.For;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.AutoAim;
import frc.robot.CanIDs;



public class Shooter extends SubsystemBase {

/** Creates a new Shooter. */
public SparkFlex shooter1;
public SparkFlex shooter2;
public InterpolatingDoubleTreeMap table;

      public void setUp()
        {
          table.put(0.0, 0.0);
          table.put(1.0, 10.0);
          table.put(2.0, 30.0);
        }

  public Shooter() {
  shooter1 = new SparkFlex(CanIDs.SHOOTER_1_MOTOR_ID, MotorType.kBrushless); 
  shooter2 = new SparkFlex(CanIDs.SHOOTER_2_MOTOR_ID, MotorType.kBrushless);
  table = new InterpolatingDoubleTreeMap();
  setUp();

    SparkFlexConfig shooterConfig = new SparkFlexConfig()
    {{
        this.smartCurrentLimit(40);
    }};

     SparkFlexConfig shooter2Config = new SparkFlexConfig()
    {{
       apply(shooterConfig);
       follow(shooter1);
       inverted(true);
       //makes shooter 2 ther same as shooter 1
    }};


    shooter1.configure(shooterConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    shooter2.configure(shooter2Config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    this.setDefaultCommand(
      new FunctionalCommand(() -> shooter1.set(0), () -> {}, (killed) -> {}, () -> {return false;}, this));
  
  }

  @Override
  public void periodic()
  {
    // This method will be called once per scheduler run
  }




  public Command shooterShoot()
    {
      return new InstantCommand(()-> shooter1.set(0.5), this);
      //gives 0.5 current to the motor 
    }

  private void setVelocity(double velocity)
    {
        shooter1.getClosedLoopController().setSetpoint(velocity, ControlType.kVelocity);
        // keeps the speed around the same speed 
    }

  public double velocityFromDistance()
  {
    double distance = AutoAim.distanceToHub();
      return table.get(distance);
  }

  public Command setVelocityFromDistance()
      {
        return new InstantCommand(() -> setVelocity(velocityFromDistance())).repeatedly();
      }






}
