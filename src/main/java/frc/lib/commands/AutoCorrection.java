// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

import static frc.robot.Constants.Swerve.*;

public class AutoCorrection extends Command 
{

    public static final double TRANSLATION_TOLERANCE_M = 0.035;
    public static final double ROTATION_TOLERANCE_DEG = 1;
    
    
    public Swerve swerve;
    public static Pose2d target;
    public PIDController x;
    public PIDController y;
    public PIDController r;
    public AutoCorrection(Swerve s){

        swerve = s;
        x = new PIDController(TRANSLATION_PID.kP, TRANSLATION_PID.kI, TRANSLATION_PID.kD);
        y = new PIDController(TRANSLATION_PID.kP, TRANSLATION_PID.kI, TRANSLATION_PID.kD);
        r = new PIDController(ROTATION_PID.kP, ROTATION_PID.kI, ROTATION_PID.kD);
        r.enableContinuousInput(180,-180);
        addRequirements(swerve);
        
    }
    // Called when the command is initially scheduled
    @Override
    public void initialize(){
        swerve.field.getObject("AutoCorrection Target").setPose(target);
        
        x.setSetpoint(target.getX());
        y.setSetpoint(target.getY());
        r.setSetpoint(target.getRotation().getDegrees());

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute(){
       swerve.drive(new Translation2d(-getXPower(),-getYPower()), getRPower(), true, false, false);

       SmartDashboard.putNumber("Autocorrect/dist",((swerve.getPose().getTranslation().getDistance(target.getTranslation())) ));
       SmartDashboard.putNumber("Autocorrect/angle", (swerve.getPose().getRotation().minus(target.getRotation())).getDegrees());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted){
        swerve.drive(new Translation2d(0,0), 0, true, true, false);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished(){
        return ((swerve.getPose().getTranslation().getDistance(target.getTranslation())) <= TRANSLATION_TOLERANCE_M && 
        Math.abs((swerve.getPose().getRotation().minus(target.getRotation())).getDegrees()) <= ROTATION_TOLERANCE_DEG);
    }

    public double getXPower(){
        return x.calculate(swerve.getPose().getX());
    }

    public double getYPower(){
        return y.calculate(swerve.getPose().getY());
    }

    public double getRPower(){
        return r.calculate(swerve.getPose().getRotation().getDegrees());
    }

}
