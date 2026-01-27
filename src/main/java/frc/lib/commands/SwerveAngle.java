package frc.lib.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.controls.JoystickUtils;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class SwerveAngle extends Command{
    
    private double rotation;
    private Translation2d translation;
    
    private boolean openLoop = true;
    private boolean fieldOriented = true;
    private boolean flipOnAlliance = true;

    private boolean interpolateTranslation = true;
    private boolean interpolateRotation = true;
    private PIDController pid = new PIDController(Constants.Swerve.ROTATION_PID.kP, Constants.Swerve.ROTATION_PID.kI, Constants.Swerve.ROTATION_PID.kD);
    


    private Swerve s_Swerve;
    private Supplier<Double> forward;
    private Supplier<Double> leftright;
    private Supplier<Rotation2d> rotate;
    
    // Configurable max speed percent per SwerveAngle object
    private double speedModifier = 1.0;

    /**
     * Command for driver control of a swerve drive. 
     * Can also be used in other ways by passing in different suppliers.
     * @param s_Swerve Swerve drive object to control
     * @param forward Supplier for forward/back motion (likely a joystick)
     * @param leftright Supplier for left/right motion (likely a joystick)
     * @param rotate Supplier for rotation control (likely a joystick)
     */
    public SwerveAngle(Swerve s_Swerve, Supplier<Double> forward, Supplier<Double> leftright, Supplier<Rotation2d> rotate){
        this(s_Swerve, forward, leftright, rotate, 1.0);
    }

    
    /**
     * Command for driver control of a swerve drive. 
     * Can also be used in other ways by passing in different suppliers.
     * @param s_Swerve Swerve drive object to control
     * @param forward Supplier for forward/back motion (likely a joystick)
     * @param leftright Supplier for left/right motion (likely a joystick)
     * @param rotate Supplier for rotation control (likely a joystick)
     * @param speedModifier Percent modifier on speed for this SwerveAngle object, defaults to 1.0
     */
    public SwerveAngle(Swerve s_Swerve, Supplier<Double> forward, Supplier<Double> leftright, Supplier<Rotation2d> rotate, double speedModifier){
        this(s_Swerve, forward, leftright, rotate, speedModifier, true, true, true);
    }


    /**
     * Command for driver control of a swerve drive. 
     * Can also be used in other ways by passing in different suppliers.
     * @param s_Swerve Swerve drive object to control
     * @param forward Supplier for forward/back motion (likely a joystick)
     * @param leftright Supplier for left/right motion (likely a joystick)
     * @param rotate Supplier for rotation control (likely a joystick)
     * @param speedModifier Percent modifier on speed for this SwerveAngle object, defaults to 1.0
     * @param openLoop Whether to run drive motors in open loop or closed loop mode. Default=true
     * @param fieldOriented Whether to treat the X and Y inputs as field-oriented (true) or 
     *  robot-oriented (false). Default=true
     * @param flipOnAlliance Whether to flip the field-oriented drive based on which alliance the robot
     *  is on. Default=true
     */
    public SwerveAngle(Swerve s_Swerve, Supplier<Double> forward, Supplier<Double> leftright, Supplier<Rotation2d> rotate, 
        double speedModifier, boolean openLoop, boolean fieldOriented, boolean flipOnAlliance){
        this(s_Swerve, forward, leftright, rotate, speedModifier, openLoop, fieldOriented, flipOnAlliance, 
        true, true);
    }

    /**
     * Command for driver control of a swerve drive. 
     * Can also be used in other ways by passing in different suppliers.
     * @param s_Swerve Swerve drive object to control
     * @param forward Supplier for forward/back motion (likely a joystick)
     * @param leftright Supplier for left/right motion (likely a joystick)
     * @param rotate Supplier for rotation control (likely a joystick)
     * @param speedModifier Percent modifier on speed for this SwerveAngle object, defaults to 1.0
     * @param openLoop Whether to run drive motors in open loop or closed loop mode. Default=true
     * @param fieldOriented Whether to treat the X and Y inputs as field-oriented (true) or 
     *  robot-oriented (false). Default=true
     * @param flipOnAlliance Whether to flip the field-oriented drive based on which alliance the robot
     *  is on. Default=true
     * @param interpolateTranslation Whether to apply our joystick interpolation function to the
     *  translation magnitude. Defaults to true, set to false if not controlling from joystick input
     * @param interpolateRotation Whether to apply our joystick interpolation function to the
     *  rotation input. Defaults to true, set to false if not controlling from joystick input
     */
    public SwerveAngle(Swerve s_Swerve, Supplier<Double> forward, Supplier<Double> leftright, Supplier<Rotation2d> rotate, 
        double speedModifier, boolean openLoop, boolean fieldOriented, boolean flipOnAlliance,
        boolean interpolateTranslation, boolean interpolateRotation){
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.forward = forward;
        this.leftright = leftright;
        this.rotate = rotate;

        this.openLoop = openLoop;
        this.fieldOriented = fieldOriented;
        this.flipOnAlliance = flipOnAlliance;

        this.interpolateTranslation = interpolateTranslation;
        this.interpolateRotation = interpolateRotation;
        
        this.speedModifier = speedModifier;
    }

    @Override
    public void execute(){
        // Query suppliers
        double yAxis = -forward.get();
        double xAxis = -leftright.get();
        //double rAxis = -rotate.get();
        pid.setSetpoint(rotate.get().getDegrees());

        // Apply speed modifier
        yAxis *= speedModifier;
        xAxis *= speedModifier;
        //rAxis *= speedModifier;

        // Calculate rotation input
        /*if(interpolateRotation)
            rotation = JoystickUtils.interpolateNow(rAxis, Constants.STICK_DEADBAND) * Constants.Swerve.MAX_ANGULAR_VELOCITY;
        else
           rotation = rAxis * Constants.Swerve.MAX_ANGULAR_VELOCITY;*/ 
           rotation = pid.calculate(s_Swerve.swerveOdometry.getEstimatedPosition().getRotation().getDegrees());

        // Calculate translation input
        Translation2d x = new Translation2d(yAxis, xAxis);
        Translation2d y = new Translation2d(
            interpolateTranslation ?  
            JoystickUtils.interpolateNow(x.getNorm(), Constants.STICK_DEADBAND)
            : x.getNorm(), 
            x.getAngle());

        translation = y.times(Constants.Swerve.MAX_SPEED);

        // Tell the swerve drive what to do
        s_Swerve.drive(translation, rotation, fieldOriented, openLoop, flipOnAlliance);
    }
}
