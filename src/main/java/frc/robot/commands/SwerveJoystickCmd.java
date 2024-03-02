package frc.robot.commands;

import java.util.function.Supplier;

/*    REPLACED BY ALTERNATE IMPORTS (SEE BELOW)
import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
*/

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;  //Changed to Command from CommandBase
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;

import frc.robot.subsystems.ShootingSubsystem;





public class SwerveJoystickCmd extends Command {     //Changed to Command from CommandBase
  
    private final SwerveSubsystem swerveSubsystem;
    private final ShootingSubsystem shootingSubsystem;


    private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
    private final Supplier<Boolean> fieldOrientedFunction;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
    private final Supplier<Boolean> orient90CWFunction;  //ADDED FOR A BUTTON TO TURN 90 Degrees CW
    private final Supplier<Boolean> orient90CCWFunction;  //ADDED FOR A BUTTON TO TURN 90 Degrees CCW
    private final Supplier<Boolean> orient0DegFunction;  //ADDED FOR A BUTTON TO TURN 0 Degrees (STRAIGHT TOWWARD OPPONENT'S WALL)

    private final Supplier<Boolean> DriveTurboFunction; // ADDED FOR A BUTTON TO CHANGE DRIVE SPEED

    private final Supplier<Boolean> shootSpeakerFunction;  //ADDED FOR A BUTTON TO SHOOT HIGH GOAL OF "SPEAKER"
    private final Supplier<Boolean> shootAmpFunction;   //ADDED FOR A BUTTON TO SHOOT LOW GOAL OF "AMP"
    private final Supplier<Boolean> intakeStarRollersOnFunction;    //ADDED FOR A BUTTON TO TURN ON "INTAKE-STAR ROLLERS"
    //private final Supplier<Boolean> intakeStarRollersOffFunction;   //ADDED FOR A BUTTON TO TURN OFF "INTAKE-STAR ROLLERS"
    private final Supplier<Boolean> intakeWheelRollersOnFunction;    //ADDED FOR A BUTTON TO TURN ON "INTAKE- WHEEL ROLLERS"
    //private final Supplier<Boolean> intakeWheelRollersOffFunction;    //ADDED FOR A BUTTON TO TURN OFF "INTAKE- WHEEL ROLLERS"

    //private final Spark primeMotor1;
    //primerMotor1 = new Spark(Constants.OperatorConstants.kPrimer1PWM);  //2
    //primerMotor2 = new Spark(Constants.OperatorConstants.kPrimer2PWM); 

    

    public SwerveJoystickCmd(SwerveSubsystem swerveSubsystem,ShootingSubsystem shootingSubsystem,
            Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction,
            Supplier<Boolean> fieldOrientedFunction, Supplier<Boolean> orient90CWFunction,Supplier<Boolean> orient90CCWFunction,Supplier<Boolean> orient0DegFunction, //ADDED FOR A BUTTON TO TURN 90 Degrees CW
            Supplier<Boolean> DriveTurboFunction, // For Speed Toggle
            Supplier<Boolean> shootSpeakerFunction,Supplier<Boolean> shootAmpFunction,Supplier<Boolean> intakeStarRollersOnFunction,Supplier<Boolean> intakeWheelRollersOnFunction)
        {  
        this.swerveSubsystem = swerveSubsystem;  //this. required because "this denotes the instance variable (from class definition) and set equal to the parameter you're passing"
        this.shootingSubsystem = shootingSubsystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turningSpdFunction = turningSpdFunction;
        this.fieldOrientedFunction = fieldOrientedFunction; 
        this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
        this.orient90CWFunction =  orient90CWFunction;  //ADDED FOR A BUTTON TO TURN 90 Degrees CW
        this.orient90CCWFunction =  orient90CCWFunction;  //ADDED FOR A BUTTON TO TURN 90 Degrees CCW
        this.orient0DegFunction =  orient0DegFunction;  //ADDED FOR A BUTTON TO TURN  0 Degrees (STRAIGHT TOWWARD OPPONENT'S WALL)
        
        this.DriveTurboFunction = DriveTurboFunction; // Added for Drive Turbo

        this.shootSpeakerFunction =  shootSpeakerFunction;  
        this.shootAmpFunction =  shootAmpFunction; 
        this.intakeStarRollersOnFunction =  intakeStarRollersOnFunction;
        this.intakeWheelRollersOnFunction =  intakeWheelRollersOnFunction;


        //DO NOT DELETE addRequirements command below!!!   
        
        //2/23/24 -> CONTINUE CODE FOR 2nd XBOX CONTROLLER BUTTONS
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("SwerveJoystickCmd Initilizing");
        
    }

    @Override
    public void execute() {
        System.out.println("SwerveJoystickCmd Executing");
        
        Boolean DriveTurboMode = false;
        Boolean DriveTurbo = DriveTurboFunction.get();
        double DriveTurboMultiplier;

        if (DriveTurbo & DriveTurboMode == false)
        {
            DriveTurboMode = true;
        }
        else if (DriveTurbo & DriveTurboMode == true)
        {
            DriveTurboMode = false;
        }

        if (DriveTurboMode == true)
        {
            DriveTurboMultiplier = 0.5;
        }
        else
        {
            DriveTurboMultiplier = 1.0;
        }

        // 1. Get real-time joystick inputs
        double xSpeed = xSpdFunction.get() * DriveTurboMultiplier;
        double ySpeed = ySpdFunction.get() * DriveTurboMultiplier;
        double turningSpeed = turningSpdFunction.get() * DriveTurboMultiplier;

        SmartDashboard.putBoolean("Turbo Mode", !DriveTurboMode);


        // 2. Apply deadband
        xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > OIConstants.kRotationDeadband ? turningSpeed : 0.0;

        // 3. Make the driving smoother
        xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        turningSpeed = turningLimiter.calculate(turningSpeed)
                * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

        // 4. Construct desired chassis speeds

        //MAYBE BREAK UP THIS IF STATEMENT?
        ChassisSpeeds chassisSpeeds;
        if (fieldOrientedFunction.get()) {
            // Relative to field
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());
        } else {
            // Relative to robot
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
        }
             
        //ADDED TO DISPLAY VALUES OF XBOX B,Y,X BUTTONS FOR "ORIENT"" ROBOT CODE
        boolean blnOrient90CW = orient90CWFunction.get();
        boolean blnOrient90CCW = orient90CCWFunction.get();
        boolean blnOrient0Deg = orient0DegFunction.get();
         SmartDashboard.putBoolean("90 Degree CW Button Test!()", blnOrient90CW);
         SmartDashboard.putBoolean("90 Degree CCW Button Test!()", blnOrient90CCW);
         SmartDashboard.putBoolean("90 Degree 0 DEG Button Test!()", blnOrient0Deg);
        // SmartDashboard.putNumber("Robot Heading", Math.round(getHeading()));

        //ADDED TO DISPLAY VALUES OF XBOX B,Y,X BUTTONS FOR "ORIENT"" ROBOT CODE
        boolean blnShootSpeaker = shootSpeakerFunction.get();
        boolean blnShootAmp = shootAmpFunction.get();
        boolean blnIntakeStar = intakeStarRollersOnFunction.get();
        boolean blnIntakeWheel = intakeWheelRollersOnFunction.get();
        //  SmartDashboard.putBoolean("SHOOT SPEAKER ON Test!()", blnShootSpeaker);
        //  SmartDashboard.putBoolean("SHOOT AMP ON Test!()", blnShootAmp);
        //  SmartDashboard.putBoolean("INTAKE STAR ROLLERS ON Test!()", blnIntakeStar);
        //  SmartDashboard.putBoolean("INTAKE WHEEL ROLLERSON Test!()", blnIntakeWheel);

//MANUALLY SETTING SPEEDS BEGIN
/* 
        double xSpeed = .25;
        double ySpeed = .25;
        double turningSpeed = .25;
        ChassisSpeeds chassisSpeeds;
         chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
*/
//MANUALLY SETTING SPEEDS END

        // 5. Convert chassis speeds to individual module states
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        // 6. Output each module states to wheels
        swerveSubsystem.setModuleStates(moduleStates);

        shootingSubsystem.mtdShootSpeaker(blnShootSpeaker,blnShootAmp); 
        // shootingSubsystem.mtdShootAmp(blnShootAmp); // Combined with Shoot Speaker to address conflict.
        shootingSubsystem.mtdIntake(blnIntakeStar);
        shootingSubsystem.mtdShooter(blnIntakeWheel);

    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("SwerveJoystickCmd Ending");
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        System.out.println("SwerveJoystickCmd Finished");
        return false;
    }
}