// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

//CTRE dependencies
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;

//WPILIB dependencies
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/* This class contains all variables and functions pertaining to a single Swerve Module. A 
 * Swerve Module is a two motor device that allows a wheel's speed and angle to be commanded 
 * separately. 
 */
public class SwerveModule extends SubsystemBase 
  {
    //A Swerve Module has a drive motor, a steering motor, and an encoder angle sensor
    private final TalonFX driveMotor; 
    private final TalonFX steeringMotor; 
    private final CANcoder steeringEncoder; 

    private final StatusSignal<Double> steeringEncoderPosition;
    private final StatusSignal<Double> steeringEncoderVelocity; 
    private final StatusSignal<Double> driveMotorPosition;
    private final StatusSignal<Double> driveMotorVelocity;
 
    //A value to store the stop angle passed in from the Swerve Module constructor
    private double stopAngle = 0;
    //A value to store the gear ratio passed in from the Swerve Module constructor
    private double gearRatio; 
    //Offsets for the normalize feature of the Swerve Module are stored in an array with default values of 0 
    private static double[] offsets = {0.0, 0.0, 0.0, 0.0};
    /* 
     * When applying the angle offset to a Swerve Module, it is necessary to increment through each 
     * Swerve Module. By convention, the front left module is used as the first module.  
    */
    private static final int ENCODER_BASE = SwerveConstants.ROTATIONFRONTLEFT;

  /**
   * Constructor for SwerveModule.
   * 
   * @param driveMotorID The CAN bus ID number of the drive motor of the Swerve Module
   * @param steeringMotorID The CAN bus ID number of the steering motor of the Swerve Module
   * @param steeringEncoderID The CAN bus ID number of the encoder angle sensor of the Swerve Module
   * @param stopAngle An angle specified (in degrees) to point the steering motor when the Swerve 
   * Module is not being commanded
  */
  public SwerveModule(int driveMotorID, int steeringMotorID, int steeringEncoderID, int stopAngle) 
    {
      //The drive motor is a CTRE Falcon 500 
      driveMotor = new TalonFX(driveMotorID); 
      //The steering motor is a CTRE Falcon 500
      steeringMotor = new TalonFX(steeringMotorID); 
      //The encoder angle sensor is a CTRE CANCoder 
      steeringEncoder = new CANcoder(steeringEncoderID);
      //Set the passed in stop angle's value to the subsystem stop angle 
      this.stopAngle = stopAngle;
      //Initialize preference to store steering motor offsets
      String prefKey = String.format("SwerveModule/Offset_%02d", steeringMotorID);
      Preferences.initDouble(prefKey, offsets[steeringMotorID-ENCODER_BASE]);
      //Assign the stored offsets in Preferences to the swerve module
      offsets[steeringMotorID-ENCODER_BASE] =  Preferences.getDouble(prefKey, offsets[steeringMotorID-ENCODER_BASE]);
      
      steeringEncoderPosition = steeringEncoder.getPosition();

      steeringEncoderVelocity = steeringEncoder.getVelocity();

      driveMotorPosition = driveMotor.getRotorPosition();

      driveMotorVelocity = driveMotor.getRotorVelocity();

      //BaseStatusSignal.waitForAll(0.1, steeringEncoderPosition, steeringEncoderVelocity, driveMotorPosition, driveMotorVelocity);
    } //End SwerveModule constructor
  

  /** 
  * Returns the current velocity and rotation angle of the swerve module 
  * (in meters per second and radians respectively) as a SwerveModuleState
  */ 
  public SwerveModuleState getSwerveModuleState() 
    { 
      final double angle = getAngle() - offsets[steeringMotor.getDeviceID()-ENCODER_BASE];
      return new SwerveModuleState(getVelocityMetersPerSecond(), Rotation2d.fromDegrees(angle)); 
    } 

  /** 
   * Returns the current angle of the encoder angle sensor (in radians) and the total distance traveled 
   * by the drive motor (in meters) as a SwerveModulePosition data type. The date provided by this function
   * is used for wheel odometry.
  */
  public SwerveModulePosition getSwerveModulePosition()
    { 
      final double absolute = getAngle(); 
      double angle = absolute - offsets[steeringMotor.getDeviceID()-ENCODER_BASE];
      return new SwerveModulePosition(getDistance(), Rotation2d.fromDegrees(angle)); 
    }
   
  /**
   * Allows for the Swerve Module to be commanded to any given veloctiy and angle and specifies 
   * if the wheels should use their stop angles
   * @param desiredState The state (speed and angle) to command the module to as SwerveModuleState data type
   * @param useStopAngle Boolean whether or not to command the wheels to their individual stop angles (useful 
   * for avoiding being pushed or to return the wheels to a neutral state when not commanded)
   * 
   */ 
  public void setDesiredState(SwerveModuleState desiredState, boolean useStopAngle) 
    {
      //Assign this internal SwerveModule state value the value of the desired state
      SwerveModuleState state = desiredState;      
      
      //Convert the velocity component of the desiredState to rotor rotations per second
      double driveSpeed = (state.speedMetersPerSecond * gearRatio) / SwerveConstants.WHEEL_CIRCUMFERENCE;
      //The raw, unormalized value of the encoder angle sensor
      final double absolute = getAngle(); 
      /* 
       * Get the difference between the commanded angle 
       * and the reported angle (normalized by the recorded offset value)
       * as a modulo-360 degree number. The encoder angle sensor used by the robot returns a double 
       * that wraps to what is effectively +- infinity, but internal fucntionality requires that this 
       * number be with a 0-360 degree range. 
      */ 
      double delta = AngleDelta( 
      absolute - offsets[steeringMotor.getDeviceID()-ENCODER_BASE], //Subtract the raw value from the recorded offset for the given swerve module
      state.angle.getDegrees()); //This is the angle portion (in degrees) of the desiredState SwerveModuleSate 
      /*
       * If the difference between the normalized current angle and the commanded angle is greater than 
       * 90 degrees, command the steering motor to stay at its current angle and invert the drive motor. This
       * saves time and energy.
      */
      if (delta > 90.0) 
        {
          /* 
           * Change delta to a mirrored version of whatever it was before, effectively commanding the
           * steering motor to stay put.  
          */ 
          delta -= 180.0;
          //Reverse the commanded speed to the drive motor
          driveSpeed *= -1;
        } 
        /* Since delta could be negative, it is 
         * necessary to cover the case where delta is less than or equal to -90 degrees. In effect, it is 
         * the complementary case to the one above.
        */
        else if (delta < -90.0)
          {
            /* 
             * Change delta to a mirrored version of whatever it was before, effectively commanding the
             * steering motor to stay put.  
            */ 
            delta += 180.0 ;
            // Reverse the commanded speed to the drive motor
            driveSpeed *= -1;
          } 
      final double target = AngleToEncoder(absolute + delta);
       if(driveSpeed == 0.0)
        { 
          if (useStopAngle == true)
            {  
             steeringMotor.setControl(new MotionMagicVoltage(AngleToEncoder(stopAngle)));
            } 
          else 
            {
              steeringMotor.set(0.0);
            }

           driveMotor.set(0.0);

        } 
       else 
          {
            //System.out.println("speed" + driveSpeed);
            steeringMotor.setControl(new MotionMagicVoltage(target)); 
            driveMotor.setControl(new VelocityVoltage(driveSpeed));
            //driveMotor.set(driveSpeed);
            
          }
       
  }

/** 
 * A getter for the velocity of the drive motor of the swerve module
 * @return the velocity of the drive motor, converted to meters per second
*/
public double getVelocityMetersPerSecond()
  { 
    BaseStatusSignal.refreshAll(driveMotorPosition, driveMotor.getAcceleration());
    return (BaseStatusSignal.getLatencyCompensatedValue(driveMotorVelocity, driveMotor.getAcceleration()) / gearRatio) * SwerveConstants.WHEEL_CIRCUMFERENCE;
    //return (driveMotor.getRotorVelocity().getValueAsDouble() / gearRatio) * SwerveConstants.WHEEL_CIRCUMFERENCE;
  } 

/** 
 * A getter for the angle of the steering motor of the swerve module.
 * @return the current angle of the steering motor, in degrees with no normalization.
 * NOTE: In order for this function to work correctly, CANCODERS must utilize "boot to absolute value"
 * boot strategy and be set to range 0 to 1 rotations
*/
public double getAngle()
  { 
    BaseStatusSignal.refreshAll(steeringEncoderPosition, steeringEncoderVelocity);
    return BaseStatusSignal.getLatencyCompensatedValue(steeringEncoderPosition, steeringEncoderVelocity) * 360;
    //return steeringEncoder.getPosition().getValueAsDouble() * 360;
  } 

//Convert an angle in degrees to rotor rotations 
private static double AngleToEncoder(double deg)
  {
      return deg / 360.0;
  }

private static double AngleDelta(double current, double target)
  {
      if (current < 0)
        {
          current += 360.0;
        } 
      if (target < 0)
        {
          target += 360.0;
        } 
      double delta = target - current;
      return Math.IEEEremainder(delta, 360);
  }
/**
 * Calculate an offset for the Swerve Module and save it to flash. This offset
 * will allow any direction to be set as the forward direction since encoders
 * almost never tell the Module to point in the correct direction relative to 
 * the robot. 
 */
public void normalizeModule() 
  {
      offsets[steeringMotor.getDeviceID()-ENCODER_BASE] = getAngle();
      String prefKey = String.format("SwerveModule/Offset_%02d", steeringMotor.getDeviceID());
      Preferences.setDouble(prefKey, offsets[steeringMotor.getDeviceID()-ENCODER_BASE]);
      System.out.printf("Normalized Module %d\n", driveMotor.getDeviceID());
  }

/** 
 * Get the distance reported by the drive motor internal sensor and convert it to meters
 * @return the distance traveled by the drive wheel in meters
 */ 
public double getDistance() 
  {  
    BaseStatusSignal.refreshAll(driveMotorPosition, driveMotorVelocity);
    return (BaseStatusSignal.getLatencyCompensatedValue(driveMotorPosition, driveMotorVelocity) / gearRatio) * SwerveConstants.WHEEL_CIRCUMFERENCE;
    //return (driveMotor.getRotorPosition().getValueAsDouble() / gearRatio) * SwerveConstants.WHEEL_CIRCUMFERENCE;
  } 

/**
* Set the drive motor of the Swerve Module to brake mode. "Brake Mode" means the motor will actively 
* fight against any attempts to turn the shaft while otherwise uncommanded using the back EMF 
* (Electromotive Force) generated by turning the shaft. 
*/ 
public void brakeMode()
  { 
    driveMotor.setNeutralMode(NeutralModeValue.Coast);  
  } 


/**
 * Congfigure any settings that vary based on the type of swerve module used. 
 * These settings include gear ratios and encoder internal configurations. 
 * @param moduleType A String name representing the module being used.
 */
public void setModuleSettings(String moduleType)
  {
    switch(moduleType)
    {
      case "geared upright":
      gearRatio = SwerveConstants.GEAR_RATIO_WCP_UPRIGHT;
      System.out.printf("Setting Module %d to %s Settings\n", steeringMotor.getDeviceID(), moduleType);
      break;
      case "geared flipped":
      gearRatio = SwerveConstants.GEAR_RATIO_WCP_GEARED;

      System.out.printf("Setting Module %d to %s Settings\n", steeringMotor.getDeviceID(), moduleType);
      break; 
      case "belted flipped":
      gearRatio = SwerveConstants.GEAR_RATIO_WCP_BELTED; 
      System.out.printf("Setting Module %d to %s Settings\n", steeringMotor.getDeviceID(), moduleType);
      break; 
      default:
      System.out.printf("Module %d not configured properly, check for possible spelling error in moduleType argument to constructor\n", steeringMotor.getDeviceID()); 
    }
  }
  
}//End class Swerve Module
