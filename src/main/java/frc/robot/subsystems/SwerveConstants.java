package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;

public final class SwerveConstants 
{
    public final static int DRIVEFRONTLEFT = 1;
    public final static int DRIVEFRONTRIGHT = 2;
    public final static int DRIVEBACKLEFT = 3;
    public final static int DRIVEBACKRIGHT = 4;

    public final static int ROTATIONFRONTLEFT = 21;
    public final static int ROTATIONFRONTRIGHT = 22;
    public final static int ROTATIONBACKLEFT = 23;
    public final static int ROTATIONBACKRIGHT = 24;

    public final static int ENCODERFRONTLEFT = 31;
    public final static int ENCODERFRONTRIGHT = 32;
    public final static int ENCODERBACKLEFT = 33;
    public final static int ENCODERBACKRIGHT = 34;

    public final static double WHEELRADIUS = 2; // inches
    public final static double WHEEL_CIRCUMFERENCE = Units.inchesToMeters(WHEELRADIUS * 2 * Math.PI);

    public static final double GEAR_RATIO_WCP_BELTED = 6.55; 
    public static final double GEAR_RATIO_WCP_GEARED = 6.55; 
    public static final double GEAR_RATIO_WCP_UPRIGHT = 7.42;

}
