package frc.robot.commands;

public class regressionMapData {
    public static double[][] rpmData = {
        //@y --> distance from targets (meters)
        //@x --> shooter speed (rpm)
        //(distance, rpm)
        {0.0, 0.0}, //exmaple data 
        {1.0, 500},
        {2.0, 1000},
        {3.0, 2000},
        {4.0, 4000}
    };
    public static double[][] angleData = {
        //@y --> distance from targets (meters)
        //@x --> shooter angle (degrees)
        //(distance, degrees)
        {0.0, 0.0}, //exmaple data 
        {1.0, 10},
        {2.0, 30},
        {3.0, 40},
        {4.0, 45}
    };
    public static double[][] timeData = {
        //@y --> distance from targets (meters)
        //@x --> projectile time in the air (seconds)
        //(distance, secpmds)
        {0.0, 0.0}, //exmaple data 
        {1.0, 1},
        {2.0, 1.2},
        {3.0, 1.5},
        {4.0, 1.9}
    };
}
