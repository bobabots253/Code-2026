package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.shooterSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class shootCommand extends Command{
    private final shooterSubsystem m_shooter;
    private final SwerveSubsystem m_SwerveSubsystem;
    private Translation2d goalTranslation2d;
    public Pose2d currentPose;
    

    
    public static InterpolatingDoubleTreeMap rpmTreeMap = new InterpolatingDoubleTreeMap();
    public static InterpolatingDoubleTreeMap angleTreeMap = new InterpolatingDoubleTreeMap();
    public static InterpolatingDoubleTreeMap timeTreeMap = new InterpolatingDoubleTreeMap();

    public shootCommand(shooterSubsystem shooter, SwerveSubsystem driveSubsystem, Translation2d goalposition){
        m_shooter = shooter;
        m_SwerveSubsystem = driveSubsystem;

        for (double[] pair : regressionMapData.rpmData) {
            rpmTreeMap.put(pair[0], pair[1]);
        }
        for (double[] pair : regressionMapData.angleData) {
            angleTreeMap.put(pair[0], pair[1]);
        }
        for (double[] pair : regressionMapData.timeData) {
            timeTreeMap.put(pair[0], pair[1]);
        }

        // addRequirments(m_SwerveSubsystem, m_shooter);
    }

    @Override
    public void execute(){

        // Translation2d goalTranslation2d;
        goalTranslation2d = calculateForMovement(goalTranslation2d);
        double distance = goalTranslation2d.getDistance(m_SwerveSubsystem.getPose().getTranslation());
        //m_shooter.setAngle(angleTreeMap.get(goalTranslation2d.getDistance(m_SwerveSubsystem.getPose().getTranslation())));
        //m_shooter.setRPM(rpmTreeMap.get(goalTranslation2d.getDistance(m_SwerveSubsystem.getPose().getTranslation())));

    }

    Translation2d calculateForMovement(Translation2d goalTranslation2d){
        Translation2d toGoal = goalTranslation2d.minus(m_SwerveSubsystem.getPose().getTranslation());

        double horizontalVelocity = 0.0;//should equal to its horizontal velocity + its acceleration
        double verticalVelocity = 0.0;//also equal to its vertical velocity

        timeTreeMap.get(toGoal.getDistance(new Translation2d()));//this is what rachet rockers have but I think you should use what is below
        double shotTime = timeTreeMap.get(goalTranslation2d.getDistance(m_SwerveSubsystem.getPose().getTranslation()));

        return new Translation2d(goalTranslation2d.getX() - horizontalVelocity*shotTime, goalTranslation2d.getY() - verticalVelocity*shotTime);//They have subtraction in their code but it could change for us
    }
    
    @Override
    public void end(boolean interrupted){
        //m_shooter should probably be stop and be set to 0 rpm and maybe a home position of the angle.

    }

}
