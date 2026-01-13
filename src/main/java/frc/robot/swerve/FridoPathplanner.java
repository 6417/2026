//TODO: Decide between Pathplanner and Choreographer for autonomous paths
// package frc.robot.swerve;

// import com.pathplanner.lib.commands.PathPlannerAuto;
// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.auto.NamedCommands;
// import com.pathplanner.lib.config.PIDConstants;
// import com.pathplanner.lib.config.RobotConfig;
// import com.pathplanner.lib.controllers.PPHolonomicDriveController;
// import com.pathplanner.lib.path.PathPlannerPath;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import java.util.Map;

// public class FridoPathplanner {
//     private SwerveDrive drive;
    
//     public FridoPathplanner(SwerveDrive drive) {
//         this.drive = drive;

//         // configuration
//         RobotConfig config;
//         try {
//             config = RobotConfig.fromGUISettings();
//         } catch (Exception e) {
//             e.printStackTrace();
//             config = null;
//         }

//         AutoBuilder.configure(
//                 drive::getPose,
//                 drive::resetOdoemetry,
//                 drive::getChassisSpeeds,
//                 drive::setChassisSpeeds,
//                 new PPHolonomicDriveController(
//                         new PIDConstants(0.75,0, 0.15),
//                         new PIDConstants(1, 0.0, 0.1)),
//                 config,
//                 () -> {

//                     var alliance = DriverStation.getAlliance();
//                     if (alliance.isPresent()) {
//                         return false;
//                     }
//                     return false;
//                 },
//                 drive
//         );
//     }

//     public void registerCommand(String name, Command command) {
//         NamedCommands.registerCommand(name, command);
//     }

//     public void registerCommand(Map<String, Command> namedCommands) {
//         NamedCommands.registerCommands(namedCommands);
//     }

//     public Command getAutoCommandGroup(String fileName) {
//         try {
//            return new PathPlannerAuto(fileName);

//         } catch (Exception e) {
//             DriverStation.reportError("PathPlanner failed: " + e.getMessage(), e.getStackTrace());
//             return Commands.none();
//         }
//     }
    
//     public Command getAutonomousCommand(String fileName) {

//         // return new PathPlannerAuto("Example Auto");
//         try {
//             PathPlannerPath path = PathPlannerPath.fromPathFile(fileName);

//             // Possible to implement a Path from here
            
//             return AutoBuilder.followPath(path);
//         } catch (Exception e) {
//             DriverStation.reportError("PathPlanner failed: " + e.getMessage(), e.getStackTrace());
//             return Commands.none();
//         }
//     }
// }

