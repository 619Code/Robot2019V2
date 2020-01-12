
package frc.robot;

import java.util.Arrays;

import com.kauailabs.navx.frc.AHRS;

//import easypath.EasyPath;
//import easypath.EasyPathConfig;
//import easypath.PathUtil;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.opencv.core.Mat;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.vision.VisionThread;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.drive.PathWeaverDrive;
//import edu.wpi.first.wpilibj.Timer;
//import frc.robot.auto.Auto;
//import frc.robot.auto.type.LeftShip;
import frc.robot.drive.WestCoastDrive;
import frc.robot.hardware.Controller;
import frc.robot.hardware.LimitSwitch;
import frc.robot.maps.ControllerMap;
import frc.robot.maps.RobotMap;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Hatch;
import frc.robot.subsystems.HelperFunctions;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lift;
import frc.robot.teleop.TeleopThread;
import frc.robot.threading.ThreadManager;
import frc.robot.vision.VisionProcess;

public class Robot extends TimedRobot {
  public static WestCoastDrive sunKist;
  public static PathWeaverDrive arachne;

  public static Arm Arm;
  public static Hatch Hatch;
  public static Intake Intake;
  public static Lift Lift;
  public static Grabber Grabber;
  public static Climb Climb;

  //DigitalInput compressorStop = new DigitalInput(5); //ONLY FOR V1 BOT

  VisionProcess visionProcess;

  ThreadManager threadManager;
  TeleopThread teleopThread;
  //Auto auto;
  Compressor compressor;
  AHRS navX;

  //EasyPathConfig config;

  VisionThread visionThread;
  boolean autoStopped;

  @Override
  public void robotInit() {
    initNavX();
    initManipulators();
    //initAuto();
    initVision(false);
    threadManager = new ThreadManager();
    threadManager.killAllThreads();
  }
  
  @Override
  public void robotPeriodic() {
    Scheduler.getInstance().run();
    //ONLY FOR V1
    // if(compressorStop.get() == false) 
    //   c.setClosedLoopControl(true);
    // else
    //   c.setClosedLoopControl(false);
  }

  public void initManipulators() {
    sunKist = new WestCoastDrive(navX);

    Lift = new Lift();
    Intake = new Intake();
    Hatch = new Hatch();
    Arm = new Arm();
    Grabber = new Grabber();
    Climb = new Climb();
    
    compressor = new Compressor(RobotMap.PCM_CAN_ID);
    compressor.setClosedLoopControl(true); 
  }

  public void initNavX() {
    navX = new AHRS(Port.kMXP);
    navX.reset();
  }

  /*public void initAuto(){
    autoStopped = false;
    config = new EasyPathConfig(
      sunKist, 
      sunKist::setLeftandRight, 
      () -> PathUtil.defaultLengthDrivenEstimator(sunKist::getLeftEncoderInches, sunKist::getRightEncoderInches),  
      sunKist::getNavXAngle,
      sunKist::initDrive, 
      RobotMap.AUTO_kP);

    config.setSwapDrivingDirection(true);
    config.setSwapTurningDirection(false);

    EasyPath.configure(config);

    sunKist.setDriveOverride(false);
  }*/

  public void initVision(boolean useCV){
    UsbCamera camera0 = CameraServer.getInstance().startAutomaticCapture("Top Hatch Camera", 0);
    camera0.setResolution(160, 120);
    camera0.setFPS(30);

    // UsbCamera camera1 = CameraServer.getInstance().startAutomaticCapture("Hatch Camera", 1);
    // camera1.setResolution(160, 120);
    // camera1.setFPS(30);

    if(useCV){
      visionProcess = new VisionProcess();
      new Thread(() -> {
        CvSink cvSink = CameraServer.getInstance().getVideo();
        CvSource outputStream = CameraServer.getInstance().putVideo("Vision", 160, 120);
  
        Mat source = new Mat();
        Mat output = new Mat();
        
        while(!Thread.interrupted()){
          cvSink.grabFrame(source);
          if(source.rows() != 0){
            output = visionProcess.process(source);
            outputStream.putFrame(output);
          }
        }
      }).start();
    }
  }

  /*@Override
  public void autonomousInit() {
    threadManager.killAllThreads();
    teleopThread = new TeleopThread(threadManager);
    sunKist.setInAuto(true);
    sunKist.setToBrake();
    auto = new Auto();
    auto.start();
  }*/

  /*@Override
  public void autonomousPeriodic() {
    boolean driving = sunKist.drive(teleopThread.getDriveMode(), ControllerMap.Primary);
    if(driving) auto.stop();
  }*/

  @Override
  public void teleopInit(){
    threadManager.killAllThreads();
    teleopThread = new TeleopThread(threadManager);
    sunKist.setInAuto(false);
    sunKist.setDriveOverride(false);
    sunKist.setToBrake();
  }

  double highestCurrent = 0;

  @Override
  public void teleopPeriodic() {
    double current = sunKist.getCurrent();
    double[] currentArray = sunKist.getCurrentArray();

    sunKist.drive(teleopThread.getDriveMode(), ControllerMap.Primary);
    //SmartDashboard.putNumber("NavX Angle", sunKist.getNavXAngle());
    /*SmartDashboard.putNumber("Current", current);
    if(current > highestCurrent) {
      highestCurrent = current;
    }
    SmartDashboard.putNumber("Highest Current", highestCurrent);*/
    SmartDashboard.putNumber("Right Master", currentArray[0]);
    SmartDashboard.putNumber("Right Rear", currentArray[1]);
    SmartDashboard.putNumber("Right Front", currentArray[2]);
    SmartDashboard.putNumber("Left Master", currentArray[3]);
    SmartDashboard.putNumber("Left Rear", currentArray[4]);
    SmartDashboard.putNumber("Left Front", currentArray[5]);
  }

  // --------------------------------------

  Controller driver;
  Controller secondary;
  LimitSwitch limitSwitch;

  // DigitalInput left = new DigitalInput(RobotMap.LEFTAUTOSWITCH);
  // DigitalInput right = new DigitalInput(RobotMap.RIGHTAUTOSWITCH);
  // DigitalInput ship = new DigitalInput(RobotMap.SHIPAUTOSWITCH);
  // DigitalInput rocket = new DigitalInput(RobotMap.ROCKETAUTOSWITCH);

  @Override
  public void testInit() {
    // driver = new Controller(0);
    // secondary = new Controller(1);
    threadManager.killAllThreads();
  }

  @Override
  public void testPeriodic() {
    // System.out.println("Left: " + left.get());
    // System.out.println("Right: " + right.get());
    // System.out.println("Ship: " + ship.get());
    // System.out.println("Rocket: " + rocket.get());
    // System.out.println();
  }

  @Override
  public void disabledInit(){
    threadManager.killAllThreads();
  }
}