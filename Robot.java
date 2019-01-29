package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Compressor;

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;

public class Robot extends TimedRobot {

  private DifferentialDrive m_myRobot;

  private Joystick _gamepad2;
  private Joystick _gamepad;
  private Joystick m_FB;
  private Joystick m_LR;

  //Drive ID's
  private static final int leftMasterID = 2;
  private static final int leftSlaveID = 3;
  private static final int leftSlave2ID = 4;
  private static final int rightMasterID = 5;
  private static final int rightSlaveID = 6;
  private static final int rightSlave2ID = 7;
  
  //Drive Motors
  private CANSparkMax _leftMaster;
  private CANSparkMax _leftSlave;
  private CANSparkMax _leftSlave2;
  private CANSparkMax _rightMaster;
  private CANSparkMax _rightSlave;
  private CANSparkMax _rightSlave2;

  //Power Takeoff
  private CANSparkMax _powerTakeoffMaster;
  private CANSparkMax _powerTakeoffSlave;
  
  //Fly Swatter
  private TalonSRX _flySwatterMaster = new TalonSRX(10);
  private TalonSRX _flySwatterSlave = new TalonSRX(11);

  //Solenoids
  private DoubleSolenoid _gearShift = new DoubleSolenoid(4, 5);
  private DoubleSolenoid _hatchGrab = new DoubleSolenoid(0, 1);
  private DoubleSolenoid _hatchEject = new DoubleSolenoid(2, 3);
  private DoubleSolenoid _powerTakeoff = new DoubleSolenoid(6, 7);

  //Compressor
  private Compressor _compressor = new Compressor();


  @Override
  public void robotInit() {

    _leftMaster = new CANSparkMax(leftMasterID, MotorType.kBrushless);
    _leftSlave = new CANSparkMax(leftSlaveID, MotorType.kBrushless);
    _leftSlave2 = new CANSparkMax(leftSlave2ID, MotorType.kBrushless);
    _rightMaster = new CANSparkMax(rightMasterID, MotorType.kBrushless);
    _rightSlave = new CANSparkMax(rightSlaveID, MotorType.kBrushless);
    _rightSlave2 = new CANSparkMax(rightSlave2ID, MotorType.kBrushless);

    _leftSlave.follow(_leftMaster);
    _leftSlave2.follow(_leftMaster);
    _rightSlave.follow(_rightMaster);
    _rightSlave2.follow(_rightMaster);


    m_myRobot = new DifferentialDrive(_leftMaster, _rightMaster);
    m_myRobot.setExpiration(Robot.kDefaultPeriod);

    _powerTakeoffMaster = new CANSparkMax(8, MotorType.kBrushless);
    _powerTakeoffSlave = new CANSparkMax(9, MotorType.kBrushless);

    m_FB = new Joystick(0);
    m_LR = new Joystick(0);
    _gamepad = new Joystick(0);

    //camera 3
    new Thread(() -> {
      UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
      camera.setResolution(640, 480);
      
      CvSink cvSink = CameraServer.getInstance().getVideo();
      CvSource outputStream = CameraServer.getInstance().putVideo("Blur", 640, 480);
      
      Mat source = new Mat();
      Mat output = new Mat();
      
      while(!Thread.interrupted()) {
          cvSink.grabFrame(source);
          Imgproc.cvtColor(source, output, Imgproc.COLOR_BGR2GRAY);
          outputStream.putFrame(output);
      }
  }).start();

  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {

    //Drive Invert?
    _leftMaster.setInverted(false);
    _leftSlave.setInverted(false);
    _leftSlave2.setInverted(false);
    _rightMaster.setInverted(true);
    _rightSlave.setInverted(true);
    _rightSlave2.setInverted(true);

    //Fly Swatter
    _flySwatterSlave.follow(_flySwatterMaster);

    //Compressor
    _compressor.start();

    //Gear Shift
    _gearShift.set(DoubleSolenoid.Value.kOff);
    _gearShift.set(DoubleSolenoid.Value.kForward);
    _gearShift.set(DoubleSolenoid.Value.kReverse);

    //Hatch Grab
    _hatchGrab.set(DoubleSolenoid.Value.kOff);
    _hatchGrab.set(DoubleSolenoid.Value.kForward);
    _hatchGrab.set(DoubleSolenoid.Value.kReverse);

    //Hatch Eject
    _hatchEject.set(DoubleSolenoid.Value.kOff);
    _hatchEject.set(DoubleSolenoid.Value.kForward);
    _hatchEject.set(DoubleSolenoid.Value.kReverse);


    //Power Takeoff
    _powerTakeoff.set(DoubleSolenoid.Value.kOff);
    _powerTakeoff.set(DoubleSolenoid.Value.kForward);
    _powerTakeoff.set(DoubleSolenoid.Value.kReverse);

  }

  @Override
  public void teleopPeriodic() {

    //Gear Shift
    boolean gearShiftH = _gamepad.getRawButton(5);
    boolean gearShiftL = _gamepad.getRawButton(6);

    //Idle Mode
    boolean _coast = _gamepad.getRawButton(7);
    boolean _brake = _gamepad.getRawButton(8);

    //Tail
    boolean Grab = _gamepad2.getRawButton(2);
    boolean Eject = _gamepad2.getRawButton(10);

    //Fly Swatter
    //boolean flySwatterD = _gamepad2.getRawButton(3);
    //boolean flySwatterU = _gamepad2.getRawButton(4);

    //Power Takeoff
    boolean PTF = _gamepad2.getRawButton(5);
    boolean PTR = _gamepad2.getRawButton(6);
    boolean PTO = _gamepad2.getRawButton(7);
    boolean powerTakeoffF = _gamepad2.getRawButton(8);
    boolean powerTakeoffB = _gamepad2.getRawButton(9);
    


    m_myRobot.arcadeDrive(m_FB.getY(), m_LR.getX());
    

    //Idle Mode
    if(_coast == true){

      _leftMaster.setIdleMode(IdleMode.kCoast);
      _leftSlave.setIdleMode(IdleMode.kCoast);
      _leftSlave2.setIdleMode(IdleMode.kCoast);
      _rightMaster.setIdleMode(IdleMode.kCoast);
      _rightSlave.setIdleMode(IdleMode.kCoast);
      _rightSlave2.setIdleMode(IdleMode.kCoast);

    }

    if(_brake == true) {

      _leftMaster.setIdleMode(IdleMode.kBrake);
      _leftSlave.setIdleMode(IdleMode.kBrake);
      _leftSlave2.setIdleMode(IdleMode.kBrake);
      _rightMaster.setIdleMode(IdleMode.kBrake);
      _rightSlave.setIdleMode(IdleMode.kBrake);
      _rightSlave2.setIdleMode(IdleMode.kBrake);

    }

    //Gear Shift
    if(gearShiftH == true){

    _gearShift.set(DoubleSolenoid.Value.kForward);

  }

    if(gearShiftL == true){

      _gearShift.set(DoubleSolenoid.Value.kReverse);

    }

    //tail
    if(Grab == true) {

      _hatchGrab.set(DoubleSolenoid.Value.kForward);
    
    }
      else if(Grab == true) {

        _hatchGrab.set(DoubleSolenoid.Value.kReverse);
      }

    if(Eject == true) {

      _hatchEject.set(DoubleSolenoid.Value.kForward);
      
    }
      else if(Eject == true) {
  
         _hatchEject.set(DoubleSolenoid.Value.kReverse);
      }

    //Fly Swatter
   /* if(flySwatterU == true) {

      _flySwatterMaster.setInverted(false);

    }

    if(flySwatterD == true) {

      _flySwatterMaster.setInverted(true);

    }*/

    //Power Takeoff Motors
    if(powerTakeoffF == true) {

      _powerTakeoffMaster.set(1);
      _powerTakeoffSlave.set(1);

    }

    if(powerTakeoffB == true) {

      _powerTakeoffMaster.set(-1);
      _powerTakeoffSlave.set(-1);

    }

    //Power Takeoff
    if(PTF == true) {

      _powerTakeoff.set(DoubleSolenoid.Value.kForward);

    }

    if(PTR == true) {

      _powerTakeoff.set(DoubleSolenoid.Value.kReverse);

    }

    if(PTO == true) {

      _powerTakeoff.set(DoubleSolenoid.Value.kOff);

    }
  }
    

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

}
