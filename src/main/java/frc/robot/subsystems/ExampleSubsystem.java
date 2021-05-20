/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
/*import edu.wpi.first.wpilibj.Encoder;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
*/
public class ExampleSubsystem extends SubsystemBase {
 /* private static final int kEncoderPortA = 0;
  private static final int kEncoderPortB = 1;
  private Encoder m_encoder;
  private static final int kEncoderPortC = 2;
  private static final int kEncoderPortD = 3;
  private Encoder m_encoder2;

  private final WPI_VictorSPX m_leftMotor = new WPI_VictorSPX(5);
    private final WPI_VictorSPX m_rightMotor = new WPI_VictorSPX(3);
    private final WPI_VictorSPX m_leftfollow = new WPI_VictorSPX(2);
    private final WPI_VictorSPX m_rightfollow = new WPI_VictorSPX(4);
  
    private final WPI_TalonSRX m_TopIntakeMotor1 = new WPI_TalonSRX(6); // gray
    private final WPI_TalonSRX m_BottomIntakeMotor2 = new WPI_TalonSRX(8); // pink :)
    private final WPI_TalonSRX m_TopIntakeMotor2 = new WPI_TalonSRX(7); // green 
    private final WPI_TalonSRX m_Pwnf = new WPI_TalonSRX(9); // white
  
    private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);
    */
  /**
   * Creates a new ExampleSubsystem.
   */
  public ExampleSubsystem() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }
  /*public void turn(String DIRECTION, int DEGREE) {

    m_encoder.reset();
    m_encoder2.reset();

    double setClicks = DEGREE / 6;

    if (DIRECTION == "ccw") {

      while (m_encoder.getDistance() < setClicks && m_encoder2.getDistance() < setClicks) {
        m_leftMotor.set(-0.5);
        m_rightMotor.set(0.5);
        // driveleft
        // driveright
        m_robotDrive.arcadeDrive(0.5, 0);
        System.out.println("Turning Left for {0} degrees" + DEGREE);
      }
    }

    else if (DIRECTION == "cw") {

      while (m_encoder.getDistance() > -setClicks && m_encoder2.getDistance() > -setClicks) {
        m_leftMotor.set(0.5);
        m_rightMotor.set(-0.5);
        // driveleft
        // driveright
        m_robotDrive.arcadeDrive(0.5, 0);
        System.out.println("Turning Right for {0} degrees" + DEGREE);
      }
    }
    
    else {
      m_leftMotor.set(0);
      m_rightMotor.set(0);
    }

    m_encoder.reset();
    m_encoder2.reset();
  }
*/
}
