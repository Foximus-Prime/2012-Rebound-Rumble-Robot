/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.foximus.prime;


import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class mainRobot extends IterativeRobot {
    
    Jaguar BL = new Jaguar(1);
    Jaguar FL = new Jaguar(2);
    Jaguar BR = new Jaguar(3);
    Jaguar FR = new Jaguar(4);
    
    Joystick joy1 = new Joystick(1);
    Joystick joy2 = new Joystick(2);
    
    RobotDrive drive = new RobotDrive(FL,BL,FR,BR);
    
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
        getWatchdog().setEnabled(false);
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {

    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
        //double magnitude, direction, rotation;
        //Create "deadzone" variables. Adjust threshold value to increase/decrease deadzone
        double X2 = 0, Y1 = 0, X1 = 0, threshold = 15.0;
        while (true && isOperatorControl() && isEnabled()) // loop until change 
        {
            
            /*magnitude = joy1.getMagnitude();
            direction = joy1.getDirectionDegrees();
            rotation = joy2.getX();*/
            //drive.mecanumDrive_Cartesian(magnitude, direction, rotation, 0); 
            //drive.mecanumDrive_Cartesian(joy1.getX(), joy1.getY(), joy1.getZ(), 0.0);
            //drive.mecanumDrive_Polar(joy1.getX(), joy1.getY(), joy2.getX());
            //Timer.delay(0.005);
            
            
            
            
            
            /*
            magnitude = joy1.getY();
            direction = joy1.getX();
            rotation = joy2.getX();
            
            drive.mecanumDrive_Polar(magnitude, direction, rotation);
            Timer.delay(0.005);
            */

            //Create "deadzone" for Y1
            if(Math.abs(joy1.getY()) > threshold)
                Y1 = joy1.getY();
            else
                Y1 = 0;
            //Create "deadzone" for X1
            if(Math.abs(joy1.getX()) > threshold)
                X1 = joy1.getX();
            else
                X1 = 0;
            //Create "deadzone" for X2
            if(Math.abs(joy2.getX()) > threshold)
                X2 = joy2.getX();
            else
                X2 = 0;
            
            double max = 0.0;
            if(Math.abs(Y1 - X2 - X1) > max)
                max = Math.abs(Y1 - X2 - X1);
            if(Math.abs(Y1 - X2 + X1) > max)
                max = Math.abs(Y1 - X2 + X1);
            if(Math.abs(Y1 + X2 + X1) > max)
                max = Math.abs(Y1 + X2 + X1);
            if(Math.abs(Y1 + X2 - X1) > max)
                max = Math.abs(Y1 + X2 - X1);

            //Remote Control Commands
            FR.set((Y1 - X2 - X1)/max);
            BR.set((Y1 - X2 + X1)/max);
            FL.set((Y1 + X2 + X1)/max);
            BL.set((Y1 + X2 - X1)/max);
        }

    }    
}
