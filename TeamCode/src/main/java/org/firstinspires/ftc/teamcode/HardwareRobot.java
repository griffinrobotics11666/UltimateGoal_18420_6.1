/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorDigitalTouch;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 */


public class HardwareRobot
{


    //define motors
    public DcMotor rearLeftDrive = null;
    public DcMotor rearRightDrive = null;
    public DcMotor frontRightDrive = null;
    public DcMotor frontLeftDrive = null;
    public DcMotor armMotor = null;
    public DcMotor shootyMotor = null;

    //define servos
    public Servo clawServo;
    public Servo clawRotationServo;
    public Servo shootyBoi;
    public Servo shootyRotation;

    VoltageSensor voltage;

    //define webcam vars
    public TFObjectDetector tfod;
    public VuforiaLocalizer vuforia;
    public static final String VUFORIA_KEY = "AdYYXLj/////AAABmbrz6/MNLUKlnU5JIPwkiDQ5jX+GIjfuIEgba3irGu46iS/W1Q9Z55uLSl31zGtBX3k5prkoSK6UxLR9gyvyIwSzRe2FOFGHEvJ19uG+pqiJJfkaRb0mCUkrx4U/fH6+Agp+7lOHB8IYjziNSuBMgABbrii5tAQiXOGfGojY+IQ/enBoy+zWiwVBx9cPRBsEHu+ipK6RXQe7CeODCRN8anBfAsn5b2BoO9lcGE0DgZdRysyByQ4wuwNQxKjba18fnzSDWpm12Brx3Ao1vkGYxTyLQfsON5VotphvWwoZpoyD+Iav/yQmOxrQDBLox6SosF8jqG9sUC5LAAdiRIWr6sNRrGzeCtsHSJBplHboPMB3";


    //define buttons
    public DigitalChannel touchyKid = null;

    //define gyros
    BNO055IMU imuControl;
    BNO055IMU imuExpansion;

    //define color & distance sensor
    //public ColorSensor colorSensor;
    //public DistanceSensor distanceSensor;

    /* local OpMode members. */
    HardwareMap hwMap =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareRobot(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        //Init drive motors
        frontLeftDrive  = hwMap.get(DcMotor.class, "front_left_drive");   //Expansion Hub Port 0
        rearLeftDrive  = hwMap.get(DcMotor.class, "rear_left_drive"); //Expansion Hub Port 1
        frontRightDrive = hwMap.get(DcMotor.class, "front_right_drive");  //Control Hub Port 0
        rearRightDrive = hwMap.get(DcMotor.class, "rear_right_drive");    //Control Hub Port 1
        armMotor = hwMap.get(DcMotor.class, "arm_motor");   //control hub port 2
        shootyMotor = hwMap.get(DcMotor.class, "shooty_motor");   //expansion hub port 2
        touchyKid = hwMap.get(DigitalChannel.class, "sensor_digital"); //control hub digital device port 0

        voltage = hwMap.voltageSensor.iterator().next();

        //sets init direction for drive motors
        rearLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        rearRightDrive.setDirection(DcMotor.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        armMotor.setDirection(DcMotor.Direction.FORWARD);
        shootyMotor.setDirection(DcMotor.Direction.REVERSE);
        shootyMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imuControl = hwMap.get(BNO055IMU.class, "imu_control");
        imuControl.initialize(parameters);
        imuExpansion = hwMap.get(BNO055IMU.class, "imu_expansion");
        imuExpansion.initialize(parameters);

        //initialize hardware variables for servos
        clawServo = hwMap.get(Servo.class, "claw_grip_servo");    //Control Hub Port 0
        clawRotationServo = hwMap.get(Servo.class, "claw_rotation_servo");  //Control Hub Port 1
        shootyBoi = hwMap.get(Servo.class, "shooty_boi_servo");    //Expansion Hub Port 1
        shootyRotation = hwMap.get(Servo.class, "shooty_rotation_servo");    //Expansion Hub port 2
        
        //init hardware variables for color & distance sensors
       // colorSensor = hwMap.get(ColorSensor.class, "color_distance_sensor");
       // distanceSensor = hwMap.get(DistanceSensor.class, "color_distance_sensor");
    }
}

