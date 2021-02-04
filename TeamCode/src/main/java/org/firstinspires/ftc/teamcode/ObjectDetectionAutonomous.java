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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@Autonomous(name="Object Detection Autonomous", group="Linear Opmode")
//@Disabled
public class ObjectDetectionAutonomous extends LinearOpMode {
    HardwareRobot robot = new HardwareRobot();

    Orientation angleExpansion;
    Orientation angleControl;


    boolean isRoofRaised = true;
    private static final double SHOOTY_BOI_SERVO_SHOOT_POS     =  0.64;
    private static final double SHOOTY_BOI_SERVO_LOAD_POS     =  0.47;
    private static final double SHOOTY_ROTATION_FLAT_POS     =  0.64;
    private static double SHOOTY_ROTATION_LAUNCH = 0.13;
    private static final double CLAW_ROTATION_SERVO_PICKUP     =  0.35;
    private static final double CLAW_ROTATION_SERVO_DROP     =  0.49;
    private static final double CLAW_SERVO_OPEN_POS     =  0.35;
    private static final double CLAW_SERVO_CLOSE_POS     =  0.55;

    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    private enum likelyAssetsDetected{quad, single, none};
    private static likelyAssetsDetected likelyAssetsDetected;

    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 537.6;    // encoder counts per revolution
    static final double DRIVE_GEAR_REDUCTION = 2.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;


    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        int quadDetected = 0;
        int singleDetected = 0;
        int counter = 0;
        int noRingDetected = 0;

        initVuforia();
        initTfod();

        if (robot.tfod != null) {
            robot.tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 1.78 or 16/9).

            // Uncomment the following line if you want to adjust the magnification and/or the aspect ratio of the input images.
            robot.tfod.setZoom(1.5, 1.78);
        }

        telemetry.addData("Status", "Ready");
        telemetry.update();
        robot.clawRotationServo.setPosition(0.68);
        robot.clawServo.setPosition(0.22);
        robot.shootyRotation.setPosition(0.89);


        telemetry.addData("Shooty Launch Rotation pos", SHOOTY_ROTATION_LAUNCH);
        telemetry.addData("voltage", robot.voltage.getVoltage());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        encoderDrive(DRIVE_SPEED, 3, 10);
        turn(20, TURN_SPEED);


        if(robot.voltage.getVoltage() > 13.8){
            SHOOTY_ROTATION_LAUNCH = 0.15;
        } else if (robot.voltage.getVoltage() > 13.5){
            SHOOTY_ROTATION_LAUNCH = 0.14;
        } else if(robot.voltage.getVoltage() > 13.15){
            SHOOTY_ROTATION_LAUNCH = 0.13;
        } else if(robot.voltage.getVoltage() > 12.9){
            SHOOTY_ROTATION_LAUNCH = 0.12;
        } else if(robot.voltage.getVoltage() > 12.78){
            SHOOTY_ROTATION_LAUNCH = 0.11;
        } else if(robot.voltage.getVoltage() >= 12.65){
            SHOOTY_ROTATION_LAUNCH = 0.10;
        } else if(robot.voltage.getVoltage() < 12.65){
            SHOOTY_ROTATION_LAUNCH = 0.09;
        }

        if (opModeIsActive()) {
            while (opModeIsActive() && counter != 150) {
                if (robot.tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = robot.tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        // step through the list of recognitions and display boundary info.
                        int i = 0;
                        for (Recognition recognition : updatedRecognitions) {
                            if(recognition.getLabel().equals("Single")){
                                singleDetected++;
                            } else if (recognition.getLabel().equals("Quad")){
                                quadDetected++;
                            } else if(updatedRecognitions.size() == 0){
                                noRingDetected++;
                            }
                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                    recognition.getLeft(), recognition.getTop());
                            telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                    recognition.getRight(), recognition.getBottom());
                        }
                        telemetry.update();
                        counter++;
                    }
                }
            }
        }

        if (robot.tfod != null) {
            robot.tfod.shutdown();
        }
        if(singleDetected != 0 && singleDetected > quadDetected){
            likelyAssetsDetected = likelyAssetsDetected.single;
            telemetry.addLine("single likely");
            telemetry.update();
            //likely single
        } else if (quadDetected != 0 && quadDetected > singleDetected){
            likelyAssetsDetected = likelyAssetsDetected.quad;
            telemetry.addLine("quad likely");
            telemetry.update();
        } else {
            likelyAssetsDetected = likelyAssetsDetected.none;
            telemetry.addLine("none likely");
            telemetry.update();
        }

        turn(-20, TURN_SPEED);


        //gyro stuff for turn()
        robot.imuControl.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        robot.imuExpansion.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        sleep(2000); //TODO maybe take out?

        encoderDrive(DRIVE_SPEED, 42, 30);      //moves the robot forward 24 inches todo change this to the distance we want it to move forward when shooting

        if (!robot.touchyKid.getState()) {   //checks if limit switch is closed - basically a safety. WILL NOT RUN if arm does not start fully up

            //lowers arm
            robot.armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.armMotor.setTargetPosition(2900);
            robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.armMotor.setPower(1);
            if (robot.armMotor.getCurrentPosition() >= 2900) {
                robot.armMotor.setPower(0.0);
            }

            robot.shootyMotor.setPower(0.59);       //turn on the shooty motor

            robot.shootyRotation.setPosition(SHOOTY_ROTATION_LAUNCH);    //sets the shooting platform to the high angle
            robot.clawRotationServo.setPosition(CLAW_ROTATION_SERVO_PICKUP);

            turn(-3, TURN_SPEED);  //turns left (make positive if turns right) 10 degrees todo test this with different values to find the best one to hit the first target
            sleep(5500); //small delay so things dont happen too quickly, adjust time and add/remove more if needed

            //SHOOT ONCE
            shoot();    //see method below
            sleep(1500);
            robot.shootyRotation.setPosition(SHOOTY_ROTATION_LAUNCH);
            sleep(1000);


            //SHOOT TWICE
            shoot();
            sleep(1500);
            robot.shootyRotation.setPosition(SHOOTY_ROTATION_LAUNCH);
            sleep(1000);

            //SHOOT THRICE
            shoot();
            turn(3, TURN_SPEED);   //rotates the bot back to its original angle

            //robot.clawRotationServo.setPosition(CLAW_ROTATION_SERVO_PICKUP);


            robot.shootyMotor.setPower(0);       //turn off the shooty motor


            switch(likelyAssetsDetected){
                case none: {
                    robot.clawServo.setPosition(CLAW_SERVO_OPEN_POS);
                }
                case single: {
                    robot.clawServo.setPosition(CLAW_SERVO_OPEN_POS);
                }
                case quad: {
                    robot.clawServo.setPosition(CLAW_SERVO_OPEN_POS);
                }
            }
            sleep(3000);

            //moves the arm motor back up
            robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.armMotor.setTargetPosition(0);
            robot.armMotor.setPower(1);
            if (!robot.touchyKid.getState()) {
                robot.armMotor.setPower(0.0);
            }
            robot.shootyBoi.setPosition(SHOOTY_BOI_SERVO_LOAD_POS);
            robot.shootyRotation.setPosition(SHOOTY_ROTATION_FLAT_POS); //returns the shooting platform to its normal flat position
        }

        encoderDrive(DRIVE_SPEED, 30, 30);  //todo whats this?

        telemetry.addData("Path", "Complete");

        telemetry.update();

    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */

    public void encoderDrive(double speed, double inches, double timeoutS) {
        int encoderTarget;


        double currentSpeed = 0.2;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            //the motors will come to a hard stop instead of coasting
            robot.frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.rearLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.rearRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            robot.rearLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rearRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //Reset Encoders

            robot.frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rearLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rearRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // calculate target positions
            encoderTarget = robot.frontLeftDrive.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH * -1);

            //set target positions for motors
            robot.frontLeftDrive.setTargetPosition(encoderTarget);
            robot.frontRightDrive.setTargetPosition(encoderTarget);
            robot.rearLeftDrive.setTargetPosition(encoderTarget);
            robot.rearRightDrive.setTargetPosition(encoderTarget);

            // Turn On RUN_TO_POSITION
            robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rearLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rearRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.frontLeftDrive.setPower(currentSpeed);
            robot.frontRightDrive.setPower(currentSpeed);
            robot.rearLeftDrive.setPower(currentSpeed);
            robot.rearRightDrive.setPower(currentSpeed);


            //if one of these is false, the loop will exit and will continue to set power for all wheels to 0
            while (opModeIsActive() && (runtime.seconds() < timeoutS) && robot.frontLeftDrive.isBusy() && robot.frontRightDrive.isBusy() && robot.rearLeftDrive.isBusy() && robot.rearRightDrive.isBusy()) {

                if(robot.frontLeftDrive.getCurrentPosition() < encoderTarget/2){   //accelerate in the positive direction
                    currentSpeed += 0.01;
                    telemetry.addData("accelerating", "positive");
                } else if (robot.frontLeftDrive.getCurrentPosition() > encoderTarget/2){    //accelerate in the negative direction
                    currentSpeed -= 0.01;
                    telemetry.addData("accelerating", "negative");
                }

                robot.frontLeftDrive.setPower(currentSpeed);
                robot.frontRightDrive.setPower(currentSpeed);
                robot.rearLeftDrive.setPower(currentSpeed);
                robot.rearRightDrive.setPower(currentSpeed);
                telemetry.addData("Current Speed", currentSpeed);
                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d", encoderTarget);
                telemetry.addData("Path2", "Running at %7d", robot.frontLeftDrive.getCurrentPosition());
                telemetry.update();
            }


            // Stop all motion;
            robot.frontLeftDrive.setPower(0);
            robot.frontRightDrive.setPower(0);
            robot.rearLeftDrive.setPower(0);
            robot.rearRightDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rearLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rearRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move
        }
    }

    public void shoot(){    //code taken from driver control
        robot.shootyBoi.setPosition(SHOOTY_BOI_SERVO_SHOOT_POS);
        double pressTime = runtime.milliseconds();
        while(runtime.milliseconds()-pressTime < 250){  //wait until the shooty servo has fully moves forward.
            telemetry.addData("Wait " ,"true");
            telemetry.update();
        }
        robot.shootyBoi.setPosition(SHOOTY_BOI_SERVO_LOAD_POS);

    }

    public void turn(double turnAngle, double speed) {
        double currentAngle = readDoubleAngle();
        double lastAngle = currentAngle;
        double deltaAngle = 0;
        double totalAngle = 0;
        if (Math.abs(deltaAngle) >= 180) {
            deltaAngle = currentAngle + lastAngle;
        }
        double increment = .001;
        double power = 0.1;
        while (opModeIsActive() && Math.abs(turnAngle - totalAngle) > .1) {
            currentAngle = readDoubleAngle();
            deltaAngle = currentAngle - lastAngle;
            totalAngle += deltaAngle;
            //missing changing lastAngle
            lastAngle = currentAngle;
            power += increment;

            if (Math.abs(turnAngle - totalAngle) < 20) {
                increment = Math.abs(increment) * -1;
            }

            if (power > speed){
                power = speed;
            }
            else if (power < .1){
                power = .1;
            }
            if (turnAngle-totalAngle > 0){
                robot.frontLeftDrive.setPower(power);
                robot.frontRightDrive.setPower(-power);
                robot.rearLeftDrive.setPower(power);
                robot.rearRightDrive.setPower(-power);
            } else {
                robot.frontLeftDrive.setPower(-power);
                robot.frontRightDrive.setPower(power);
                robot.rearLeftDrive.setPower(-power);
                robot.rearRightDrive.setPower(power);
            }
            telemetry.addData("total angle", totalAngle);
            telemetry.addData("delta angle", deltaAngle);
            telemetry.addData("last angle", lastAngle);
            telemetry.addData("current angle", currentAngle);
            telemetry.update();


        }
        robot.frontLeftDrive.setPower (0);
        robot.frontRightDrive.setPower(0);
        robot.rearLeftDrive.setPower(0);
        robot.rearRightDrive.setPower(0);

    }


    public String readAngle() {
        angleControl = robot.imuControl.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        angleExpansion = robot.imuExpansion.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return "Expansion: " + String.valueOf(angleExpansion.firstAngle) + "\nAngle: Control: " + String.valueOf(angleControl.firstAngle);
    }

    public double readDoubleAngle() {
        angleControl = robot.imuControl.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angleControl.firstAngle;
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = robot.VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        robot.vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        robot.tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, robot.vuforia);
        robot.tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}
