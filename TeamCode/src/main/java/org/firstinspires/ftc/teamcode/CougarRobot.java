package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

//This class will act as the blueprint behind the robot, and will be used in both Auto and TeleOp
public class CougarRobot {

    /** Configuration Notes: CenterStage
     * Port 00: frontLeft
     * Port 01: frontRight
     * Port 02: backLeft
     * Port 03: backRight
     */

    //Drive Train Motors
    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;

    public double botHeading;

    //Appendage Motors - Will come later on
    public DcMotor intakeMotor;
    public DcMotor conveyorMotor;
    public DcMotor shooterMotor;
    public CRServo shooterServo;

    //Hand Servos/Motors - Will come later on
    public CRServo handServo;
    public DcMotor armMotor;

    public IMU imu;
    HardwareMap hwMap;


    public void init(HardwareMap ahwMap) {

        hwMap = ahwMap;

        frontLeft = hwMap.get(DcMotor.class, "frontLeft");
        frontRight = hwMap.get(DcMotor.class, "frontRight");
        backLeft = hwMap.get(DcMotor.class, "backLeft");
        backRight = hwMap.get(DcMotor.class, "backRight");

        imu = hwMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);


        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
        botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

        public void moveRobot ( double leftStickY, double leftStickX, double rightStickX){
            /**
             * Wheel powers calculated using gamepad 1's inputs leftStickY, leftStickX, and rightStickX
             * **/
            /*

            double topLeftPower = leftStickY + leftStickX - rightStickX;
            double bottomLeftPower = leftStickY - leftStickX - rightStickX; // +
            double topRightPower = leftStickY - leftStickX + rightStickX;
            double bottomRightPower = leftStickY + leftStickX + rightStickX; // -
             */

            double topLeftPower = leftStickY - rightStickX + leftStickX;
            double bottomLeftPower = leftStickY + rightStickX + leftStickX; // +
            double topRightPower = leftStickY + rightStickX - leftStickX;
            double bottomRightPower = leftStickY - rightStickX - leftStickX; // -

            /**
             * Sets the wheel's power
             * **/
            frontLeft.setPower(topLeftPower);
            frontRight.setPower(topRightPower);
            backLeft.setPower(bottomLeftPower);
            backRight.setPower(bottomRightPower);
        }

        public void updateHeading(){
            botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        }

        public void moveRobotFC ( double y, double x, double rx){
             // Rotate the movement direction counter to the bots rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.4;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            frontLeft.setPower(frontLeftPower);
            backLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            backRight.setPower(backRightPower);

            if (rx != 0) {
                updateHeading();
            }
        }
        /**
         * This method takes in 1 input : the A button
         * Once the A button is pressed, we set the intake and conveyor motors to max power
         * **/
    /*
    public void transportRings(int speed){
        intakeMotor.setPower(speed);
        conveyorMotor.setPower(speed);
    }

    /**
     * This method takes in 1 input: the X button
     * Once the X button is pressed, we set the shooter motor to max power and the shooter servo to max power
     * **
    public void shootRings(int speed){
        shooterMotor.setPower(speed*0.85);
        shooterServo.setPower(-speed);
    }

    /**
     * This method takes in 2 inputs : left and right trigger
     * Whichever trigger power is greater is the one that will move forwards/backwards
     * **
    public void moveArm(double speed){
        armMotor.setPower(speed);
    }

    /**
     * This method takes in 1 input: right bumper
     * If the right bumper is pressed then the servo hand will loosen else it will tighten
     * **
    public void changeHandPosition(int position){
        handServo.setPower(position);
    }
    */
        /**\
         * These are the strictly autonomous methods which are controlled by the timer and wheel/motor selection
         * There is no gamepad influence on this
         * **/

        /**
         * This autonomous move method allows the robot to move forwards and backwards like a tank drive and is controlled by a boolean isForwards
         * **/
        public void autonomousMotorMove ( double speed){
            frontLeft.setPower(speed);
            frontRight.setPower(speed);
            backLeft.setPower(speed);
            backRight.setPower(speed);
        }

        /**
         * This autonomous strafe method allows the robot to strafe using mecanum wheels and the direction
         * and the direction is specified via a boolean variable
         * **/
        public void autonomousMotorStrafe ( boolean topLeft, boolean bottomLeft, boolean topRight,
        boolean bottomRight){
            if (topLeft) {
                frontLeft.setPower(0);
                backLeft.setPower(0.5);
                frontRight.setPower(0.5);
                backRight.setPower(0);
            } else if (bottomLeft) {
                frontLeft.setPower(-0.5);
                backLeft.setPower(0);
                frontRight.setPower(0);
                backRight.setPower(-0.5);
            } else if (topRight) {
                frontLeft.setPower(0.5);
                backLeft.setPower(0);
                frontRight.setPower(0);
                backRight.setPower(0.5);
            } else if (bottomRight) {
                frontLeft.setPower(0);
                backLeft.setPower(-0.5);
                frontRight.setPower(-0.5);
                backRight.setPower(0);
            }
        }



        public void autonomousMotorTurn ( double right, double left){
            frontLeft.setPower(left);
            backLeft.setPower(right);
            frontRight.setPower(right);
            backRight.setPower(left);
        }


    /*
    public void autonomousMotorShoot(double speed) throws InterruptedException {
        shooterMotor.setPower(speed);
        Thread.sleep(500);
        conveyorMotor.setPower(speed);
        intakeMotor.setPower(speed);
        shooterServo.setPower(-1);
    }

    /**
     * This autonomous servo handle method allows the servo arm to lift or drop a wobble goal depending on a boolean variable
     * **/

        /*
        public void autonomousServoHandle ( boolean drop){
            ElapsedTime runtime = new ElapsedTime();
            if (drop) {
                while (runtime.seconds() < 1.5) {
                    armMotor.setPower(-0.5);
                }
                while (runtime.seconds() < 3.0) {
                    handServo.setPower(0);
                }
            } else {
                while (runtime.seconds() < 1.5) {
                    handServo.setPower(0.5);
                }
                while (runtime.seconds() < 1.5) {
                    armMotor.setPower(0.1);
                }
            }

        }*/

    }
