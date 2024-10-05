package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class VSliderSystem {
    private static volatile DcMotorEx sliderMtr;
    private static volatile Servo clawTilt;
    private static volatile Servo clawPan;

    private static volatile Servo clawLeft;
    private static volatile Servo clawRight;

    public static volatile DistanceSensor leftSensor;
    public static volatile DistanceSensor rightSensor;
    public static volatile DistanceSensor middleSensor;

    private static volatile TouchSensor vTouch;
    private static volatile LinearOpMode mainTask;
    private static volatile Telemetry telemetry;



    private static final double SERVO_RES = 0.0037;

    //if you switch servo, may need modify these values
    private static final double TILT_ZERO = 0.5;
    private static final double PAN_ZERO = 0.505;
    private static final double CLAW_ZERO = 0.5;

    private static final double TILT_H_LM = 90;
    private static final double TILT_L_LM = -135;
    private static final double CLAW_L_LM = -20;
    private static final double CLAW_H_LM = 80;
    private static final double PAN_H_LM = 75;
    private static final double PAN_L_LM = -80;;



    private static final double SLIDER_PPR = 384.5;  //435RPM
    private static final double PULL_WHEEL_DIA = 0.03565;  //unit : m
    private static final double PULLY_WIRE_DIA = 0.00075;  //unit: m
    private static final int SLIDER_STAGE = 2;
    private static final double SLIDER_STAGE_LEN = 0.244; // unit: m
    public static final int SLIDER_LOWLM = 0;

    public static  final int SLIDER_MAX = (int)(SLIDER_STAGE * SLIDER_STAGE_LEN / (Math.PI * (PULL_WHEEL_DIA + PULLY_WIRE_DIA * 3)) * SLIDER_PPR);  // 2400  2300
    public static final double SLIDER_LEN_RES = (Math.PI * PULL_WHEEL_DIA) / SLIDER_PPR;  // 2400  2300



    private static volatile double sliderLen = 0.01;
    private static final double SLIDER_STEP = 0.01;


    public static final double TILT_DROP_ANGLE = 135.0;
    public static final double TILT_INI_ANGLE = -88;  //0.88
    public static final double TILT_GET_CONE_POS = -27;   //0.4;

    public static final double PAN_DROP_ANGLE = 0.33;
    public static final double PAN_INI_ANGLE = 0;
    public static final double PAN_GET_CONE_POS = 0.33;


    public static final double CLAW_CLOSE_ANGLE = 75.8;
    public static final double CLAW_OPEN_ANGLE = 20;   //10
    private static final double RIGHT_CLAW_OFFSET = 0.0;


    private static final double TILT_STEP = 5.0;      //degree
    private static final double PAN_STEP = 5.0;       //degree


    private static volatile double tiltAngle = TILT_INI_ANGLE;
    private static volatile double panAngle = PAN_INI_ANGLE;
    private static volatile double clawAngle = CLAW_CLOSE_ANGLE - 10;  //degree

    private static volatile int clawDelayCnt = 0;

    private static volatile boolean scanFlag = false;
    private static volatile ScanJunctionThread scanJunction = null;

    private static volatile double sliderBtnCnt = 0;
    private static volatile double sliderPwr = 0;
    private static volatile double sliderMinPwr = 0.3;
    private static volatile int sliderPos = 0;


    public VSliderSystem(LinearOpMode mainTask, HardwareMap hwMap, Telemetry telemetry, Boolean auto)
    {
        this.mainTask = mainTask;
        this.telemetry = telemetry;
        sliderMtr = hwMap.get(DcMotorEx.class, "vSlider");
        sliderMtr.setDirection(DcMotorSimple.Direction.REVERSE);
        sliderMtr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sliderMtr.setPositionPIDFCoefficients(4.0);

        clawTilt = hwMap.get(Servo.class, "vcTilt");
        clawPan = hwMap.get(Servo.class, "vcPan");
        clawLeft = hwMap.get(Servo.class, "vcLeft");
        clawRight = hwMap.get(Servo.class, "vcRight");

        clawTilt.setDirection(Servo.Direction.REVERSE);

        clawPan.setDirection(Servo.Direction.FORWARD);

        clawLeft.setDirection(Servo.Direction.FORWARD);
        clawRight.setDirection(Servo.Direction.REVERSE);

        vTouch = hwMap.get(TouchSensor.class, "vTouch");
        leftSensor = hwMap.get(DistanceSensor.class, "vLeftDis");
        rightSensor = hwMap.get(DistanceSensor.class, "vRightDis");
        middleSensor = hwMap.get(DistanceSensor.class, "vMiddleDis");



        if (auto) {
            tiltAngle = -108.11;
            setTiltAngle(tiltAngle);
            panAngle = 30.48;
            setPanAngle(panAngle);
            setClawAngle(65);
        }
        else{
            tiltAngle = 0;
            setTiltAngle(tiltAngle);
            panAngle = 0;
            setPanAngle(panAngle);
            clawAngle = CLAW_CLOSE_ANGLE;
            setClawAngle(clawAngle);
        }

        sliderLen = getSliderLen();

        if (auto) {
            sliderMtr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            sliderMtr.setPower(-0.5);
            while (!mainTask.isStopRequested() && !vTouch.isPressed()) {
                mainTask.sleep(10);

            }
            sliderMtr.setPower(0);
            sliderMtr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            sliderLen = 0.01;  // if height allowed, if not, set to 0
            sliderLenCtrl(sliderLen, sliderMinPwr);
        }
        else{
            sliderLen = getSliderLen();
            sliderLenCtrl(sliderLen, sliderMinPwr);
        }


    }
    private static void sliderHoldPos(double pwr){

        sliderMtr.setTargetPosition(sliderMtr.getCurrentPosition());
        sliderMtr.setPower(pwr);
        sliderMtr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public static void joystickCtrl(Gamepad gamepad1, Gamepad gamepad2)
    {
        if (gamepad2.right_trigger > 0.5)
        {
            if (sliderMtr.getCurrentPosition() >= 30) {
                sliderBtnCnt += 0.05;
                sliderBtnCnt = sliderBtnCnt > (1- sliderMinPwr) ?   (1- sliderMinPwr) : sliderBtnCnt;
                sliderPwr = -(sliderMinPwr + sliderBtnCnt);
                if (getSliderLen() < 0.1)  sliderPwr = -sliderMinPwr;
                if (sliderMtr.getMode() != DcMotor.RunMode.RUN_USING_ENCODER){
                    sliderMtr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
                sliderMtr.setPower(sliderPwr);
            }
            else
            {
                sliderLenCtrl(0.01, 0.3);
                sliderBtnCnt = 0;
            }
        }
        else if(gamepad2.right_bumper){
            if (sliderMtr.getCurrentPosition() <= SLIDER_MAX) {
                sliderBtnCnt += 0.05;
                sliderBtnCnt = sliderBtnCnt > (1 - sliderMinPwr) ? (1 - sliderMinPwr) : sliderBtnCnt;
                sliderPwr = (sliderMinPwr + sliderBtnCnt);
                if (sliderMtr.getCurrentPosition() > SLIDER_MAX - 200) sliderPwr = sliderMinPwr;

                if (sliderMtr.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
                    sliderMtr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
                sliderMtr.setPower(sliderPwr);
            }
            else{
                sliderHoldPos(sliderMinPwr);
                sliderBtnCnt = 0;
            }

        }
        else{
            if (sliderBtnCnt != 0){
                sliderHoldPos(sliderMinPwr);
                sliderBtnCnt = 0;
            }
        }


        //right joystick control vSlider tilt and pan
        if (Math.abs(gamepad2.right_stick_y) > 0.3){
            tiltAngle =  tiltAngle - gamepad2.right_stick_y * TILT_STEP;
            tiltAngle = tiltAngle > TILT_H_LM ? TILT_H_LM : tiltAngle;
            tiltAngle = tiltAngle < TILT_L_LM ? TILT_L_LM : tiltAngle;
            setTiltAngle(tiltAngle);
        }

        if (Math.abs(gamepad2.right_stick_x) > 0.3)
        {
            panAngle =  panAngle + gamepad2.right_stick_x * PAN_STEP;
            panAngle = panAngle > PAN_H_LM ? PAN_H_LM : panAngle;
            panAngle = panAngle < PAN_L_LM ? PAN_L_LM : panAngle;
            setPanAngle(panAngle);
        }


        if (gamepad2.right_stick_button && clawDelayCnt == 0)
        {

            if (getClawAngle() <= CLAW_OPEN_ANGLE + 10)
            {
                clawAngle = CLAW_CLOSE_ANGLE;
                setClawAngle(clawAngle);
                clawDelayCnt  = 5;
            }
            else{
                clawAngle = CLAW_OPEN_ANGLE;
                setClawAngle(clawAngle);
                clawDelayCnt  = 5;
            }
        }else{
            clawDelayCnt --;
            clawDelayCnt = clawDelayCnt < 0 ? 0 : clawDelayCnt;
        }


    }

    private static void sliderCtrl(int extendPos) {
       sliderCtrl(extendPos, 1.0);
    }

    private static void sliderCtrl(int extendPos, double pwr) {
        sliderPos = extendPos;
        sliderPos = sliderPos > SLIDER_MAX ?  SLIDER_MAX: sliderPos;
        sliderPos = sliderPos < SLIDER_LOWLM ? SLIDER_LOWLM: sliderPos;
        sliderMtr.setTargetPosition(sliderPos);
        sliderMtr.setPower(pwr);
        sliderMtr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    private static void setTiltPos(double pos)
    {
        pos = pos < 0 ? 0 : pos;
        pos = pos > 1.0 ? 1.0: pos;
        clawTilt.setPosition(pos);

    }

    private static void setPanPos(double pos)
    {
        pos = pos < 0 ? 0 : pos;
        pos = pos > 1.0 ? 1.0: pos;

        clawPan.setPosition(pos);
    }

    private static void setClawPosition(double pos)
    {
        pos = pos < 0 ? 0 : pos;
        pos = pos > 1.0 ? 1.0: pos;
        clawLeft.setPosition(pos);
        clawRight.setPosition(pos + RIGHT_CLAW_OFFSET);
    }


    public static void sliderLenCtrl(double len, double pwr){
        int pos = (int)(len / SLIDER_LEN_RES);
        pos = pos > SLIDER_MAX ?  SLIDER_MAX: pos;
        pos = pos < SLIDER_LOWLM ? SLIDER_LOWLM: pos;
        sliderLen = pos * SLIDER_LEN_RES;
        sliderCtrl(pos, pwr);;

    }

    public static double getSliderLen(){
        double len = sliderMtr.getCurrentPosition() * SLIDER_LEN_RES;
        return len;
    }


    public static void setTiltAngle(double angle){
        angle = angle > TILT_H_LM ? TILT_H_LM : angle;
        angle = angle < TILT_L_LM ? TILT_L_LM : angle;
        tiltAngle = angle;
        double pos = angle * SERVO_RES + TILT_ZERO;
        setTiltPos(pos);

    }

    public static void setPanAngle(double angle){
        angle = angle > PAN_H_LM ? PAN_H_LM : angle;
        angle = angle < PAN_L_LM ? PAN_L_LM: angle;
        panAngle = angle;
        double pos  = angle * SERVO_RES + PAN_ZERO;
        setPanPos(pos);

    }

    public static void setClawAngle(double angle){
        angle = angle > CLAW_H_LM ? CLAW_H_LM : angle;
        angle = angle < CLAW_L_LM ? CLAW_L_LM : angle;
        double pos = angle * SERVO_RES + CLAW_ZERO;
        setClawPosition(pos);
    }

    public static double getTiltAngle(){
        double angle = (clawTilt.getPosition() - TILT_ZERO) / SERVO_RES;
        return angle;
    }

    public static double getPanAngle(){
        double angle = (clawPan.getPosition() - PAN_ZERO) / SERVO_RES;
        return angle;
    }

    public static double getClawAngle(){
        double angle = (clawLeft.getPosition() - CLAW_ZERO) / SERVO_RES;
        return angle;
    }

    public static void sliderVelCtrl(double pwr)
    {
        if (sliderMtr.getCurrentPosition() < SLIDER_MAX && sliderMtr.getCurrentPosition() > SLIDER_LOWLM){
            sliderMtr.setPower(pwr);
            sliderMtr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public static void enableTouchCheck()
    {
        ResetCascade resetCascade = new VSliderSystem.ResetCascade();
        resetCascade.start();
    }

    private static class ResetCascade extends Thread
    {
        public boolean runningFlag = false;
        public boolean resetFlag = false;

        public ResetCascade()
        {
            runningFlag = true;
        }

        @Override
        public void run()
        {
            while(!mainTask.opModeIsActive() && runningFlag)
            {
                if(vTouch.isPressed() && !resetFlag)
                {
                    sliderMtr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    sliderLen = 0.01;
                    sliderLenCtrl(sliderLen, 0.3);
                    resetFlag = true;
                    telemetry.addLine("HSlider Reset slider-----------------");
                    telemetry.update();
                }

                if (Math.abs(sliderMtr.getCurrentPosition()) > 100){
                    resetFlag = false;
                }
                // check the position is reached or not, 2 is too small?
                if (Math.abs(sliderMtr.getCurrentPosition() - sliderPos) < 2){
                    sliderCtrl(sliderPos, 0.3);
                }
                mainTask.sleep(100);
            }
        }
    }

    public void scanToFindJunction()
    {
        if (scanJunction == null){
            scanJunction = new ScanJunctionThread();
            scanFlag = true;
            scanJunction.start();
        }

    }


    public void stopScan(){
        if (scanJunction != null){
            scanJunction.stopScan();
        }
        scanJunction = null;

    }

    public boolean isFoundJunction()
    {
        if (scanJunction != null){
            return scanJunction.isFoundJunction();
        }
        else{
            return false;
        }
    }


    private static class ScanJunctionThread extends Thread
    {
        private static volatile boolean runningFlag = false;
        private static volatile boolean resetFlag = false;

        private static volatile double leftDis = 2000;
        private static volatile double rightDis = 2000;
        private static volatile double middleDis = 2000;
        private static final double DETECT_DIS = 300; // 50cm = 0.5m

        private static volatile double scanServoAngle = 0;
        private static boolean foundJunction = false;

        public ScanJunctionThread()
        {
            scanServoAngle = getPanAngle();
            runningFlag = true;
            foundJunction = false;
            scanFlag = true;
        }

        @Override
        public void run()
        {
            double ctrlValue = 0;
            scanServoAngle = getPanAngle();
            double scanServoStep = 10;
            boolean scanDir = true;
            double scanStopAngle = 50;
            // now scan +/- 30 degree
            double scanHigh = scanServoAngle + 30;
            scanHigh = scanHigh > PAN_H_LM ? PAN_H_LM : scanHigh;

            double scanLow = scanServoAngle - 30;
            scanLow = scanLow < PAN_L_LM ? PAN_L_LM : scanLow;

            while(mainTask.opModeIsActive() && runningFlag )
            {
                leftDis = leftSensor.getDistance(DistanceUnit.MM) ;
                middleDis = middleSensor.getDistance(DistanceUnit.MM) ;
                rightDis = rightSensor.getDistance(DistanceUnit.MM) ;

                if (leftDis > DETECT_DIS && rightDis > DETECT_DIS && middleDis > DETECT_DIS){
                    if (scanDir) {
                        scanServoAngle += scanServoStep;
                        if (scanServoAngle > scanHigh) {
                            scanServoAngle = scanHigh;
                            scanDir = false;
                        }
                    } else {
                        scanServoAngle -= scanServoStep;
                        if (scanServoAngle < scanLow) {
                            scanServoAngle = scanLow;
                            scanDir = true;

                        }
                    }
                }
                else  if (leftDis< middleDis && leftDis < DETECT_DIS ){
                   scanServoAngle = scanServoAngle - 8;
                   scanDir = false;

                }else if(rightDis < middleDis && rightDis < DETECT_DIS){
                   scanServoAngle = scanServoAngle + 8;
                    scanDir = true;
                }
                else if(rightDis > middleDis + 50 && rightDis < DETECT_DIS){
                    scanServoAngle = scanServoAngle + 2;
                    scanDir = true;
                }
                else if(leftDis > middleDis + 50 && leftDis < DETECT_DIS){
                    scanServoAngle = scanServoAngle - 2;
                    scanDir = true;
                }
                setPanAngle(scanServoAngle);
                if(middleDis + 200 < leftDis && middleDis + 200 < rightDis && middleDis < DETECT_DIS)
                {
                    foundJunction = true;
                }

                mainTask.sleep(50);
            }
            runningFlag = false;
        }

        public static boolean isFoundJunction(){
            return foundJunction;
        }
        public static void stopScan(){
            runningFlag = false;
            scanFlag = false;
        }
    }


}
