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
import org.firstinspires.ftc.robotcore.internal.android.dex.Code;

public class HSliderSystem {
    private static volatile DcMotorEx sliderMtr;

    private static volatile Servo clawTiltLeft;
    private static volatile Servo clawTiltRight;
   // public Servo clawRot;
    private static volatile Servo clawLeft;
    private static volatile Servo clawRight;
    private static volatile Servo clawPan;

    public static volatile TouchSensor hTouch;

    public static volatile DistanceSensor leftSensor;
    public static volatile DistanceSensor rightSensor;


    private static volatile LinearOpMode mainTask;
    private static volatile Telemetry telemetry;


    private static final double SERVO_RES = 0.0037;   // 1/ 270
    //if you switch servo, may need modify these values
    private static final double TILT_ZERO = 0.5 - 7 * SERVO_RES;
    private static final double PAN_ZERO = 0.515;
    private static final double CLAW_ZERO = 0.5;

    private static final double TILT_H_LM = 45; // mechanical limitation
    private static final double TILT_L_LM = -104;
    private static final double CLAW_L_LM = -10;
    private static final double CLAW_H_LM = 80;
    private static final double PAN_H_LM = 58;
    private static final double PAN_L_LM = -58;


    private static final double SLIDER_PPR = 145.1 * 1.5;   /// 435 RPM motor 384.5, 1150 RPM motor 145.1, 1.5 gear ratio
    private static final double PULL_WHEEL_DIA = 0.03565;  //unit : m
    private static final double PULLY_WIRE_DIA = 0.00075;  //unit: m
    private static final int SLIDER_STAGE = 5;
    private static final double SLIDER_STAGE_LEN = 0.244; // unit: m
    public static  final int SLIDER_LOWLM = 1;
    public static final int SLIDER_MAX =  1950;   //due to the wire length  calculate (int)(SLIDER_STAGE * SLIDER_STAGE_LEN / (Math.PI * (PULL_WHEEL_DIA + PULLY_WIRE_DIA * 3)) * SLIDER_PPR);  // 2400  2300
    //unit: meter
    public static final double SLIDER_LEN_RES = (Math.PI * PULL_WHEEL_DIA) / SLIDER_PPR;  // 2400  2300

    public static final double TILT_STEP = 5.0;       // degree
    public static final double PAN_STEP = 5.0;        // degree

    // to make sure both side work together
    private static final double RIGHT_TILT_OFFSET = 0;
    private static final double RIGHT_CLAW_OFFSET = 0;


    public static final double CLAW_CLOSE_ANGLE = CLAW_H_LM;

    public static final double CLAW_OPEN_ANGLE = 10;  //10


    public static final double CLAW_INIT_ANGLE = CLAW_CLOSE_ANGLE - 10;

    private static final double TILT_INIT_ANGLE = 131.0;
    private static final double TILT_PICK_ANGLE = 0;
    private static final double TILT_DELIVER_ANGEL = -54.0;

    private static final double PAN_INIT_ANGLE = -26;


    private static volatile double tiltAngle = TILT_INIT_ANGLE;
    private static volatile double clawAngle = CLAW_INIT_ANGLE;
    private static volatile double panAngle = PAN_INIT_ANGLE;

    private static volatile double sliderLen = 0;

    private final double SLIDER_STEP = 0.01;  // 1cm


    private static volatile int clawDelayCnt = 0;

    private static volatile ScanConeThread scanCone = null;
    private static volatile ResetCascadeThread resetCascade = null;
    private static volatile boolean scanFlag = false;

    private static volatile double sliderBtnCnt = 0;
    private static volatile double sliderPwr = 0;
    private static volatile double sliderMinPwr = 0.3;
    private static volatile int sliderPos = 0;


    public HSliderSystem(LinearOpMode mainTask, HardwareMap hwMap, Telemetry telemetry, boolean auto)
    {
        this.mainTask = mainTask;
        this.telemetry = telemetry;
        sliderMtr = hwMap.get(DcMotorEx.class, "hSlider");
        sliderMtr.setDirection(DcMotorSimple.Direction.REVERSE);
        sliderMtr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sliderMtr.setPositionPIDFCoefficients(8.0);

        clawTiltLeft = hwMap.get(Servo.class, "hcTiltLeft");
        clawTiltRight = hwMap.get(Servo.class, "hcTiltRight");

        clawLeft = hwMap.get(Servo.class, "hcLeft");
        clawRight = hwMap.get(Servo.class, "hcRight");
        clawPan = hwMap.get(Servo.class, "hcPan");

        clawTiltLeft.setDirection(Servo.Direction.FORWARD);
        clawTiltRight.setDirection(Servo.Direction.REVERSE);

        clawPan.setDirection(Servo.Direction.FORWARD);

        clawLeft.setDirection(Servo.Direction.REVERSE);
        clawRight.setDirection(Servo.Direction.FORWARD);

        hTouch = hwMap.get(TouchSensor.class, "hTouch");

        leftSensor = hwMap.get(DistanceSensor.class, "hLeftDis");
        rightSensor = hwMap.get(DistanceSensor.class, "hRightDis");



        sliderLen = getSliderLen();


        //reset slider if in auto mode

        if (auto) {
            sliderMtr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            sliderMtr.setPower(-0.8);
            while (!mainTask.isStopRequested() && !hTouch.isPressed()) {
                mainTask.sleep(10);

            }
            sliderMtr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            mainTask.sleep(200);
            sliderHoldPos(0.01);
        }
        else{
            sliderLen = getSliderLen();
            sliderLenCtrl(sliderLen, sliderMinPwr);
        }

        if (auto) {
            tiltAngle = 38;
            setTiltAngle(tiltAngle);
            panAngle = 0;
            setPanAngle(panAngle);
            clawAngle = CLAW_CLOSE_ANGLE;
            setClawAngle(clawAngle);

        }
        else{

            tiltAngle = 0;
            setTiltAngle(tiltAngle);
            clawAngle = CLAW_CLOSE_ANGLE;
            setClawAngle(clawAngle);
            panAngle = 0;
            setPanAngle(panAngle);
        }



    }
    private void setSliderMotorKp( double kp){
        sliderMtr.setPositionPIDFCoefficients(kp);
    }

    private void sliderHoldPos(double pwr){
        sliderPos = sliderMtr.getCurrentPosition();
        sliderMtr.setTargetPosition(sliderPos);
        sliderMtr.setPower(pwr);
        sliderMtr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void joystickCtrl(Gamepad gamepad1, Gamepad gamepad2)
    {
        if (gamepad2.left_trigger > 0.5)
        {
            if (sliderMtr.getCurrentPosition() >= 30 ) {
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
                sliderLenCtrl(0.01, 0.1);
                sliderBtnCnt = 0;
            }
        }
        else if(gamepad2.left_bumper){
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


        //left joystick control vSlider tilt and pan
        if (Math.abs(gamepad2.left_stick_y) > 0.3){
            tiltAngle =  tiltAngle - gamepad2.left_stick_y * TILT_STEP;
            tiltAngle = tiltAngle > TILT_H_LM ? TILT_H_LM : tiltAngle;
            tiltAngle = tiltAngle < TILT_L_LM ? TILT_L_LM : tiltAngle;
            setTiltAngle(tiltAngle);
        }

        if (Math.abs(gamepad2.left_stick_x) > 0.3)
        {
            if (!scanFlag){
                panAngle =  panAngle - gamepad2.left_stick_x * PAN_STEP;
                panAngle = panAngle > PAN_H_LM ? PAN_H_LM : panAngle;
                panAngle = panAngle < PAN_L_LM ? PAN_L_LM : panAngle;
                setPanAngle(panAngle);
            }

        }



        if (gamepad2.left_stick_button && clawDelayCnt == 0)
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

    private void sliderCtrl(int extendPos) {
        sliderCtrl(extendPos, 1.0);

    }

    private void sliderCtrl(int extendPos, double pwr) {
        sliderPos = extendPos;
        sliderPos = sliderPos > SLIDER_MAX ?  SLIDER_MAX: sliderPos;
        sliderPos = sliderPos < SLIDER_LOWLM ? SLIDER_LOWLM: sliderPos;
        sliderMtr.setTargetPosition(sliderPos);
        sliderMtr.setPower(pwr);
        sliderMtr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    private void setTiltPos(double pos)
    {
        pos = pos < 0 ? 0 : pos;
        pos = pos > 1.0 ? 1.0: pos;
        clawTiltLeft.setPosition(pos);
        clawTiltRight.setPosition(pos + RIGHT_TILT_OFFSET);
    }



    private void setClawPosition(double pos)
    {
        pos = pos < 0 ? 0 : pos;
        pos = pos > 1.0 ? 1.0: pos;
        clawLeft.setPosition(pos);
        clawRight.setPosition(pos + RIGHT_CLAW_OFFSET);
    }



    private void setPanPos(double pos)
    {
        pos = pos < 0 ? 0 : pos;
        pos = pos > 1.0 ? 1.0: pos;

        clawPan.setPosition(pos);

    }

    public boolean isATEnd()
    {
        return hTouch.isPressed();
    }

    public void sliderLenCtrl(double len, double pwr){

        int pos = (int)(len / SLIDER_LEN_RES);
        pos = pos > SLIDER_MAX ?  SLIDER_MAX: pos;
        pos = pos < SLIDER_LOWLM ? SLIDER_LOWLM: pos;
        sliderLen = pos  * SLIDER_LEN_RES;
        sliderCtrl(pos, pwr);

    }

    public double getSliderLen(){
        double len = sliderMtr.getCurrentPosition() * SLIDER_LEN_RES;
        return len;
    }


    public void setTiltAngle(double angle){
        angle = angle > TILT_H_LM ? TILT_H_LM : angle;
        angle = angle < TILT_L_LM ?  TILT_L_LM: angle;
        tiltAngle = angle;
        double pos = angle * SERVO_RES + TILT_ZERO;
        setTiltPos(pos);

    }

    public void setPanAngle(double angle){
        angle = angle > PAN_H_LM ? PAN_H_LM : angle;
        angle = angle < PAN_L_LM ? PAN_L_LM : angle;
        panAngle = angle;
        double pos  = angle * SERVO_RES + PAN_ZERO;
        setPanPos(pos);

    }

    public void setClawAngle(double angle){
        angle = angle > CLAW_H_LM ? CLAW_H_LM : angle;
        angle = angle < CLAW_L_LM ? CLAW_L_LM : angle;
        double pos = angle * SERVO_RES + CLAW_ZERO;
        setClawPosition(pos);
    }

    public double getTiltAngle(){
        double angle = (clawTiltLeft.getPosition() - TILT_ZERO) / SERVO_RES;
        return angle;
    }

    public double getPanAngle(){
        double angle = (clawPan.getPosition() - PAN_ZERO) / SERVO_RES;
        return angle;
    }

    public double getClawAngle(){
        double angle = (clawLeft.getPosition() - CLAW_ZERO) / SERVO_RES;
        return angle;
    }


    public void sliderVelCtrl(double pwr)
    {
        if (sliderMtr.getCurrentPosition() < SLIDER_MAX && sliderMtr.getCurrentPosition() > SLIDER_LOWLM){
            sliderMtr.setPower(pwr);
            sliderMtr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }


    public void enableTouchCheck()
    {
        resetCascade = new ResetCascadeThread();
        resetCascade.start();
    }

    private class ResetCascadeThread extends Thread
    {
        public  volatile boolean runningFlag = false;
        public  volatile boolean resetFlag = false;

        public ResetCascadeThread()
        {
            runningFlag = true;
        }

        @Override
        public void run()
        {
            while(!mainTask.opModeIsActive() && runningFlag)
            {
                if(hTouch.isPressed() && !resetFlag)
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


    public void scanToFindCone()
    {
        if (scanCone == null){
            scanCone = new ScanConeThread();
            scanFlag = true;
            scanCone.start();
        }

    }


    public void stopScan(){
        if (scanCone != null){
            scanCone.stopScan();
        }
        scanCone = null;

    }

    public boolean isFoundCone()
    {
        if (scanCone != null){
            return scanCone.isFoundCone();
        }
        else{
            return false;
        }
    }


    private class ScanConeThread extends Thread
    {
        public  volatile boolean runningFlag = false;
        public  volatile boolean resetFlag = false;

        private  double leftDis = 2000;
        private  double rightDis = 2000;
        private  final double DETECT_DIS = 200; // 50cm = 0.5m

        private  double scanServoAngle = 0;
        private  boolean foundCone = false;
        private  double kp = 0.08;
        public ScanConeThread()
        {
            scanServoAngle = getPanAngle();
            runningFlag = true;
            foundCone = false;
            scanFlag = true;
        }

        @Override
        public void run()
        {
            double ctrlValue = 0;
            while(mainTask.opModeIsActive() && runningFlag)
            {
                leftDis = leftSensor.getDistance(DistanceUnit.MM) + 0; //mm
                rightDis = rightSensor.getDistance(DistanceUnit.MM)  + 0; //mm

                if(leftDis > 8000 || rightDis > 8000){
                    foundCone = true;
                }
                else {
                    if (leftDis < DETECT_DIS && rightDis < DETECT_DIS) {
                        ctrlValue = -kp * (leftDis - rightDis);
                        ctrlValue = Math.abs(ctrlValue) > 2 ? Math.signum(ctrlValue) * 2 : ctrlValue;

                        scanServoAngle += ctrlValue;
                        setPanAngle(scanServoAngle);
                        if (Math.abs(leftDis - rightDis) < 10) {
                            foundCone = true;
                        } else {
                            foundCone = false;
                        }
                    }
                }

                mainTask.sleep(20);
            }
            runningFlag = false;
        }

        public boolean isFoundCone(){
            return foundCone;
        }
        public void stopScan(){
            runningFlag = false;
            scanFlag = false;
        }
    }



    public void slowMoveSliderMtr(double pwr)
    {
        sliderMtr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        sliderMtr.setPower(pwr);
    }

    public void stopSliderMtr(){
        sliderMtr.setPower(0);
        sliderMtr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       // mainTask.sleep(200);
    }

    public int getSliderMtrEncoder(){
        return sliderMtr.getCurrentPosition();
    }


}
