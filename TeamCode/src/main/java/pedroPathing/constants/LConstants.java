package pedroPathing.constants;

import com.pedropathing.localization.*;
import com.pedropathing.localization.constants.*;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class LConstants {
    static {
//        OTOSConstants.useCorrectedOTOSClass = false;
//        OTOSConstants.hardwareMapName = "otos";
//        OTOSConstants.linearUnit = DistanceUnit.INCH;
//        OTOSConstants.angleUnit = AngleUnit.RADIANS;
//        OTOSConstants.offset = new SparkFunOTOS.Pose2D(2.851732283464567, 0, - Math.PI / 2);//Math.PI / 2///
//        OTOSConstants.linearScalar = 0.99345;
//        OTOSConstants.angularScalar = 0.9745;

        PinpointConstants.forwardY = 0;
        PinpointConstants.strafeX = 4.9;
        PinpointConstants.distanceUnit = DistanceUnit.INCH;
        PinpointConstants.hardwareMapName = "pinpoint";
        PinpointConstants.useYawScalar = false;
        PinpointConstants.yawScalar = 1.0;
        PinpointConstants.useCustomEncoderResolution = false;
        PinpointConstants.encoderResolution = GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;
        PinpointConstants.customEncoderResolution = 13.26291192;
        PinpointConstants.forwardEncoderDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;
        PinpointConstants.strafeEncoderDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;
    }
}




