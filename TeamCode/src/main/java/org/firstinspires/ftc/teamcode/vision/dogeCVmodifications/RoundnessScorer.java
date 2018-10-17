package org.firstinspires.ftc.teamcode.vision.dogeCVmodifications;

import com.disnodeteam.dogecv.scoring.DogeCVScorer;

import org.firstinspires.ftc.teamcode.vision.SilverExampleModified;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.imgproc.Imgproc;

public class RoundnessScorer extends DogeCVScorer {

    public double weight = 0.05;
    public RoundnessScorer(double weight)
    {
        this.weight = weight;
    }

    public double calculateDifference(MatOfPoint contours)
    {
        MatOfPoint2f floatVer = new MatOfPoint2f( contours.toArray() );
        MatOfPoint2f approx = new MatOfPoint2f();
        Imgproc.approxPolyDP(floatVer, approx,0.01*Imgproc.arcLength(floatVer,true),true);
        return (4-approx.toList().size())*weight;
    }

    @Override
    public double calculateScore(Mat input) {
        if(!(input instanceof MatOfPoint)) return Double.MAX_VALUE;
        MatOfPoint contour = (MatOfPoint) input;
        SilverExampleModified.display( "Roundness Evaluating, input size - width: " + input.width() + "height: " + input.height());

        MatOfPoint2f floatVer = new MatOfPoint2f(contour.toArray());
        MatOfPoint2f approx = new MatOfPoint2f();
        Imgproc.approxPolyDP(floatVer, approx,0.01*Imgproc.arcLength(floatVer,true),true);
        SilverExampleModified.display( "Roundness Evalueated, value: " + (4-approx.toList().size())*weight);
        return (4-approx.toList().size())*weight;
    }
}
