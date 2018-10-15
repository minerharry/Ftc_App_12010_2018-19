package org.firstinspires.ftc.teamcode.vision.dogeCVmodifications;

import com.disnodeteam.dogecv.scoring.DogeCVScorer;

import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.imgproc.Imgproc;

public class RoundnessScorer extends DogeCVScorer {
    public double weight = 0.05;
    public RoundnessScorer(double weight)
    {
        this.weight = weight;
    }
    @Override
    public double calculateDifference(MatOfPoint contours)
    {
        MatOfPoint2f floatVer = new MatOfPoint2f( contours.toArray() );
        MatOfPoint2f approx = new MatOfPoint2f();
        Imgproc.approxPolyDP(floatVer, approx,0.01*Imgproc.arcLength(floatVer,true),true);
        return (4-approx.toList().size())*weight;
    }
}
