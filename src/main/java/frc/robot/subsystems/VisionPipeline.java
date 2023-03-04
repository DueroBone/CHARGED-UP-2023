package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import org.opencv.core.*;
import org.opencv.imgproc.*;


public class VisionPipeline {
	public static final VisionPipeline m_visionPipeline = new VisionPipeline();

	// Outputs
	private Mat hslThresholdOutput = new Mat();
	private ArrayList<MatOfPoint> findContoursOutput = new ArrayList<MatOfPoint>();
	private ArrayList<MatOfPoint> filterContoursOutput = new ArrayList<MatOfPoint>();

	// Load Native OpenCV Library
	static {
		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
	}

	//Primary Output Updater:
	public void process(Mat source0) {
		// HSL Thresholding:
		Mat hslThresholdInput = source0;
		double[] hslThresholdHue = { 76.07913669064747, 180.0 };
		double[] hslThresholdSaturation = { 0.0, 255.0 };
		double[] hslThresholdLuminance = { 116.68773772511348, 252.44313340856658 };
		hslThreshold(hslThresholdInput, hslThresholdHue, hslThresholdSaturation, hslThresholdLuminance,
				hslThresholdOutput);

		// Find Contours:
		Mat findContoursInput = hslThresholdOutput;
		boolean findContoursExternalOnly = false;
		findContours(findContoursInput, findContoursExternalOnly, findContoursOutput);

		// Filter Contours:
		ArrayList<MatOfPoint> filterContoursContours = findContoursOutput;
		double filterContoursMinArea = 60.0;
		double filterContoursMinPerimeter = 0.0;
		double filterContoursMinWidth = 20.0;
		double filterContoursMaxWidth = 50.0;
		double filterContoursMinHeight = 15.0;
		double filterContoursMaxHeight = 800.0;
		double[] filterContoursSolidity = { 64.74820143884892, 100 };
		double filterContoursMaxVertices = 1000000.0;
		double filterContoursMinVertices = 0.0;
		double filterContoursMinRatio = 0.0;
		double filterContoursMaxRatio = 1000.0;
		filterContours(filterContoursContours, filterContoursMinArea, filterContoursMinPerimeter,
				filterContoursMinWidth, filterContoursMaxWidth, filterContoursMinHeight, filterContoursMaxHeight,
				filterContoursSolidity, filterContoursMaxVertices, filterContoursMinVertices, filterContoursMinRatio,
				filterContoursMaxRatio, filterContoursOutput);
	}

	public Mat hslThresholdOutput() {
		return hslThresholdOutput;
	}

	public ArrayList<MatOfPoint> findContoursOutput() {
		return findContoursOutput;
	}

	public ArrayList<MatOfPoint> filterContoursOutput() {
		return filterContoursOutput;
	}

	private void hslThreshold(Mat input, double[] hue, double[] sat, double[] lum,
			Mat out) {
		Imgproc.cvtColor(input, out, Imgproc.COLOR_BGR2HLS);
		Core.inRange(out, new Scalar(hue[0], lum[0], sat[0]),
				new Scalar(hue[1], lum[1], sat[1]), out);
	}

	private void findContours(Mat input, boolean externalOnly,
			List<MatOfPoint> contours) {
		Mat hierarchy = new Mat();
		contours.clear();
		int mode;
		if (externalOnly) {
			mode = Imgproc.RETR_EXTERNAL;
		} else {
			mode = Imgproc.RETR_LIST;
		}
		int method = Imgproc.CHAIN_APPROX_SIMPLE;
		Imgproc.findContours(input, contours, hierarchy, mode, method);
	}

	private void filterContours(List<MatOfPoint> inputContours, double minArea,
			double minPerimeter, double minWidth, double maxWidth, double minHeight, double maxHeight,
			double[] solidity, double maxVertexCount, double minVertexCount, double minRatio, double maxRatio,
			List<MatOfPoint> output) {
		final MatOfInt hull = new MatOfInt();
		output.clear();
		// Perform Requested Operations:
		for (int i = 0; i < inputContours.size(); i++) {
			final MatOfPoint contour = inputContours.get(i);
			final Rect bb = Imgproc.boundingRect(contour);
			if (bb.width < minWidth || bb.width > maxWidth)
				continue;
			if (bb.height < minHeight || bb.height > maxHeight)
				continue;
			final double area = Imgproc.contourArea(contour);
			if (area < minArea)
				continue;
			if (Imgproc.arcLength(new MatOfPoint2f(contour.toArray()), true) < minPerimeter)
				continue;
			Imgproc.convexHull(contour, hull);
			MatOfPoint mopHull = new MatOfPoint();
			mopHull.create((int) hull.size().height, 1, CvType.CV_32SC2);
			for (int j = 0; j < hull.size().height; j++) {
				int index = (int) hull.get(j, 0)[0];
				double[] point = new double[] { contour.get(index, 0)[0], contour.get(index, 0)[1] };
				mopHull.put(j, 0, point);
			}
			final double solid = 100 * area / Imgproc.contourArea(mopHull);
			if (solid < solidity[0] || solid > solidity[1])
				continue;
			if (contour.rows() < minVertexCount || contour.rows() > maxVertexCount)
				continue;
			final double ratio = bb.width / (double) bb.height;
			if (ratio < minRatio || ratio > maxRatio)
				continue;
			output.add(contour);
		}
	}
}