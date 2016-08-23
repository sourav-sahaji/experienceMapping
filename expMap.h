#include<stdio.h>
#include<iostream>
#include<fstream>

#include<opencv2/core.hpp>
#include<opencv2/highgui.hpp>
#include<opencv2/imgproc.hpp>

using namespace std;
using namespace cv;

#define RENDER_FREQ			10		// Frequency at which plotting is done
#define EXP_LOOPS			10
#define EXP_CORRECTION		0.5
#define PI					CV_PI


struct link{
	int expId;
	double d;
	double headingRad;
	double facingRad;
};

/*!
@brief A structure for describing an experience in the map.
*/
struct exps{
	double x_m;
	double y_m;
	double facingRad;
	int vtId;
	int numLinks;
	vector<link> links;
};

/*!
@brief The experience mapping class
*/
class expMap{

private:
	double accumDeltaX, accumDeltaY, accumDeltaFacing;
	int numExps, currentExpId, previousExpId;
	Mat expHistory;

public:
	vector<exps> expsVec;

	expMap(): accumDeltaX(0), accumDeltaY(0), accumDeltaFacing(0), numExps(0), currentExpId(0), previousExpId(0) 
	{
		exps exp = {0, 0, PI, 0, 0};
		expsVec.push_back(exp);
	};
	
	double adjustAngle360(double angle);
	double adjustAngle180(double angle);
	double getSignedDeltaRad(double angle1, double angle2);
	void createNewExp(int currentExpId, int numExps, int vtId);
	void processExp(int vtId, double vTrans, double vRot);

};


/*!
@brief Adjusts the angle between 0 to 2*pi radians
*/
double expMap::adjustAngle360(double angle)
{
	while(angle < 0)
		angle += 2*PI;
	while(angle >= 2*PI)
		angle -= 2*PI;

	return angle;
}


/*!
@brief Adjusts the angle between -pi to pi radians
*/
double expMap::adjustAngle180(double angle)
{
	while(angle > PI)
		angle -= 2*PI;
	while(angle <= -PI)
		angle += 2*PI;

	return angle;
}


/*!
@breif Get the signed delta angle from angle1 to angle2 handling the wrap from 2*pi to 0.
*/
double expMap::getSignedDeltaRad(double angle1, double angle2)
{
	double retAngle;

	double dir = adjustAngle180(angle2 - angle1);

	double deltaAngle = abs( adjustAngle360(angle1) - adjustAngle360(angle2) );

	if( deltaAngle < (2*PI - deltaAngle) )
	{
		if(dir > 0)
			retAngle = deltaAngle;
		else
			retAngle = -deltaAngle;
	}
	else
	{
		if(dir > 0)
			retAngle = 2*PI - deltaAngle;
		else
			retAngle = -(2*PI - deltaAngle);
	}

	return retAngle;
}

/*!
@brief Creates a new experience
*/
void expMap::createNewExp(int currentExpId, int numExps, int vtId)
{
	// Add link information to the current experience for the new experience
	struct link newLink;
	newLink.expId = numExps;
	newLink.d = sqrt( pow(accumDeltaX,2) + pow(accumDeltaY,2) );
	newLink.headingRad = atan2( accumDeltaY, accumDeltaX );
	newLink.facingRad = accumDeltaFacing;

	expsVec[currentExpId].links.push_back(newLink);
	expsVec[currentExpId].numLinks++;

	// Create new experience which will have no links to begin with
	exps exp;
	exp.vtId = vtId;
	exp.x_m = expsVec[currentExpId].x_m	+ cos( expsVec[currentExpId].facingRad ) * accumDeltaX - sin( expsVec[currentExpId].facingRad ) * accumDeltaY;
	exp.y_m = expsVec[currentExpId].y_m	+ sin( expsVec[currentExpId].facingRad ) * accumDeltaX + cos( expsVec[currentExpId].facingRad ) * accumDeltaY;
	exp.facingRad = adjustAngle180( expsVec[currentExpId].facingRad + accumDeltaFacing );
	exp.numLinks = 0;

	expsVec.push_back(exp);
}


/*!
@brief Processes the experience using the data available.
*/
void expMap::processExp(int vtId, double vTrans, double vRot)
{
	// Integrate the delta x, y and facing
	accumDeltaFacing = adjustAngle180(accumDeltaFacing + vRot);
	accumDeltaX += vTrans * cos(accumDeltaFacing);
	accumDeltaY += vTrans * sin(accumDeltaFacing);

	// If vt is new, create a new experience
	if(vtId > numExps)
	{
		numExps++;

		// Create a new experience
		createNewExp(currentExpId,numExps,vtId);

		previousExpId = currentExpId;
		currentExpId = numExps;

		accumDeltaX = 0;
		accumDeltaY = 0;
		accumDeltaFacing = 0;
	}
	// Else, if vt has changed (but isn't new) search for the matching experience
	else
	{
		int matchedExpId = vtId;

		if(matchedExpId != previousExpId)
		{
			// Check if the previous experience already has a link to the current experience
			bool linkExists = (currentExpId == matchedExpId);

			if(!linkExists)
			{
				// Search for the link
				for(int j1=0; j1<expsVec[currentExpId].numLinks; j1++)
				{
					if(expsVec[currentExpId].links[j1].expId == matchedExpId)
					{
						linkExists = true;
						break;
					}
				}

				// If no link is found yet, then create a link between current experience and the experience for current vt
				if(!linkExists)
				{
					struct link newLink;
					newLink.expId = matchedExpId;
					newLink.d = sqrt( pow(accumDeltaX,2) + pow(accumDeltaY,2) );
					newLink.headingRad = atan2( accumDeltaY, accumDeltaX );
					newLink.facingRad = accumDeltaFacing;

					expsVec[currentExpId].links.push_back(newLink);
					expsVec[currentExpId].numLinks++;
				}
			}

			accumDeltaX = 0;
			accumDeltaY = 0;
			accumDeltaFacing = 0;
		}

		previousExpId = currentExpId;
		currentExpId = vtId;
	}

	// Do the experience map correction iteratively for all the links in all the experiences
	for(int i1=0; i1<EXP_LOOPS; i1++)
	{
		for(int expID=0; expID<=numExps; expID++)
		{
			for(int linkID=0; linkID<expsVec[expID].numLinks; linkID++)
			{
				// Experience 0 has link to experience 1
				int e0 = expID;
				int e1 = expsVec[expID].links[linkID].expId;

				// Work out where e0 thinks e1 (x,y) should be based on the stored link information
				double lx = expsVec[e0].x_m + expsVec[e0].links[linkID].d * cos( expsVec[e0].facingRad + expsVec[e0].links[linkID].headingRad);
				double ly = expsVec[e0].y_m + expsVec[e0].links[linkID].d * sin( expsVec[e0].facingRad + expsVec[e0].links[linkID].headingRad);
			
				// Correct e0 and e1 (x,y) by equal and opposite amounts, a 0.5 correction parameter means that 
				// e0 and e1 will be fully corrected based on e0's link information.
				expsVec[e0].x_m += ( ( expsVec[e1].x_m - lx ) *  EXP_CORRECTION );
				expsVec[e0].y_m += ( ( expsVec[e1].y_m - ly ) *  EXP_CORRECTION );
				expsVec[e1].x_m -= ( ( expsVec[e1].x_m - lx ) *  EXP_CORRECTION );
				expsVec[e1].y_m -= ( ( expsVec[e1].y_m - ly ) *  EXP_CORRECTION );

				// Determine the angle between where e0 thinks e1 is facing
				// should be based on the link information
				double df = getSignedDeltaRad( (expsVec[e0].facingRad + expsVec[e0].links[linkID].facingRad), expsVec[e1].facingRad );

				// Correct e0 and e1 facing by equal but opposite amounts, a 0.5 correction parameter means that
				// e0 and e1 will be fully corrected based on e0's link information.
				expsVec[e0].facingRad = adjustAngle180( expsVec[e0].facingRad + df * EXP_CORRECTION );
				expsVec[e1].facingRad = adjustAngle180( expsVec[e1].facingRad - df * EXP_CORRECTION );
			}
		}
	}

	expHistory.push_back(currentExpId);
}


/*!
@brief Reads CSV file into OpenCV matrix
@return Returns the Matrix
*/
Mat readCSV(ifstream& inputFile)
{
	Mat dataMat;
	string buffer;
	while(getline(inputFile,buffer))
	{
		stringstream ss(buffer);
		string elem;
		Mat rowMat;
		while(getline(ss,elem,','))
		{
			rowMat.push_back(atof(&elem[0]));
		}

		if(dataMat.empty())
		{
			dataMat.push_back(rowMat);
			dataMat = dataMat.t();
		}
		else
		{
			vconcat(dataMat,rowMat.t(),dataMat);
		}
	}
	return dataMat;
}


/*!
@brief Plots the experience on an image.
*/
void plotData(vector<Point2f> expPoints, Mat& plotImg)
{
	// Set the image size to contain the plot (major container)
	plotImg = Mat(400,400,CV_8UC3,Scalar::all(255));

	// Set the margins as empty space around the plot (around minor container)
	int verticalMargin = 10;
	int horizMargin = 10;

	// Calculate the multiplying factor for translation of the values to be plotted
	int mulFactorX = plotImg.cols - 2 * horizMargin;
	int mulFactorY = plotImg.rows - 2 * verticalMargin;

	// Draw the minor container
	rectangle(plotImg,Rect(horizMargin,verticalMargin,mulFactorX,mulFactorY),Scalar(0,0,0),1,CV_AA);

	// Set the range of points to be plotted within 0 to 1
	Mat xPts, yPts;
	for(int i1=0; i1<expPoints.size(); i1++)
	{
		xPts.push_back(expPoints[i1].x);
		yPts.push_back(expPoints[i1].y);
	}

	double minVal, maxVal;
	minMaxLoc(xPts, &minVal, &maxVal);
	xPts.convertTo(xPts, CV_32FC1, 1.0 / (maxVal - minVal), (-1.0*minVal) / (maxVal - minVal));

	minMaxLoc(yPts, &minVal, &maxVal);
	yPts.convertTo(yPts, CV_32FC1, 1.0 / (maxVal - minVal), (-1.0*minVal) / (maxVal - minVal));

	// Calculate the new points translated to the plot container (major) and plot them
	vector<Point2i> transPoints;			// Translated points
	for(int i1=0; i1<xPts.rows; i1++)
	{
		Point2i p1;
		p1.x = horizMargin + xPts.at<float>(i1) * mulFactorX;
		
		// Invert the y axis
		p1.y = plotImg.rows - (verticalMargin + yPts.at<float>(i1) * mulFactorY);

		// Thickness set to -1 for filled circles
		circle(plotImg,p1,2,Scalar(255,0,0),-1,CV_AA);

		transPoints.push_back(p1);
	}

	// Draw the polygon joining the points in sequential order
	polylines(plotImg,transPoints,false,Scalar(255,0,0),1,CV_AA);

	circle(plotImg,transPoints[transPoints.size()-1],8,Scalar(0,0,255),2,CV_AA);



	// Display the plot
	//imshow("plot",plotImg);
	//int key = waitKey(100);
	//if(key == 27)
	//{
	//	cerr << "Esc key pressed, exiting..." << endl;
	//	exit(-1);
	//}
}

/*!
@brief Reads input data file into OpenCV matrix
@return Returns the Matrix
*/
cv::Mat readDataFile(std::ifstream& inputFile, char delimiter)
{
    cv::Mat dataMat;
    std::string buffer;
    while(std::getline(inputFile,buffer))
    {
        std::stringstream ss(buffer);
        std::string elem;
        cv::Mat rowMat;
        while(std::getline(ss,elem,delimiter))
        {
            rowMat.push_back(atof(&elem[0]));
        }

        if(dataMat.empty())
        {
            dataMat.push_back(rowMat);
            dataMat = dataMat.t();
        }
        else
        {
            cv::vconcat(dataMat,rowMat.t(),dataMat);
        }
    }
    return dataMat;
}
