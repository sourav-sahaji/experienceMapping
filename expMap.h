#include<stdio.h>
#include<iostream>
#include<fstream>

#include<opencv2\core.hpp>
#include<opencv2\highgui.hpp>

using namespace std;
using namespace cv;

#define EXP_LOOPS			10
#define EXP_CORRECTION		0.5
#define PI					CV_PI


float accumDeltaX = 0, accumDeltaY = 0, accumDeltaFacing = 0;
int numExps = 1, currentExpId = 1, previousExpId = 0, numVt = 1;
Mat expHistory;

struct link{
	int expId;
	float d;
	float headingRad;
	float facingRad;
};

/*!
@brief A class for describing an experience in the map.
*/
class exps{

public:
	float x_m;
	float y_m;
	float facingRad;
	int vtId;
	int numLinks;
	vector<link> links;

	//exps(): x_m(0), y_m(0), facingRad(PI/2.0), vtId(1), numLinks(0) {}

};

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
			rowMat.push_back(float(atof(&elem[0])));
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
@brief Adjusts the angle between 0 to 2*pi radians
*/
float adjustAngle360(float angle)
{
	if(angle < 0)
		angle += 2*PI;
	else if(angle >= 2*PI)
		angle -= 2*PI;

	return angle;
}


/*!
@brief Adjusts the angle between -pi to pi radians
*/
float adjustAngle180(float angle)
{
	if(angle > PI)
		angle -= 2*PI;
	else if(angle <= -PI)
		angle += 2*PI;

	return angle;
}


/*!
@breif Get the signed delta angle from angle1 to angle2 handling the wrap from 2*pi to 0.
*/
float getSignedDeltaRad(float angle1, float angle2)
{
	float retAngle;

	float dir = adjustAngle180(angle2 - angle1);

	float deltaAngle = abs( adjustAngle360(angle1) - adjustAngle360(angle2) );

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
void createNewExp(int currentExpId, int numExps, int vtId, vector<exps>& expsVec)
{
	// Add link information to the current experience for the new experience
	expsVec[currentExpId].numLinks++;
	int linkId = expsVec[currentExpId].numLinks;
	expsVec[currentExpId].links[linkId].expId = numExps;
	expsVec[currentExpId].links[linkId].d = sqrt( pow(accumDeltaX,2) + pow(accumDeltaY,2) );
	expsVec[currentExpId].links[linkId].headingRad = atan2( accumDeltaY, accumDeltaX );
	expsVec[currentExpId].links[linkId].facingRad = accumDeltaFacing;

	// Create new experience which will have no links to begin with
	exps exp;
	exp.vtId = numExps;
	exp.x_m = expsVec[currentExpId].x_m	+ cos( expsVec[currentExpId].facingRad ) * accumDeltaX - sin( expsVec[currentExpId].facingRad ) * accumDeltaY;
	exp.y_m = expsVec[currentExpId].y_m	+ sin( expsVec[currentExpId].facingRad ) * accumDeltaX - cos( expsVec[currentExpId].facingRad ) * accumDeltaY;
	exp.facingRad = adjustAngle180( expsVec[currentExpId].facingRad + accumDeltaFacing );
	exp.numLinks = 0;

	expsVec.push_back(exp);
}


/*!
@brief Processes the experience using the data available.
*/
void processExp(int vtId, float vTrans, float vRot, vector<exps>& expsVec)
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
		createNewExp(currentExpId,numExps,vtId,expsVec);

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
					expsVec[currentExpId].numLinks++;
					int linkId = expsVec[currentExpId].numLinks;
					expsVec[currentExpId].links[linkId].expId = matchedExpId;
					expsVec[currentExpId].links[linkId].d = sqrt( pow(accumDeltaX,2) + pow(accumDeltaY,2) );
					expsVec[currentExpId].links[linkId].headingRad = atan2( accumDeltaY, accumDeltaX );
					expsVec[currentExpId].links[linkId].facingRad = accumDeltaFacing;
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
		for(int expID=0; expID<numExps; expID++)
		{
			for(int linkID=0; linkID<expsVec[expID].numLinks; linkID++)
			{
				// Experience 0 has link to experience 1
				int e0 = expID;
				int e1 = expsVec[expID].links[linkID].expId;

				// Work out where e0 thinks e1 (x,y) should be based on the stored link information
				float lx = expsVec[e0].x_m + expsVec[e0].links[linkID].d * cos( expsVec[e0].facingRad + expsVec[e0].links[linkID].headingRad);
				float ly = expsVec[e0].y_m + expsVec[e0].links[linkID].d * sin( expsVec[e0].facingRad + expsVec[e0].links[linkID].headingRad);
			
				// Correct e0 and e1 (x,y) by equal and opposite amounts, a 0.5 correction parameter means that 
				// e0 and e1 will be fully corrected based on e0's link information.
				expsVec[e0].x_m += ( ( expsVec[e1].x_m - lx ) *  (float)EXP_CORRECTION );
				expsVec[e0].y_m += ( ( expsVec[e1].y_m - ly ) *  (float)EXP_CORRECTION );
				expsVec[e1].x_m -= ( ( expsVec[e1].x_m - lx ) *  (float)EXP_CORRECTION );
				expsVec[e1].y_m -= ( ( expsVec[e1].y_m - ly ) *  (float)EXP_CORRECTION );

				// Determine the angle between where e0 thinks e1 is facing
				// should be based on the link information
				float df = getSignedDeltaRad( (expsVec[e0].facingRad + expsVec[e0].links[linkID].facingRad), expsVec[e1].facingRad );

				// Correct e0 and e1 facing by equal but opposite amounts, a 0.5 correction parameter means that
				// e0 and e1 will be fully corrected based on e0's link information.
				expsVec[e0].facingRad = adjustAngle180( expsVec[e0].facingRad + df * (float)EXP_CORRECTION );
				expsVec[e1].facingRad = adjustAngle180( expsVec[e1].facingRad - df * (float)EXP_CORRECTION );
			}
		}
	}

	expHistory.push_back(currentExpId);
}

/*!
@brief Plots the experience on an image.
*/
void plotData()
{
}