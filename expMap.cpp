#include"expMap.h"

#define FAKE_ODOM			1		// Generate virtual odometry data and overwrite the file read

int main()
{
	cout << "Starting experience mapping" << endl;

	// Read the odometery data file
	ifstream odoFile("velocities2.csv");
	if(!odoFile)
	{
		cerr << "Odometry Data File not found!" << endl;
		getchar();
		exit(-1);
	}

	Mat odoData = readCSV(odoFile);


	// Read the output matching result file
	ifstream matchFile("matchResult.txt");
	if(!matchFile)
	{
		cerr << "Match Result File not found!" << endl;
		getchar();
		exit(-1);
	}

	Mat dataMat = readCSV(matchFile);

	// Load the individual columns and release original matrix
	Mat fi = dataMat.col(0).clone();
	Mat fnum1 = dataMat.col(1).clone();
	Mat fnum2 = dataMat.col(2).clone();
	
	fi.convertTo(fi,CV_32SC1);
	fnum1.convertTo(fnum1,CV_32SC1);
	fnum2.convertTo(fnum2,CV_32SC1);

	dataMat.release();

#if(FAKE_ODOM == 1)
	Mat transVelMat = Mat::ones(fnum1.rows,1,CV_64FC1);
	Mat rotVelMat = Mat(fnum1.rows,1,CV_64FC1,Scalar::all(0.03));
	odoData.release();
	fnum1.convertTo(odoData,CV_64FC1);
	hconcat(odoData,transVelMat,odoData);
	hconcat(odoData,rotVelMat,odoData);
#endif

	ofstream tidfile("tid.txt");

	// Generate virtual templates
	Mat tId;
	int c = 0;

	for(int i1=0; i1<fnum1.rows; i1++)
	{
		if(fi.at<int>(i1) == 0)
		{
			tId.push_back(c);

			int index = min(i1+1,fnum1.rows-1);
			if(odoData.at<double>(index,1) > 0)
			{
				c++;
			}
		}
		else
		{
			int index = fnum2.at<int>(i1);
			int val = tId.at<int>(index);
			tId.push_back(val);
		}
		tidfile << tId.at<int>(i1) << endl;
	}

	vector<exps> expsVec;
	expMap expMap1;

	// Create first experience which will have no links to begin with
	exps exp;
	exp.vtId = 0;
	exp.x_m = 0;
	exp.y_m = 0;
	exp.facingRad = 0.5*PI;
	exp.numLinks = 0;

	//expsVec.push_back(exp);
	expsVec.push_back(exp);

	double vTrans, vRot;
	int vtId;
	for(int i1=0; i1<odoData.rows; i1++)
	{
		vTrans = odoData.at<double>(i1,1);
		vRot = odoData.at<double>(i1,2);
		vtId = tId.at<int>(i1);
	
		expMap1.processExp(vtId,vTrans,vRot,expsVec);

		vector<Point2f> expPoints;
		for(int j1=0; j1<expsVec.size(); j1++)
		{
			Point2f p1;
			p1.x = expsVec[j1].x_m;
			p1.y = expsVec[j1].y_m;

			expPoints.push_back(p1);
		}

		if(i1 % RENDER_FREQ == 0)
			plotData(expPoints);
	}

	for(int i1=0; i1<500; i1++)
	{
		expMap1.processExp(vtId,vTrans,vRot,expsVec);
	}

	ofstream testExpsVals("expsData.txt");
	vector<Point2f> expPoints2;

	for(int i1=0; i1<expsVec.size(); i1++)
	{
		testExpsVals << expsVec[i1].x_m << endl;
		expPoints2.push_back(Point2f(expsVec[i1].x_m,expsVec[i1].y_m));
	}

	Mat plotImg;
	plotData(expPoints2,plotImg);
	imshow("plot2",plotImg);
	int key = waitKey(0);
	if(key == 27)
	{
		cerr << "Esc key pressed, exiting..." << endl;
		exit(-1);
	}

	getchar();
}