#include"expMap.h"

#define RENDER_FREQ			10		// Frequency at which plotting is done
#define DRAW_LINKS			1		// Draw links

int main()
{
	cout << "Starting experience mapping" << endl;

	// Read the odometery data file
	ifstream odoFile("velocities2.csv");
	if(!odoFile)
	{
		cerr << "Odometry Data File not found!" << endl;
	}

	Mat odoData = readCSV(odoFile);


	// Read the output matching result file
	ifstream matchFile("fnums.txt");
	if(!matchFile)
	{
		cerr << "Match Result File not found!" << endl;
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


	ofstream tidfile("tid.txt");

	// Generate virtual templates
	Mat tId;
	int c = 1;
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
			tId.push_back(tId.at<int>(index));
		}
		tidfile << tId.at<int>(i1) << endl;
	}

	vector<exps> expsVec;

	// Create first experience which will have no links to begin with
	exps exp;
	exp.vtId = 1;
	exp.x_m = 0;
	exp.y_m = 0;
	exp.facingRad = 0.5*PI;
	exp.numLinks = 0;

	expsVec.push_back(exp);
	expsVec.push_back(exp);

	double vTrans, vRot;
	int vtId;
	for(int i1=0; i1<odoData.rows; i1++)
	{
		vTrans = odoData.at<double>(i1,1);
		vRot = odoData.at<double>(i1,2);
		vtId = tId.at<int>(i1);
	
		processExp(vtId,vTrans,vRot,expsVec);

		vector<Point2f> expPoints;
		for(int j1=1; j1<expsVec.size(); j1++)
		{
			Point2f p1;
			p1.x = expsVec[j1].x_m;
			p1.y = expsVec[j1].y_m;

			expPoints.push_back(p1);
		}

		if(i1 % 10 == 0)
			plotData(expPoints);
	}

	for(int i1=0; i1<500; i1++)
	{
		processExp(vtId,vTrans,vRot,expsVec);
	}

	ofstream testExpsVals("expsData.txt");
	vector<Point2f> expPoints2;

	for(int i1=1; i1<expsVec.size(); i1++)
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