/*!
@file The code reads a ground truth file and converts that into a format to be read as a matching result for expMap code.
*/

#include<stdio.h>
#include<iostream>
#include<fstream>
#include<sstream>
#include<vector>

using namespace std;

int main()
{
	// Read the output matching result file
	ifstream gt("gt.txt");
	if(!gt)
	{
		cerr << "Match Result File not found!" << endl;
		getchar();
		exit(-1);
	}

	ofstream matchResult("matchResult.txt");

	string buffer;
	vector<int> col1, col2;

	while(getline(gt,buffer))
	{
		stringstream ss(buffer);
		string elem;

		vector<int> row;
		while(getline(ss,elem,'\t'))
		{
			row.push_back(atof(&elem[0]));
		}

		if(!row.empty())
		{
			col1.push_back(row[0]);
			col2.push_back(row[1]);
		}
	}

	int colIter = 0;
	for(int i1=0; i1<1100; i1++)
	{
		if(colIter < col2.size() && i1 == col2[colIter])
		{
			matchResult << 1 << "," << col2[colIter] << "," << col1[colIter] << endl;
			colIter++;
		}
		else
			matchResult << 0 << "," << i1 << "," << -1 << endl;
	}
}