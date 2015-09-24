#include <vision/structures/Blob.h>
#include <iostream>

using namespace std;

bool sortBlobAreaPredicate(Blob* left, Blob* right) {
  return left->dx * left->dy < right->dx * right->dy;
}



bool matchBlobBeacon(Blob* a, Blob* b)
{
	int xmax = 1000;
	int ymax = 1000;
	int xmin = 8;
	int ymin = 8;
	double sigmaSquareMax = 100000;
	double ratioA = (a->dx < a->dy) ? (double) a->dx / (double) a->dy : (double) a->dy / (double) a->dx;
        double ratioB = (b->dx < b->dy) ? (double) b->dx / (double) b->dy : (double) b->dy / (double) b->dx;
	int xDiffMax = (int) (((double) a->dx * .35) + .5);
	int yDiffMax = (int) (((double) a->dy * .35) + .5);
	double densityA = (double) a->lpCount / (double) ((double) (a->dx) * (double) (a->dy));
	double densityB = (double) b->lpCount / (double) ((double) (b->dx) * (double) (b->dy));
	//height width ratio
	double heightRatio = (a->dy < b->dy) ? (double) a->dy / (double) b->dy : (double) b->dy / (double) a->dy;
	double widthRatio = (a->dx < b->dx) ? (double) a->dx / (double) b->dx : (double) b->dx / (double) a->dx;
	if ((densityA < .25) || (densityB < .25)){
		return false;
        }
	if ((heightRatio < .60) || (widthRatio < .60)){
		return false;
	}
	if ((a->dx>xmax)||(a->dx<xmin)||(a->dy>ymax)||(a->dy<ymin)){
		//cout<<"size of a not"<<endl;
		return false;}
	if ((b->dx>xmax)||(b->dx<xmin)||(b->dy>ymax)||(b->dy<ymin)){
		//cout<<"size of b not"<<endl;
		return false;}
	if((ratioA < .6) || (ratioB < .6)){
		return false;
	}
	int xDiffStart = ((b->xi)-(a->xi)>0)?(b->xi)-(a->xi):(a->xi)-(b->xi);
	int xDiffEnd = ((b->xf)-(a->xf)>0)?(b->xf)-(a->xf):(a->xf)-(b->xf);
	if ((xDiffStart>xDiffMax)||(xDiffEnd>xDiffMax)){
		//cout<<"Diff of X not"<<endl;
		return false;}
	int yDiff1 = ((b->yi)-(a->yf)>0)?(b->yi)-(a->yf):(a->yf)-(b->yi);
	int yDiff2 = ((b->yf)-(a->yi)>0)?(b->yf)-(a->yi):(a->yi)-(b->yf);
	if ((yDiff1>yDiffMax)&&(yDiff2>yDiffMax)){
		//cout<<"Diff of y not"<<endl;
		return false;}
        //cout<<"d "<<density;

	Blob* twoBlobs[2];
	twoBlobs[0] = a;
	twoBlobs[1] = b;
	
	for (int j=0;j<2;j++)
	{
		Blob orangeBlob = *(twoBlobs[j]);
		int height = orangeBlob.dy;
		int leftX[250];
		int rightX[250];
		vector<int> distance;
		int runY, runXStart, runXEnd;
		int dx, dy;
		for (int i=0;i<height;i++)
		{
			rightX[i] = -1;
			leftX[i] = 1000;
		}
		for (int i = 0;i < orangeBlob.runList.size(); i++)
		{
			runY = orangeBlob.runList[i]->y;
			runXStart = orangeBlob.runList[i]->xStart;
			runXEnd = orangeBlob.runList[i]->xEnd;
			if (runXStart<leftX[runY-orangeBlob.yi])
			{
				leftX[runY-orangeBlob.yi] = runXStart;
			}
			if (runXEnd>rightX[runY-orangeBlob.yi])
			{
				rightX[runY-orangeBlob.yi] = runXEnd;
			}
		}
		for (int i=0;i<height;i++)
		{
			if ((rightX[i]!=-1)&&(leftX[i]!=1000))
			{
				dx = rightX[i] - leftX[i];
				distance.push_back(dx);	
			}
		}
		double sum = 0;
		for (int i = 0;i < distance.size(); i++)
		{
			sum += distance[i];
		}
		double average = (double)sum/distance.size();
		sum = 0;
		for (int i = 0;i < distance.size(); i++)
		{
			sum += (distance[i]-average)*(distance[i]-average);
		}
		double sigma_square = sum/distance.size();
		if (sigma_square > sigmaSquareMax)
		{
			return false;
		}
	}

	return true;
}

bool relaxedmatchBlobBeacon(Blob* a, Blob* b)
{
	//cout<<"& "<<b->xi<<" "<<b->xf<<" "<<b->yi<<" "<<b->yf<<endl;
	int xmax = 1000;
	int ymax = 1000;
	int xmin = 4;
	int ymin = 4;
	double sigmaSquareMax = 100000;
	double ratioA = (a->dx < a->dy) ? (double) a->dx / (double) a->dy : (double) a->dy / (double) a->dx;
        double ratioB = (b->dx < b->dy) ? (double) b->dx / (double) b->dy : (double) b->dy / (double) b->dx;
	int xDiffMax = (int) (((double) a->dx * .60) + .5);
	int yDiffMax = (int) (((double) a->dy * .60) + .5);
	double densityA = (double) a->lpCount / (double) ((double) (a->dx) * (double) (a->dy));
	double densityB = (double) b->lpCount / (double) ((double) (b->dx) * (double) (b->dy));
	//height width ratio
	double heightRatio = (a->dy < b->dy) ? (double) a->dy / (double) b->dy : (double) b->dy / (double) a->dy;
	double widthRatio = (a->dx < b->dx) ? (double) a->dx / (double) b->dx : (double) b->dx / (double) a->dx;
	//cout<< " " << ratioA << " " << ratioB << " " << xDiffMax << " " << yDiffMax << " " << densityA << " " << densityB <<endl;;
	if ((densityA < .15) || (densityB < .15)){
		return false;
        }
	if ((heightRatio < .1) || (widthRatio < .15)){
		return false;
	}
	if ((a->dx>xmax)||(a->dx<xmin)||(a->dy>ymax)||(a->dy<ymin)){
		//cout<<"size of a not"<<endl;
		return false;}
	if ((b->dx>xmax)||(b->dx<xmin)||(b->dy>ymax)||(b->dy<ymin)){
		//cout<<"size of b not"<<endl;
		return false;}
	
	if((ratioA < .4) || (ratioB < .15)){
		return false;
	}
	int xDiffStart = ((b->xi)-(a->xi)>0)?(b->xi)-(a->xi):(a->xi)-(b->xi);
	int xDiffEnd = ((b->xf)-(a->xf)>0)?(b->xf)-(a->xf):(a->xf)-(b->xf);
	//cout<< "@" << xDiffStart << " " << xDiffEnd<<endl;
	if ((xDiffStart>xDiffMax)||(xDiffEnd>xDiffMax)){
		//cout<<"Diff of X not"<<endl;
		return false;}
	int yDiff1 = ((b->yi)-(a->yf)>0)?(b->yi)-(a->yf):(a->yf)-(b->yi);
	int yDiff2 = ((b->yf)-(a->yi)>0)?(b->yf)-(a->yi):(a->yi)-(b->yf);
	//cout<< "!" << xDiffStart << " " << xDiffEnd<<endl;
	if ((yDiff1>yDiffMax)&&(yDiff2>yDiffMax)){
		//cout<<"Diff of y not"<<endl;
		return false;}
        //cout<<"d "<<density;

	return true;

}

