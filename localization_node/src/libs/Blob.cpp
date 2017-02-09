#include "Blob.h"

Blob::Blob()
{
	UMblobs.clear();
}


void Blob::createBlobs(vector<Point2d> &populationPoints, float threshold, int points_number, Point robotCenter, BlobType blobType, int robot_orientation)
{
	if(populationPoints.size() == 0)return;
	int index;

	for(unsigned i = 0; i<populationPoints.size(); i++)
	{
		index = findBlobReal(populationPoints[i], threshold);

		if(index==-1)
		{
			BlobInfo b(populationPoints[i]);
			UMblobs.push_back(b);
		}
		else updateBlob(UMblobs[index], populationPoints[i], robotCenter);
	}
	filterBlobs(points_number, blobType);
	//seeNeighbor();
	relocBlobs(robot_orientation); // Reajusts blob center
}

void Blob::relocBlobs(int orientation)
{
	double ang = 0.00;
	for(unsigned i = 0; i<UMblobs.size();i++){
		UMblobs[i].dist_to_robot = sqrt((UMblobs[i].center.x * UMblobs[i].center.x) + (UMblobs[i].center.y * UMblobs[i].center.y));
		UMblobs[i].dist_to_robot += UMblobs[i].radius_id;
		ang = atan2(UMblobs[i].center.y, UMblobs[i].center.x)*(180.0/M_PI);
		while(ang>360)ang-=360;
		while(ang<0)ang+=360;
		UMblobs[i].center = mapPointToRobot(0, Point2d(UMblobs[i].dist_to_robot,ang));
	}
}
void Blob::filterBlobs(int num, BlobType blobType)
{
	switch(blobType){
		case OBS_BLOB:
		for(unsigned i = 0; i<UMblobs.size();i++){
			if(UMblobs[i].points.size()<=num){ // || (UMblobs[i].radius_id*2)>=0.60
				UMblobs.erase(UMblobs.begin()+i);
				i--;
			}
		}
			break;
		case BALL_BLOB:
		for(unsigned i = 0; i<UMblobs.size();i++){
			if(UMblobs[i].points.size()<=num){ // || (UMblobs[i].radius_id*2)>=0.30
				UMblobs.erase(UMblobs.begin()+i);
				i--;
			}
			else UMblobs[i].radius_id = 0.25;
		}
		break;

	default:
		std::cout<<" Filtering method not supported!"<<std::endl;
		break;
	}
}

void Blob::updateBlob(BlobInfo &blob, Point2d populationPoint, Point robotCenter)
{
	float x, y;
	double dist=0;
	x = y = 0;
	blob.points.push_back(populationPoint);
	for(unsigned int i = 0; i < blob.points.size(); i++)
	{
		x = x + blob.points[i].x;
		y = y + blob.points[i].y;
	}
	blob.center.x = x/blob.points.size();
	blob.center.y = y/blob.points.size();

	if(blob.points.size()==1)blob.radius_id = 0.00;
	else {
		for(unsigned int i = 0; i < blob.points.size(); i++){
			dist = distance(blob.center,blob.points[i]);
			if(abs(dist) >= blob.radius_id)blob.radius_id = abs(dist);
			//std::cerr << "Distance : " << abs(dist) << std::endl;
		}
	}
}

int Blob::findBlobReal(Point2d populationPoint, float thresh)
{
	if(UMblobs.size() == 0)return -1;
	double dist=0;
	Point2d centerBlob;
	for(unsigned i = 0; i<UMblobs.size(); i++)
	{
		centerBlob = UMblobs[i].center;
		dist = distance(centerBlob, populationPoint);
		if(abs(dist) <= thresh + UMblobs[i].radius_id) return i; // +UMblobs[i].radius_id
	}
	return -1;
}

double Blob::distance(Point2d p1, Point2d p2)
{
	double distance = sqrt((p1.x-p2.x)*(p1.x-p2.x)+(p1.y-p2.y)*(p1.y-p2.y));

	return distance;
}

void Blob::seeNeighbor()
{
	double dist=0;

	for(unsigned i = 0; i<UMblobs.size(); i++){
		for(unsigned j = 0; j<UMblobs.size(); j++){
			if(j!=i){
				dist = UMblobs[i].radius_id + UMblobs[j].radius_id;
				dist = abs(abs(distance(UMblobs[i].center,UMblobs[j].center))-dist); // mudar isto
				if(dist<=0.05){
					fuseBlobs(UMblobs[i],UMblobs[j]);
						UMblobs.erase(UMblobs.begin()+j);
						j--;
				}
			}
		}
	}
}

void Blob::fuseBlobs(BlobInfo &b1, BlobInfo &b2)
{
	for(unsigned i = 0; i<b2.points.size(); i++){
		b1.points.push_back(b2.points[i]);
	}
	updateFusedBlob(b1);

}

void Blob::updateFusedBlob(BlobInfo &blob)
{
	float x, y;
	double dist=0;
	x = y = 0;
	for(unsigned int i = 0; i < blob.points.size(); i++)
	{
		x = x + blob.points[i].x;
		y = y + blob.points[i].y;
	}
	blob.center.x = x/blob.points.size();
	blob.center.y = y/blob.points.size();

	if(blob.points.size()==1)blob.radius_id = 0;
	else {
		for(unsigned int i = 0; i < blob.points.size(); i++){
			dist = distance(blob.center,blob.points[i]);
			if(abs(dist) >= blob.radius_id)blob.radius_id = abs(dist);
			//std::cerr << "Distance : " << abs(dist) << std::endl;
		}
	}
}

void Blob::draw(Scalar color, Mat *destination, double max_dist)
{
	double raio=0;
	double conversion = (IMG_SIZE/2)/max_dist;
	for(unsigned i = 0; i < UMblobs.size(); i++){
		raio = abs(conversion*UMblobs[i].radius_id);
		circle(*destination,Point((IMG_SIZE/2)-conversion*UMblobs[i].center.x,(IMG_SIZE/2)+conversion*UMblobs[i].center.y),raio,color,CV_FILLED);
	}
}

void Blob::sort(SortMethod sortMethod)
{

	if(UMblobs.size() <= 1) return;
	Point2d temp;
	BlobInfo tmp(temp);

	switch(sortMethod){
	case SORT_BY_DISTANCE:
		for(unsigned i = 0 ; i < UMblobs.size() - 1 ; i++)
		{
			for(unsigned j = i + 1 ; j < UMblobs.size() ; j++)
			{
				if (UMblobs[i].center.x > UMblobs[j].center.x && UMblobs[i].center.y > UMblobs[j].center.y)
				{
					tmp = UMblobs[j];
					UMblobs[j] = UMblobs[i];
					UMblobs[i] = tmp;
				}
			}
		}
		break;

	case SORT_BY_SIZE:

		for(unsigned i = 0 ; i < UMblobs.size() - 1 ; i++)
		{
			for(unsigned j = i + 1 ; j < UMblobs.size() ; j++)
			{
				if(UMblobs[i].points.size() < UMblobs[j].points.size())
				{
					tmp = UMblobs[j];
					UMblobs[j] = UMblobs[i];
					UMblobs[i] = tmp;
				}
			}
		}
		break;
	default:
		std::cout<<" Sort method not supported!"<<std::endl;
		break;
	}
}

Point2d Blob::mapPointToRobot(double orientation, Point2d dist_lut)
{
   // Always mapped in relation to (0,0)
   double pointRelX = dist_lut.x*cos((dist_lut.y)*DEGTORAD);
   double pointRelY = dist_lut.x*sin((dist_lut.y)*DEGTORAD);
   double ang = orientation*DEGTORAD;

   return Point2d(cos(ang)*pointRelX-sin(ang)*pointRelY,
                  sin(ang)*pointRelX+cos(ang)*pointRelY);
}
