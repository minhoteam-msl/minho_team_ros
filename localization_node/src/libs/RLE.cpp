#include "RLE.h"

RLE::RLE()
{

}

RLE::RLE(ScanLines &s_, unsigned colorBefore, unsigned colorOfInterest, unsigned colorAfter,unsigned threshColorBefore,
         unsigned threshColorOfInterest, unsigned threshColorAfter, unsigned searchWindow)
{
	unsigned pointsColor;
	unsigned pointsColorBefore;
	unsigned pointsColorAfter;
    RLEInfo aux_rle;
	std::vector<int> temp;
	s = s_;

	searchWindow = searchWindow/s.getStep();
	threshColorBefore = threshColorBefore/s.getStep();
	threshColorAfter = threshColorAfter/s.getStep();


	for(unsigned j = 0; j < s.scanlines.size(); j++)
	{
		temp = s.getLine(j);

		for (unsigned i = 0; i < temp.size(); i++)
		{
			//std::cerr << " i " << i << " j " << j << " img " << (unsigned)s.image.ptr()[temp[i]] << std::endl;
			if(!((unsigned)s.image.ptr()[temp[i]] & colorOfInterest))
				continue;

			pointsColor = 0;
			pointsColorAfter = 0;

			aux_rle.start = temp[i];
			aux_rle.startBefore = temp[i];
			while(i < temp.size() - 1 && ((unsigned)s.image.ptr()[temp[i]] & colorOfInterest))
			{
				pointsColor++;
				i++;
			}
			aux_rle.end = temp[i];
			aux_rle.endAfter = temp[i];
			aux_rle.center = temp[i - pointsColor/2];

			if(pointsColor < threshColorOfInterest)
				continue;

            int k;

			pointsColorBefore = 0;
			for(k = i - pointsColor ; k > (i - pointsColor - searchWindow) && k >= 0 ; k--)
			{
				if((unsigned)s.image.ptr()[temp[k]] & colorBefore)
				{
					pointsColorBefore++;
				}
			}
			aux_rle.startBefore = temp[k];

			pointsColorAfter = 0;
			for(k = i ; k < (i + searchWindow) && k < temp.size() - 1 ; k++) // -1 for the end of SC
			{
				if((unsigned)s.image.ptr()[temp[k]] & colorAfter)
				{
					pointsColorAfter++;

				}
			}
			aux_rle.endAfter = temp[k];

			if((pointsColorAfter >= threshColorAfter && pointsColorBefore >= threshColorBefore)
					&& pointsColor >= threshColorOfInterest)
			{

				aux_rle.scIdx = j;
				//aux_rle.start = temp[i - pointsColor];
				//aux_rle.end = temp[i];
				//aux_rle.center = temp[i - pointsColor/2];
				aux_rle.lengthColor = pointsColor * s.getStep();
				aux_rle.lengthColorBefore = pointsColorBefore * s.getStep();
				aux_rle.lengthColorAfter = pointsColorAfter * s.getStep();
				rlData.push_back(aux_rle);
			}
		}
  }
}

void RLE::drawLine(int pt1, int pt2, Scalar color, Mat *img)
{
    cv::line(*img, cv::Point(pt1 % img->cols, pt1 / img->cols), cv::Point(pt2 % img->cols, pt2 / img->cols), color, 2);
}

void RLE::drawCircle(int pt, int radius, Scalar color, Mat *img)
{
    cv::circle(*img, cv::Point(pt % img->cols, pt / img->cols), radius, color, 3);
}

void RLE::drawInterestPoints(Scalar color, Mat *destination, UAV_COLORS_BIT idx)
{
    int target = 0;
    for(unsigned k = 0; k < rlData.size(); k++)
    {
        if(idx&UAV_WHITE_BIT ) target = rlData[k].center;
        else if(idx&UAV_BLACK_BIT || idx&UAV_ORANGE_BIT) target = rlData[k].start;

        circle(*destination,cv::Point(target%destination->cols, target/destination->cols),2,color,2);
    }
}

void RLE::LinespushData(std::vector<Point> &destination, Mat& img, std::vector<double> &distPix,  std::vector<int> &distPixVal, Point robotCenter)
{
  int temp = 0, value = 0;
  unsigned int index = 0;
  Point point;

  for(unsigned k = 0; k < rlData.size(); k++){
    index = 0;
    point = Point(rlData[k].center%img.cols,rlData[k].center/img.cols);
    temp = d2p(robotCenter,point);

    while(temp>distPix[index] && index<(distPix.size()-1))index++;
    if(temp>distPix[distPix.size()-1]){
        value=1000;
    }
    else if(index<=0) value = 0;
    else value = distPixVal[index-1]+(((temp-distPix[index-1])*(distPixVal[index]-distPixVal[index-1]))/(distPix[index]-distPix[index-1]));

    //std::cerr << "Numero de pontos para limitar: " << value << "  Numero possuido de pontos: " << linePointsLength[i];

    if(rlData[k].lengthColor<(value+LINE_LIMIT) && rlData[k].lengthColor>(value-LINE_LIMIT)) destination.push_back(point);
  }
}

void RLE::LinespushDataC(std::vector<Point> &destination, Mat& img)
{
  int target = 0;

  for(unsigned k = 0; k < rlData.size(); k++){
    target = rlData[k].center;
    destination.push_back(cv::Point(target%img.cols,target/img.cols));
  }
}

// Filter found RLE's
void RLE::pushData(std::vector<Point> &destination,Mat& img)
{
  int target = 0;

  for(unsigned k = 0; k < rlData.size(); k++){
    target = rlData[k].start;
    destination.push_back(cv::Point(target%img.cols,target/img.cols));
  }
}

void RLE::draw(cv::Scalar colorBefore, cv::Scalar colorOfInterest, cv::Scalar colorAfter, cv::Mat *destination)
{
	for(unsigned k = 0; k < rlData.size(); k++)
	{
        //drawLine(rlData[k].startBefore, rlData[k].start, colorBefore, destination);
        drawLine(rlData[k].start, rlData[k].end, colorOfInterest, destination);
        //drawLine(rlData[k].end, rlData[k].endAfter, colorAfter, destination);
	}
}
