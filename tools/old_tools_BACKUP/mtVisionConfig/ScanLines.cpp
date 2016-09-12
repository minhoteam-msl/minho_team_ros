#include "ScanLines.h"

ScanLines::ScanLines()
{

	start = cv::Point(0,0);
	end = cv::Point(0,0);
	stepX = 0;
	stepY = 0;
	inRadius = 0;
	outRadius = 0;
    type = 0;
}

ScanLines::ScanLines(Mat &img, Point center_, Point end_)
{
    inRadius = 0;
    outRadius = 1000;
    type = UAV_RADIAL;
    center = center_;
    image = img;
    std::vector<int> temp;

    if(type == UAV_RADIAL)
    {
        float teta;
        temp.clear();
        if(end_.x == 0) teta = -M_PI;
        else teta = -atan2(end_.y-center_.y,end_.x-center_.x);

        start.x = (int)(center.x + inRadius * cos(teta));
        start.y = (int)(center.y - inRadius * sin(teta));
        end.x = (int)(center.x + outRadius * cos(teta));
        end.y = (int)(center.y - outRadius * sin(teta));

        int sx = start.x;
        int sy = start.y;
        int ex = end.x;
        int ey = end.y;
        int x1 = sx;
        int y1 = sy;
        int x2 = ex;
        int y2 = ey;
        int dx, dy, sdx, sdy, px, py, dxabs, dyabs, i;
        //float slope;
        int x,y;


        dx = x2 - x1;      // the horizontal distance of the line
        dy = y2 - y1;      /// the vertical distance of the line
        dxabs = abs(dx);
        dyabs = abs(dy);
        sdx = signum(dx);
        sdy = signum(dy);
        x = dyabs >> 1;
        y = dxabs >> 1;
        px = x1;
        py = y1;

        if (dxabs >= dyabs) // the line is more horizontal than vertical
        {
            for(i = 0 ; i < dxabs ; i++)
            {
                y += dyabs;
                if (y >= dxabs)
                {
                    y -= dxabs;
                    py += sdy;
                }

                px += sdx;
                if( py < 0 || (unsigned)py >= image.rows || px < 0 || (unsigned)px >= image.cols )
                    continue;

                temp.push_back(px+py*image.cols);

            }
        }
        else // the line is more vertical than horizontal
        {
            for(i = 0 ; i < dyabs ; i++)
            {
                x += dxabs;
                if (x >= dyabs)
                {
                    x -= dyabs;
                    px += sdx;
                }

                py += sdy;
                if( py < 0 || (unsigned)py >= image.rows || px < 0 || (unsigned)px >= image.cols )
                    continue;

                temp.push_back(px+py*image.cols);

            }
        }

        scanlines.push_back(temp);
    }
}

ScanLines::ScanLines (Mat &img, unsigned scanType, cv::Point center_,
        unsigned nSensors, unsigned inRad, unsigned outRad, int radiusStep)
{
	inRadius = inRad;
	outRadius = outRad;
	type = scanType;
	center = center_;
    image = img;

	std::vector<int> temp;


	if(type == UAV_RADIAL)
	{

		float delta = M_PI * 2 / nSensors; // 1 if 360
		float teta;

		for(unsigned k = 0 ; k < nSensors ; k++)
		{
			temp.clear();
			teta = -M_PI_2 + delta * k;

			if(nSensors > 700 )
			{
				start.x = (int)(center.x + (inRadius + (k % 4) * 100 )* cos(teta));
				start.y = (int)(center.y - (inRadius + (k % 4) * 100 ) * sin(teta));
			}
			else if(nSensors > 360)
			{
				start.x = (int)(center.x + (inRadius + (k % 2) * 250 )* cos(teta));
				start.y = (int)(center.y - (inRadius + (k % 2) * 250 ) * sin(teta));
			}
			else
			{
				start.x = (int)(center.x + inRadius * cos(teta));
				start.y = (int)(center.y - inRadius * sin(teta));
			}
			end.x = (int)(center.x + outRadius * cos(teta));
			end.y = (int)(center.y - outRadius * sin(teta));


			int sx = start.x;
			int sy = start.y;
			int ex = end.x;
			int ey = end.y;
			int x1 = sx;
			int y1 = sy;
			int x2 = ex;
			int y2 = ey;
			int dx, dy, sdx, sdy, px, py, dxabs, dyabs, i;
			//float slope;
			int x,y;


			dx = x2 - x1;      // the horizontal distance of the line
			dy = y2 - y1;      /// the vertical distance of the line
			dxabs = abs(dx);
			dyabs = abs(dy);
			sdx = signum(dx);
			sdy = signum(dy);
			x = dyabs >> 1;
			y = dxabs >> 1;
			px = x1;
			py = y1;

			if (dxabs >= dyabs) // the line is more horizontal than vertical
			{
				for(i = 0 ; i < dxabs ; i++)
				{
					y += dyabs;
					if (y >= dxabs)
					{
						y -= dxabs;
						py += sdy;
					}

					px += sdx;
					if( py < 0 || (unsigned)py >= image.rows || px < 0 || (unsigned)px >= image.cols )
						continue;

					temp.push_back(px+py*image.cols);

				}
			}
			else // the line is more vertical than horizontal
			{
				for(i = 0 ; i < dyabs ; i++)
				{
					x += dxabs;
					if (x >= dyabs)
					{
						x -= dyabs;
						px += sdx;
					}

					py += sdy;
					if( py < 0 || (unsigned)py >= image.rows || px < 0 || (unsigned)px >= image.cols )
						continue;

					temp.push_back(px+py*image.cols);

				}
			}

			scanlines.push_back(temp);
		}
	}

	if(type == UAV_CIRCULAR)
	{

		cv::Point currentPoint, relCoord;
		double delta1, delta2, delta3;
		int cdir;
		int limit = 0;

		for(double radius = inRadius; radius <= outRadius && radius <image.rows / 2 && radius < image.cols / 2 ; radius+=radiusStep )
		{
			temp.clear();

			relCoord.x = 0;
			relCoord.y = radius;
			cdir = 0;

			limit = 0;
			do
			{
				currentPoint.x = center.x + relCoord.x;
				currentPoint.y = center.y + relCoord.y;

				//std::cerr<<currentPoint.x<<"*"<<currentPoint.y<<" r= " << radius << std::endl;

				if(currentPoint.x < image.cols && currentPoint.y < image.rows)
				{
					temp.push_back(currentPoint.y*image.cols+currentPoint.x);
				}

				switch (cdir)
				{
				case 0:
					relCoord.x ++;
					delta1 = fabs(radius - sqrt((relCoord.x) * (relCoord.x) + (relCoord.y) * (relCoord.y)));
					delta2 = fabs(radius - sqrt((relCoord.x) * (relCoord.x) + (relCoord.y - 1) * (relCoord.y - 1)));
					delta3 = fabs(radius - sqrt((relCoord.x) * (relCoord.x) + (relCoord.y + 1) * (relCoord.y + 1)));
					if (delta2 < delta1)
					{
						relCoord.y --;
						cdir = 1;
					}
					else if (delta3 < delta1)
					{
						relCoord.y ++;
						cdir = 7;
					}
					break;

				case 1:
					relCoord.x++;
					relCoord.y--;
					delta1 = fabs(radius - sqrt(relCoord.x * relCoord.x + relCoord.y * relCoord.y));
					delta2 = fabs(radius - sqrt((relCoord.x - 1) * (relCoord.x - 1) + (relCoord.y) * (relCoord.y)));
					delta3 = fabs(radius - sqrt((relCoord.x) * (relCoord.x) + (relCoord.y + 1) * (relCoord.y + 1)));
					if (delta2 < delta1)
					{
						relCoord.x--;
						cdir = 2;
					}
					else if (delta3 < delta1)
					{
						relCoord.y++;
						cdir = 0;
					}
					break;

				case 2:
					relCoord.y--;
					delta1 = fabs(radius - sqrt((relCoord.x) * (relCoord.x) + (relCoord.y) * (relCoord.y)));
					delta2 = fabs(radius - sqrt((relCoord.x - 1) * (relCoord.x - 1) + (relCoord.y) * (relCoord.y)));
					delta3 = fabs(radius - sqrt((relCoord.x + 1) * (relCoord.x + 1) + (relCoord.y) * (relCoord.y)));
					if (delta2 < delta1)
					{
						relCoord.x--;
						cdir = 3;
					}
					else if (delta3 < delta1)
					{
						relCoord.x++;
						cdir = 1;
					}
					break;

				case 3:
					relCoord.x--;
					relCoord.y--;
					delta1 = fabs(radius - sqrt(relCoord.x * relCoord.x + relCoord.y * relCoord.y ));
					delta2 = fabs(radius - sqrt(relCoord.x * relCoord.x + (relCoord.y + 1) * (relCoord.y + 1)));
					delta3 = fabs(radius - sqrt((relCoord.x+1) * (relCoord.x+1) + (relCoord.y) * (relCoord.y)));
					if (delta2 < delta1)
					{
						relCoord.y++;
						cdir = 4;
					}
					else if (delta3 < delta1)
					{
						relCoord.x++;
						cdir = 2;
					}
					break;

				case 4:
					relCoord.x--;
					delta1 = fabs(radius - sqrt(relCoord.x * relCoord.x + relCoord.y * relCoord.y));
					delta2 = fabs(radius - sqrt(relCoord.x * relCoord.x + (relCoord.y + 1) * (relCoord.y + 1)));
					delta3 = fabs(radius - sqrt(relCoord.x * relCoord.x + (relCoord.y - 1) * (relCoord.y - 1)));
					if (delta2 < delta1)
					{
						relCoord.y++;
						cdir = 5;
					}
					else if (delta3 < delta1)
					{
						relCoord.y--;
						cdir = 3;
					}
					break;

				case 5:
					relCoord.x--;
					relCoord.y++;
					delta1 = fabs(radius - sqrt(relCoord.x * relCoord.x + relCoord.y * relCoord.y));
					delta2 = fabs(radius - sqrt((relCoord.x + 1) * (relCoord.x + 1) + relCoord.y * relCoord.y));
					delta3 = fabs(radius - sqrt((relCoord.x) * (relCoord.x) + (relCoord.y-1) * (relCoord.y-1)));
					if (delta2 < delta1)
					{
						relCoord.x++;
						cdir = 6;
					}
					else if (delta3 < delta1)
					{
						relCoord.y--;
						cdir = 4;
					}
					break;

				case 6:
					relCoord.y++;
					delta1 = fabs(radius - sqrt(relCoord.x * relCoord.x + relCoord.y * relCoord.y));
					delta2 = fabs(radius - sqrt((relCoord.x + 1) * (relCoord.x + 1) + relCoord.y * relCoord.y));
					delta3 = fabs(radius - sqrt((relCoord.x - 1) * (relCoord.x - 1) + relCoord.y * relCoord.y));
					if (delta2 < delta1)
					{
						relCoord.x++;
						cdir = 7;
					}
					else if (delta3 < delta1)
					{
						relCoord.x--;
						cdir = 5;
					}
					break;

				case 7:
					relCoord.x++;
					relCoord.y++;
					delta1 = fabs(radius - sqrt(relCoord.x * relCoord.x + relCoord.y * relCoord.y));
					delta2 = fabs(radius - sqrt(relCoord.x * relCoord.x + (relCoord.y - 1) * (relCoord.y - 1)));
					delta3 = fabs(radius - sqrt((relCoord.x-1) * (relCoord.x-1) + (relCoord.y) * (relCoord.y)));
					if (delta2 < delta1)
					{
						relCoord.y--;
						cdir = 0;
					}
					else if (delta3 < delta1)
					{
						relCoord.x--;
						cdir = 6;
					}
					break;
				}
				limit++;


			}while ((relCoord.y != radius || relCoord.x != 0) && limit < 3000);

			if( limit == 3000 )
				std::cerr << "ERROR limit exceeded" <<std::endl;

			scanlines.push_back(temp);
		}

	}
}

ScanLines::ScanLines (Mat &img,unsigned scanType, cv::Point center_,
        unsigned nSensors, unsigned inRad, unsigned outRad, int nLevels, int step)
{

	inRadius = inRad;
	outRadius = outRad;
	type = scanType;
	center = center_;
    image = img;
	std::vector<int> temp;

	if(type == UAV_RADIAL)
	{

		float delta;    // = M_PI * 2 / nSensors; // 1 if 360
		float teta;
		int offsetStep = (int)(outRadius - inRadius)/nLevels;

		for(int l = 0 ; l < nLevels ; l++)
		{
			unsigned nSensorsOnLevel;
			float angleOffset;

			if(l == 0)
			{
				angleOffset = 0;
				nSensorsOnLevel = nSensors / (int)pow(2, nLevels - 1);
			}
			else
				if(l == 1)
				{
					nSensorsOnLevel = nSensors/(int)pow(2, nLevels-1); //nSensors / (int)pow(2, (nLevels - l));
					//angleOffset = nSensors / (nSensorsOnLevel);
				}
				else
				{
					nSensorsOnLevel = 2*nSensorsOnLevel;
				}

			delta = M_PI * 2 / nSensorsOnLevel;
			unsigned offset = offsetStep * l;

			if(l != 0)
			{
				angleOffset = delta / 2;
			}

			for(unsigned k = 0 ; k < nSensorsOnLevel ; k += step)
			{
				temp.clear();
				teta = -M_PI_2 + delta * k + angleOffset;


				start.x = (int)(center.x + (inRadius + offset)* cos(teta));
				start.y = (int)(center.y - (inRadius + offset) * sin(teta));

				end.x = (int)(center.x + outRadius * cos(teta));
				end.y = (int)(center.y - outRadius * sin(teta));


				int sx = start.x;
				int sy = start.y;
				int ex = end.x;
				int ey = end.y;
				int x1 = sx;
				int y1 = sy;
				int x2 = ex;
				int y2 = ey;
				int dx, dy, sdx, sdy, px, py, dxabs, dyabs, i;
				//float slope;
				int x,y;


				dx = x2 - x1;      // the horizontal distance of the line
				dy = y2 - y1;      /// the vertical distance of the line
				dxabs = abs(dx);
				dyabs = abs(dy);
				sdx = signum(dx);
				sdy = signum(dy);
				x = dyabs >> 1;
				y = dxabs >> 1;
				px = x1;
				py = y1;

				if (dxabs >= dyabs) // the line is more horizontal than vertical
				{
					for(i = 0 ; i < dxabs ; i++)
					{
						y += dyabs;
						if (y >= dxabs)
						{
							y -= dxabs;
							py += sdy;
						}

						px += sdx;
						if( py < 0 || (unsigned)py >= image.rows || px < 0 || (unsigned)px >= image.cols )
							continue;

						temp.push_back(px+py*image.cols);

					}
				}
				else // the line is more vertical than horizontal
				{
					for(i = 0 ; i < dyabs ; i++)
					{
						x += dxabs;
						if (x >= dyabs)
						{
							x -= dyabs;
							px += sdx;
						}

						py += sdy;
						if( py < 0 || (unsigned)py >= image.rows || px < 0 || (unsigned)px >= image.cols )
							continue;

						temp.push_back(px+py*image.cols);

					}
				}

				scanlines.push_back(temp);
			}
		}
	}

	if(type == UAV_CIRCULAR)
	{

		cv::Point currentPoint, relCoord;
		double delta1, delta2, delta3;
		int cdir;
		int limit = 0;
		int radiusStep = (outRadius-inRadius)/nSensors;

		for(double radius = inRadius; radius <= outRadius && radius <image.rows / 2 && radius < image.cols / 2 ; radius+=radiusStep )
		{
			temp.clear();

			relCoord.x = 0;
			relCoord.y = radius;
			cdir = 0;

			limit = 0;
			do
			{
				currentPoint.x = center.x + relCoord.x;
				currentPoint.y = center.y + relCoord.y;

				//std::cerr<<currentPoint.x<<"*"<<currentPoint.y<<" r= " << radius << std::endl;

				if(currentPoint.x < image.cols && currentPoint.y < image.rows)
				{
					temp.push_back(currentPoint.y*image.cols+currentPoint.x);
				}

				switch (cdir)
				{
				case 0:
					relCoord.x ++;
					delta1 = fabs(radius - sqrt((relCoord.x) * (relCoord.x) + (relCoord.y) * (relCoord.y)));
					delta2 = fabs(radius - sqrt((relCoord.x) * (relCoord.x) + (relCoord.y - 1) * (relCoord.y - 1)));
					delta3 = fabs(radius - sqrt((relCoord.x) * (relCoord.x) + (relCoord.y + 1) * (relCoord.y + 1)));
					if (delta2 < delta1)
					{
						relCoord.y --;
						cdir = 1;
					}
					else if (delta3 < delta1)
					{
						relCoord.y ++;
						cdir = 7;
					}
					break;

				case 1:
					relCoord.x++;
					relCoord.y--;
					delta1 = fabs(radius - sqrt(relCoord.x * relCoord.x + relCoord.y * relCoord.y));
					delta2 = fabs(radius - sqrt((relCoord.x - 1) * (relCoord.x - 1) + (relCoord.y) * (relCoord.y)));
					delta3 = fabs(radius - sqrt((relCoord.x) * (relCoord.x) + (relCoord.y + 1) * (relCoord.y + 1)));
					if (delta2 < delta1)
					{
						relCoord.x--;
						cdir = 2;
					}
					else if (delta3 < delta1)
					{
						relCoord.y++;
						cdir = 0;
					}
					break;

				case 2:
					relCoord.y--;
					delta1 = fabs(radius - sqrt((relCoord.x) * (relCoord.x) + (relCoord.y) * (relCoord.y)));
					delta2 = fabs(radius - sqrt((relCoord.x - 1) * (relCoord.x - 1) + (relCoord.y) * (relCoord.y)));
					delta3 = fabs(radius - sqrt((relCoord.x + 1) * (relCoord.x + 1) + (relCoord.y) * (relCoord.y)));
					if (delta2 < delta1)
					{
						relCoord.x--;
						cdir = 3;
					}
					else if (delta3 < delta1)
					{
						relCoord.x++;
						cdir = 1;
					}
					break;

				case 3:
					relCoord.x--;
					relCoord.y--;
					delta1 = fabs(radius - sqrt(relCoord.x * relCoord.x + relCoord.y * relCoord.y ));
					delta2 = fabs(radius - sqrt(relCoord.x * relCoord.x + (relCoord.y + 1) * (relCoord.y + 1)));
					delta3 = fabs(radius - sqrt((relCoord.x+1) * (relCoord.x+1) + (relCoord.y) * (relCoord.y)));
					if (delta2 < delta1)
					{
						relCoord.y++;
						cdir = 4;
					}
					else if (delta3 < delta1)
					{
						relCoord.x++;
						cdir = 2;
					}
					break;

				case 4:
					relCoord.x--;
					delta1 = fabs(radius - sqrt(relCoord.x * relCoord.x + relCoord.y * relCoord.y));
					delta2 = fabs(radius - sqrt(relCoord.x * relCoord.x + (relCoord.y + 1) * (relCoord.y + 1)));
					delta3 = fabs(radius - sqrt(relCoord.x * relCoord.x + (relCoord.y - 1) * (relCoord.y - 1)));
					if (delta2 < delta1)
					{
						relCoord.y++;
						cdir = 5;
					}
					else if (delta3 < delta1)
					{
						relCoord.y--;
						cdir = 3;
					}
					break;

				case 5:
					relCoord.x--;
					relCoord.y++;
					delta1 = fabs(radius - sqrt(relCoord.x * relCoord.x + relCoord.y * relCoord.y));
					delta2 = fabs(radius - sqrt((relCoord.x + 1) * (relCoord.x + 1) + relCoord.y * relCoord.y));
					delta3 = fabs(radius - sqrt((relCoord.x) * (relCoord.x) + (relCoord.y-1) * (relCoord.y-1)));
					if (delta2 < delta1)
					{
						relCoord.x++;
						cdir = 6;
					}
					else if (delta3 < delta1)
					{
						relCoord.y--;
						cdir = 4;
					}
					break;

				case 6:
					relCoord.y++;
					delta1 = fabs(radius - sqrt(relCoord.x * relCoord.x + relCoord.y * relCoord.y));
					delta2 = fabs(radius - sqrt((relCoord.x + 1) * (relCoord.x + 1) + relCoord.y * relCoord.y));
					delta3 = fabs(radius - sqrt((relCoord.x - 1) * (relCoord.x - 1) + relCoord.y * relCoord.y));
					if (delta2 < delta1)
					{
						relCoord.x++;
						cdir = 7;
					}
					else if (delta3 < delta1)
					{
						relCoord.x--;
						cdir = 5;
					}
					break;

				case 7:
					relCoord.x++;
					relCoord.y++;
					delta1 = fabs(radius - sqrt(relCoord.x * relCoord.x + relCoord.y * relCoord.y));
					delta2 = fabs(radius - sqrt(relCoord.x * relCoord.x + (relCoord.y - 1) * (relCoord.y - 1)));
					delta3 = fabs(radius - sqrt((relCoord.x-1) * (relCoord.x-1) + (relCoord.y) * (relCoord.y)));
					if (delta2 < delta1)
					{
						relCoord.y--;
						cdir = 0;
					}
					else if (delta3 < delta1)
					{
						relCoord.x--;
						cdir = 6;
					}
					break;
				}
				limit++;


			}while ((relCoord.y != radius || relCoord.x != 0) && limit < 3000);

			if( limit == 3000 )
				std::cerr << "ERROR limit exceeded" <<std::endl;

			scanlines.push_back(temp);
		}

	}
}

int ScanLines::signum(int d)
{
	return d<0? -1 : d>0;

}

void ScanLines::draw(cv::Mat &destination, cv::Scalar color)
{
	cv::Point temp;
	std::vector<int> s;

	for (unsigned k = 0 ; k < scanlines.size() ; k++){
		s = getLine(k);
		for(unsigned i = 0; i < s.size(); i++)
		{
			destination.ptr()[s[i] * 3] = color[0];
			destination.ptr()[s[i] * 3 + 1] = color[1];
			destination.ptr()[s[i] * 3 + 2] = color[2];
		}
	}
}

unsigned ScanLines::getStep()
{
	unsigned result;
	cv::Point p1, p2;
	double distance;

	switch(type)
	{
	case UAV_HORIZONTAL:
		result = stepX;
		break;
	case UAV_VERTICAL:
		result = stepY;
		break;
	case UAV_RADIAL:
		result = 1;
		break;
	case UAV_CIRCULAR:
		result = 1;
		break;
	default:
		std::cout<< "Type of scanlines not supported!" << std::endl;
		break;
	}

	return result;

}

void ScanLines::drawLine(int pt1, int pt2, cv::Scalar color, cv::Mat &img)
{
    cv::line(img, cv::Point(pt1 % img.cols, pt1 / img.cols), cv::Point(pt2 % img.cols, pt2 / img.cols), color, 2);
}

void ScanLines::drawCircle(int pt, int radius, cv::Scalar color, cv::Mat &img)
{
    cv::circle(img, cv::Point(pt % img.cols, pt / img.cols), radius, color, 4);
}
