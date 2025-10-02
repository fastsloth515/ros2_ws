#include "sk_robot_lib/skSubOccupancyGrid.h"

skSubOccupancyGrid::skSubOccupancyGrid()
{
}

skSubOccupancyGrid::~skSubOccupancyGrid()
{
}

double skSubOccupancyGrid::getDist(const sPoint2D p, const double& minDist/* = 0.0*/)
{
    if( !this->haveMsg() )
        return (-1.0);
    
    // read grid data
    this->m_resolution = this->m_msg.info.resolution;
    this->m_width = this->m_msg.info.width;
    this->m_height = this->m_msg.info.height;
    this->m_x = this->m_msg.info.origin.position.x;
    this->m_y = this->m_msg.info.origin.position.y;

    const int xc(this->x2i(p.x));
    const int yc(this->y2i(p.y));
    const int range(ceil(0.5/this->m_resolution));
    int distSQ(range*range);

    for( int x = -range+1; x < range; x++ )
    {
        if( x+xc < 0 || x+xc >= this->m_width)
            continue;

        for( int y = -range+1; y < range; y++ )
        {
            if( y+yc < 0 || y+yc >= this->m_height)
                continue;
            if( x*x+y*y > range*range )
                continue;
            if( !this->isFeasible(x+xc,y+yc) )
            {
                distSQ = MIN(distSQ,x*x+y*y);
            }
        }
    }

    return (sqrt((double)distSQ)*this->m_resolution);
}

double skSubOccupancyGrid::getVoronoi(const double& height) const
{
    const int h(this->y2i(height));

    if( 0 < h && h < this->m_height )
    {
        std::vector<int> s, e;
        s.clear();
        e.clear();
        bool feasible(false);
        for( int j = 0 ; j < this->m_width; j++ )
        {
            if( feasible )
            {
                if( !this->isFeasible(j,h ))
                {
                    e.push_back(j-1);
                    feasible = false;
                }
                else if( j == this->m_width-1 )
                {
                    e.push_back(j);
                    feasible = false;
                }
            }
            else
            {
                if( this->isFeasible(j,h) )
                {
                    s.push_back(j);
                    feasible = true;
                }
            }
        }
        if( s.size() > 0 )
        {
            int length(e[0]-s[0]);
            int ret((e[0]-s[0])/2);
            for( unsigned int j = 1; j < s.size(); j++ )
            {
                if( e[j]-s[j] > length )
                {
                    length = e[j]-s[j];
                    ret = (e[j]+s[j])/2;
                }
            }
            return (this->m_x+(double)ret*this->m_resolution);
        }
    }
}


int skSubOccupancyGrid::x2i(const double& x) const
{
    return(lround((x-this->m_x)/this->m_resolution));
    const int i(lround((x-this->m_x)/this->m_resolution));

    if( i < 0 )
        return (0);

    if( i >= this->m_width )
        return (this->m_width-1);

    return (i);
}

int skSubOccupancyGrid::y2i(const double& y) const
{
    return(lround((y-this->m_y)/this->m_resolution));
    const int i(lround((y-this->m_y)/this->m_resolution));

    if( i < 0 )
        return (0);

    if( i >= this->m_height )
        return (this->m_height-1);

    return (i);
}

bool skSubOccupancyGrid::isFeasible(const int& x, const int& y) const
{
    return (this->m_msg.data[x+y*this->m_width] == 0);
}
