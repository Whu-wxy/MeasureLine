#include "linesgroup.h"

#include <math.h>
#include <iostream>
using namespace std;

linesGroup::linesGroup()
{

}

linesGroup::~linesGroup()
{
}

void linesGroup::setLine(Vec4i aline)
{
    m_lines.push_back(aline);
    m_iX1 = aline[0];
    m_iY1 = aline[1];
    m_iX2 = aline[2];
    m_iY2 = aline[3];
    //先判断竖线
    if ((m_iX2 - m_iX1) == 0)
    {
        m_averageA = 90;
    }
    else
    {
        m_averageK = (double)(m_iY2 - m_iY1) / (m_iX2 - m_iX1);
        m_averageA = atan((double)(m_iY2 - m_iY1) / (m_iX2 - m_iX1));
        m_averageA = m_averageA * 180 / CV_PI;
    }
}

bool linesGroup::addLine(Vec4i aline, double a, int distance)
{
    int x1, y1, x2, y2, d1, d2;
    double aa;
    x1 = aline[0];
    y1 = aline[1];
    x2 = aline[2];
    y2 = aline[3];

    if ((x2 - x1) == 0)
    {
        aa = 90;
    }
    else
    {
        aa = atan((double)(y2 - y1) / (x2 - x1));
        aa = aa * 180 / CV_PI;
    }
    if (abs(abs(aa) - abs(m_averageA)) > a)
        return false;
    d1 = point2Line(x1, y1);
    d2 = point2Line(x2, y2);
    if (d1 > distance)
        return false;
    if (d2 > distance)
        return false;
    m_lines.push_back(aline);

    if((m_averageA * aa) >= 0)
        m_averageA = (m_averageA + aa) / 2;
    else
        m_averageA = (m_averageA - aa) / 2;
    m_averageK = m_averageA * CV_PI / 180;
    m_averageK = tan(m_averageK);

    return true;
}

int	linesGroup::point2Line(int x, int y)
{
    double distance = 0;
    if(m_averageA == 90)
    {
        distance = abs(m_iX1 - x);
        return (int)distance;
    }
    double a = m_averageK;
    double c = m_iY1 - m_averageK * m_iX1;
    distance = abs(a * x - y + c) / sqrt(a * a + 1);
    return (int)distance;
}

void linesGroup::group2Line()
{
    Vec4i aline;
    Point2f p0, p1;
    aline = m_lines.at(0);
    int xmin = aline[0], ymin = aline[1], xmax = aline[2], ymax = aline[3];
    if (xmin > xmax)
    {
        int xx = xmin;
        xmax = xmin;
        xmin = xx;
    }
    if (ymin > ymax)
    {
        int yy = ymin;
        ymax = ymin;
        ymin = yy;
    }

    int x1, y1, x2, y2;

    for (int i = 0; i < m_lines.size(); i++)
    {
        Vec4i aline2 = m_lines.at(i);
        x1 = aline2[0];
        y1 = aline2[1];
        x2 = aline2[2];
        y2 = aline2[3];

        if (x1 < xmin)
            xmin = x1;
        if (x2 < xmin)
            xmin = x2;
        if (x1 > xmax)
            xmax = x1;
        if (x2 > xmax)
            xmax = x2;
        if (y1 < ymin)
            ymin = y1;
        if (y2 < ymin)
            ymin = y2;
        if (y1 > ymax)
            ymax = y1;
        if (y2 > ymax)
            ymax = y2;
    }

    if (m_averageA > 0)			//注意和数学坐标系不同
    {
        p0 = Point2f(xmin,ymin);
        p1 = Point2f(xmax,ymax);
    }
    else
    {
        p0 = Point2f(xmin,ymax);
        p1 = Point2f(xmax,ymin);
    }

    m_midX = (p0.x + p1.x) / 2;
    m_midY = (p0.y + p1.y) / 2;

    m_mergedLine.push_back(p0);
    m_mergedLine.push_back(p1);
}

Point linesGroup::getMidPosition()
{
    Point mid(m_midX, m_midY);
    return mid;
}
