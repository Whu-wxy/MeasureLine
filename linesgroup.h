#ifndef LINESGROUP_H
#define LINESGROUP_H

#include <vector>
#include<opencv2/opencv.hpp>
using namespace cv;
using namespace std;

class linesGroup
{
public:
    linesGroup();
    ~linesGroup();

    //斜率阈值k,两点到直线距离distance 加入linesGroup
    void	setLine(Vec4i);                               //设置线组的第一条线作为参考
    bool	addLine(Vec4i aline, double k, int distance); //将其他接近的线加入，k:0.268
    int		point2Line(int, int);                       //点到线的距离计算
    int		linesNumber() { return m_lines.size(); }    //线组内线数
    Point   getMidPosition();                           //中点位置
    double  getLineA() { return m_averageA; }           //合并线角度
    double  getLineK() { return m_averageK; }           //合并线斜率
    double  getMidX() { return m_midX; }
    double  getMidY() { return m_midY; }
    vector<Vec4i>   getLines() { return m_lines; }      //获得线组中所有线
    int     getLinesCount(){return m_lines.size(); }
    void	group2Line();                               //内部线消除重复
    vector<Point2f>   getMergedLine() {return m_mergedLine; }

private:
    vector<Vec4i>	m_lines;
    vector<Point2f>   m_mergedLine; //线组合并的一条线
    int				m_iX1;
    int				m_iY1;
    int				m_iX2;
    int				m_iY2;
    double			m_averageK;
    double			m_averageA; //arctan k
    double			m_midX;
    double			m_midY;
    Vec4i			m_averageLine;
};


#endif // LINESGROUP_H
