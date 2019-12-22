#ifndef MEASURETOOL_H
#define MEASURETOOL_H

#include "opencv2/opencv.hpp"//添加Opencv相关头文件
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <iostream>
#include <QString>
#include <QMap>
#include "linesGroup.h"

using namespace cv;
using namespace std;

struct CircleData
{
    Point center;
    float radius;
};

class MeasureTool
{
public:
    MeasureTool();
    ~MeasureTool();

    void setFilePath(QString path) {m_filePath = path; }           //图片路径
    void setRulerLength(double length) {m_dActualRuler = length; } //设置标尺真实长度cm
    void startMeasure();    //进行全部测量任务

    void line2Group(vector<linesGroup> &,vector<Vec4i> &, double angle, double dis);//一组线合并
    void clearOneLineGroup(vector<linesGroup> &);
    void cvThin(Mat& src, cv::Mat& dst, int intera);      //细化（未使用）
    bool get_line_intersection(float p0_x, float p0_y, float p1_x, float p1_y,
        float p2_x, float p2_y, float p3_x, float p3_y, float *i_x, float *i_y);//求线的交点，如果没有直接交点，返回false
    vector<Point2f> findEdgePoints(Mat img, double &width, double &height);//纸张角点
    Point2f getCrossPoint(Point2f p1, Point2f p2, Point2f p3, Point2f p4);  //获得线的交点位置,不管线是否直接有交点

    void    projectRepair(Mat srcImg,Mat &dstImg);      //透视校正，通过纸张的四个角点，原来的点可通过设置已知纸张参数或者取对边中最长的作为长宽
    void    setPaperSize(double width, double height);  //设置纸张长宽
    void    linesProcess(Mat &srcImg);                  //合并线，得到线组和可能的标尺线组
    double  findHRuler(Mat &img, vector<linesGroup> rulerHGroups,vector<double>);//横向标尺长度
    double  findVRuler(Mat &img, vector<linesGroup> rulerVGroups,vector<double>);//纵向标尺长度
    Mat     measureLength(Mat &img);                    //测量线的长度
    void    measureAngle(Mat &img);                     //测量线的夹角

    void    findCurve(Mat BiImg, Mat &img2Draw);        //提取出弧线
    CircleData findCircle1(vector<Point> threePoints);  //用圆拟合弧线

    //存在问题，目前不用
    vector<Point> scanToThreePoints(Mat img, bool bHori, vector<Point> points);//获取弧线上三个点

private:
    QString    m_filePath;
    Mat        m_srcImage,m_repairedImg,repairedBiImg;//原图，透视校正的图，透视二值化的图
    vector<linesGroup> m_lineGroups;//相近线分组

    vector<double>	m_linesX;  //目的是得到所有近于垂直的线，从其中找出垂直标尺
    vector<double>	m_linesY;  //目的是得到所有近于水平的线，从其中找出水平标尺
    vector<linesGroup> m_rulerHGroups;		//横标尺线组
    vector<linesGroup> m_rulerVGroups;		//竖标尺线组

    double     m_dHRuler;   //测量得到平行标尺像素数
    double     m_dVRuler;   //测量得到垂直标尺像素数


    int        m_blockSize; //adaptiveThreshold
    int        m_constValue;//adaptiveThreshold
    //hough
    int        m_iThreshold;
    double     m_dMinLength;
    double     m_dMaxLineGap;
    //相近线分为一组
    int        m_iAngle;
    int        m_iDistance;
    //覆盖直线获得其他，直线线宽
    int        m_iLineWidth;
    //曲线，过滤小的形状
    //这里过滤面积使用的阈值是面积的平均值
    int        m_iContourLength;

    double     m_dActualRuler;//标尺实际长度cm
    double     m_dWidth;    //若知纸张参数
    double     m_dHeight;   //若知纸张参数

    double     m_fontScale;

};

#endif // MEASURETOOL_H
