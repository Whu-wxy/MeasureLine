#include "measuretool.h"
#include <QDebug>

MeasureTool::MeasureTool()
{
//    m_filePath = "";
//    m_blockSize = 25; //adaptiveThreshold
//    m_constValue = 10;//adaptiveThreshold
//    m_dWidth = 0;     //若知纸张参数
//    m_dHeight = 0;    //若知纸张参数
//    m_dActualRuler = 10; //标尺实际长度cm
//    //hough
//    m_iThreshold = 220;
//    m_dMinLength = 200;
//    m_dMaxLineGap = 50;
//    //相近线分为一组
//    m_iAngle = 40;
//    m_iDistance = 100;
//    //覆盖直线获得其他，直线线宽
//    m_iLineWidth = 25;
//    //曲线，过滤小的形状
//    m_iContourLength = 400;

    m_filePath = "";
    m_blockSize = 25; //adaptiveThreshold
    m_constValue = 10;//adaptiveThreshold
    m_dWidth = 0;     //若知纸张参数
    m_dHeight = 0;    //若知纸张参数
    m_dActualRuler = 4.48; //标尺实际长度cm
    //hough
    m_iThreshold = 150;
    m_dMinLength = 60;
    m_dMaxLineGap = 10;
    //相近线分为一组
    m_iAngle = 1.5;
    m_iDistance = 11;
    //覆盖直线获得其他，直线线宽
    m_iLineWidth = 25;
    //曲线，过滤小的形状
    m_iContourLength = 400;

    m_fontScale = 1.5;
}

MeasureTool::~MeasureTool()
{

}

void MeasureTool::startMeasure()
{
    m_srcImage = imread(m_filePath.toStdString());
    if (!m_srcImage.data)
    {
        cout << "读取失败";
        return;
    }
    Mat srcImageTemp;

//    pyrMeanShiftFiltering(m_srcImage, m_srcImage, 25, 20);//meanshift平滑,保护边缘效果好
//    imwrite("C:/Users/Administrator/Desktop/meanshift.JPG", m_srcImage);
//    return;

    m_srcImage.copyTo(srcImageTemp);
    m_srcImage.copyTo(m_repairedImg);
    projectRepair(srcImageTemp,m_repairedImg);
    imwrite("C:/Users/Administrator/Desktop/projectRepair.JPG", m_repairedImg);
  //  return;

    linesProcess(m_repairedImg);
    // /////////////////////////////////////
    // 找出标尺，标尺是在纸张靠近下边缘的横线
    // 纸张的下边缘有且只检测出一条横线
    // /////////////////////////////////////
    if(m_dWidth == 0)
    {
        m_dHRuler = findHRuler(m_repairedImg,m_rulerHGroups,m_linesY);
        m_dVRuler = findVRuler(m_repairedImg,m_rulerVGroups,m_linesX);
        if(m_dHRuler == 1 && m_dVRuler == 1)
            return;

        if(m_dHRuler == 1)
            m_dHRuler = m_dVRuler;
        if(m_dVRuler == 1)
            m_dVRuler = m_dHRuler;
    }
    measureAngle(m_repairedImg);
    Mat repairedImg2Curve = measureLength(m_repairedImg);
    findCurve(repairedImg2Curve,m_repairedImg);

    //存储结果
    imwrite("C:/Users/Administrator/Desktop/final.JPG", m_repairedImg);
    cout << "finish"<<endl;
}

//纸张透视校正，如果已知纸张长宽，则使用已知数据计算
void MeasureTool::projectRepair(Mat srcImg,Mat &dstImg)
{
    Mat srcGray;
    cvtColor(srcImg, srcGray, CV_RGB2GRAY);
   // equalizeHist(srcGray,srcGray);
    // 局部二值化
    Mat local;
    adaptiveThreshold(srcGray, local, 255, CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY_INV, m_blockSize, m_constValue);
    imwrite("C:/Users/Administrator/Desktop/threshold.JPG", local);
    // //////////////////////检测线////////////////////////
    vector<Vec4i> lines;
    HoughLinesP(local, lines, 1, CV_PI / 180, m_iThreshold, m_dMinLength,m_dMaxLineGap);

    //相近线分组
    vector<linesGroup> lineGroups;
    vector<linesGroup> rulerHGroups;		//横标尺线组
    vector<linesGroup> rulerVGroups;		//竖标尺线组
    vector<double>	linesX;
    vector<double>	linesY;

    line2Group(lineGroups,lines,0,0);
    clearOneLineGroup(lineGroups);//清除掉只有一条线的线组（曲线上可能会有很多单独的线）
    vector<linesGroup>::iterator it = lineGroups.begin();
    int r,g=50,b=10;
    int num=0;
    while (it != lineGroups.end())
    {
        linesGroup group = *it;
        vector<Vec4i> liness = group.getLines();
        vector<Point2f> linel = group.getMergedLine();
        num++;
        srand(num);
        r = rand() % 255;
        g = rand() % 255;
        b = rand() % 255;
//        for(int i=0;i<liness.size();i++)
//        {
//            Vec4i pp = liness.at(i);
//            line(m_srcImage,Point(pp[0],pp[1]),Point(pp[2],pp[3]),Scalar(r,g,b),1);
      //  }
      //  line(m_srcImage,linel[0],linel[1],Scalar(r,g,b),4);

        //横线竖线分类
        bool bNear = false;
        double groupX = group.getMidX();
        for(int i=0; i<linesX.size(); i++)
        {
            if(abs(groupX-linesX.at(i)) < 50)
                bNear = true;
        }
        if(!bNear)
        {
            vector<Point2f> aline = group.getMergedLine();
            Point2f p1 = aline[0];
            Point2f p2 = aline[1];
            int maxY = max(p1.y,p2.y);
            if(maxY > srcImg.rows/2)
            {
                linesX.push_back(groupX);
            }
        }

        bNear = false;
        double groupY = group.getMidY();
        for(int i=0; i<linesY.size(); i++)
        {
            if(abs(groupY-linesY.at(i)) < 50)
                bNear = true;
        }
        if(!bNear)
        {
            vector<Point2f> aline = group.getMergedLine();
            Point2f p1 = aline[0];
            Point2f p2 = aline[1];
            int minX = min(p1.x,p2.x);
            if(minX < srcImg.cols/2)
            {
                linesY.push_back(groupY);
            }
        }

        double aveA = abs(group.getLineA());
        if (aveA < 40)                         // //////////找出所有横线，作为备选标尺
            rulerHGroups.push_back(group);
        else if (90 - aveA < 40)			   // //////////找出所有竖线，作为备选标尺
            rulerVGroups.push_back(group);

        it++;
    }
    sort(linesX.begin(), linesX.end());//升序
    sort(linesY.begin(), linesY.end());//纸张边缘外不能有线

    double width, height;
    vector<Point2f> repairPoints;
    Point topLeftP(0,0);
    vector<Point2f> edgePoints = findEdgePoints(local, width, height);
    if(edgePoints.size() != 0)
        topLeftP = edgePoints.at(0);
    else
        return;

    for(int i=0;i<edgePoints.size();i++)
        circle(srcImg,edgePoints.at(i),8,Scalar(255,255,40),5);

    if(m_dWidth == 0)//使用已知纸张尺寸
    {
        //标尺发生长度变化，根据标尺进行修正
        double hRuler = findHRuler(m_repairedImg,rulerHGroups,linesY);
        double vRuler = findVRuler(m_repairedImg,rulerVGroups,linesX);
        cout<<"hhh:"<<hRuler<<endl;
        cout<<"vvv:"<<vRuler<<endl;
        double factor;
        if(hRuler >= vRuler)
        {
            factor = hRuler / vRuler;
            if(hRuler == 1 || vRuler == 1)
                factor = 1;
            cout<<"factor0:"<<factor;
            repairPoints.push_back(topLeftP);
            repairPoints.push_back(Point2f(topLeftP.x+width, topLeftP.y));
            repairPoints.push_back(Point2f(topLeftP.x+width, (topLeftP.y+height) * factor));
            repairPoints.push_back(Point2f(topLeftP.x,       (topLeftP.y+height) * factor));
        }
        else
        {
            factor = vRuler / hRuler;
            if(hRuler == 1 || vRuler == 1)
                factor = 1;
            cout<<"factor:"<<factor;
            repairPoints.push_back(topLeftP);
            repairPoints.push_back(Point2f((topLeftP.x+width) * factor, topLeftP.y));
            repairPoints.push_back(Point2f((topLeftP.x+width) * factor, topLeftP.y + height));
            repairPoints.push_back(Point2f(topLeftP.x, topLeftP.y + height));
        }
    }
    else
    {
        double factor = height / width;
        double paperFactor = m_dHeight / m_dWidth;
        if(factor <= paperFactor)
        {
            repairPoints.push_back(topLeftP);
            repairPoints.push_back(Point2f(topLeftP.x + m_dWidth, topLeftP.y));
            repairPoints.push_back(Point2f(topLeftP.x + m_dWidth, (topLeftP.y + m_dHeight)*paperFactor));
            repairPoints.push_back(Point2f(topLeftP.x, (topLeftP.y + m_dHeight)*paperFactor));
        }
        else
        {
            paperFactor = m_dWidth / m_dHeight;
            repairPoints.push_back(topLeftP);
            repairPoints.push_back(Point2f((topLeftP.x+m_dWidth)*paperFactor, topLeftP.y));
            repairPoints.push_back(Point2f((topLeftP.x+m_dWidth)*paperFactor, topLeftP.y + m_dHeight));
            repairPoints.push_back(Point2f(topLeftP.x, topLeftP.y + m_dHeight));
        }
    }
    Point2f src_points[] = { edgePoints.at(0), edgePoints.at(1), edgePoints.at(2), edgePoints.at(3) };
    Point2f dst_points[] = { repairPoints.at(0), repairPoints.at(1), repairPoints.at(2), repairPoints.at(3) };
    // ///////////////////////透视变换////////////////////////////////////
//    imwrite("C:/Users/Administrator/Desktop/1111.JPG", m_srcImage);
    Mat M = getPerspectiveTransform(src_points, dst_points);
    warpPerspective(srcImg, dstImg, M, srcImg.size(), INTER_LINEAR, BORDER_REPLICATE);
}

void MeasureTool::setPaperSize(double width, double height)
{
    m_dWidth = width;
    m_dHeight = height;
}

void MeasureTool::linesProcess(Mat &srcImg)
{
    Mat clone = m_repairedImg.clone();
    Mat clone2 = m_repairedImg.clone();
    Mat srcGray;
    cvtColor(srcImg, srcGray, CV_RGB2GRAY);
    // 局部二值化
    adaptiveThreshold(srcGray, repairedBiImg, 255, CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY_INV, m_blockSize, m_constValue);

    vector<Vec4i> lines;
    HoughLinesP(repairedBiImg, lines, 1, CV_PI / 180, m_iThreshold, m_dMinLength,m_dMaxLineGap);

    // 相近线分组
    line2Group(m_lineGroups, lines,0,0);
    clearOneLineGroup(m_lineGroups);//清除掉只有一条线的线组（曲线上可能会有很多单独的线）
    int r=40,g,b;
    int num=0;
    vector<linesGroup>::iterator it = m_lineGroups.begin();
    while (it != m_lineGroups.end())
    {
        linesGroup group = *it;
        vector<Vec4i> liness = group.getLines();
        num++;
        qsrand(num);
        r = rand() % 255;
        g = rand() % 255;
        b = rand() % 255;
        for(int i=0;i<liness.size();i++)
        {
            Vec4i pp = liness.at(i);
            line(clone2,Point(pp[0],pp[1]),Point(pp[2],pp[3]),Scalar(r,g,b),2);
        }
        imwrite("C:/Users/Administrator/Desktop/repaired0.JPG",clone2);

        // //////目的是获得纸张边缘
        bool bNear = false;

        double groupX = group.getMidX();
        for(int i=0; i<m_linesX.size(); i++)
        {
            if(abs(groupX-m_linesX.at(i)) < 50)
                bNear = true;
        }
        if(!bNear)
        {
            vector<Point2f> aline = group.getMergedLine();
            Point2f p1 = aline[0];
            Point2f p2 = aline[1];
            int maxY = max(p1.y,p2.y);
            if(maxY > srcImg.rows/2)
            {
                m_linesX.push_back(groupX);
            }
        }

        bNear = false;
        double groupY = group.getMidY();
        for(int i=0; i<m_linesY.size(); i++)
        {
            if(abs(groupY-m_linesY.at(i)) < 50)
                bNear = true;
        }
        if(!bNear)
        {
            vector<Point2f> aline = group.getMergedLine();
            Point2f p1 = aline[0];
            Point2f p2 = aline[1];
            int minX = min(p1.x,p2.x);
            if(minX < srcImg.cols/2)
            {
                m_linesY.push_back(groupY);
            }
        }
        double aveA = abs(group.getLineA());
        if (aveA < 40)                         // //////////找出所有横线，作为备选标尺
            m_rulerHGroups.push_back(group);
        else if (90 - aveA < 40)			   // //////////找出所有竖线，作为备选标尺
            m_rulerVGroups.push_back(group);

        srand(r);
        r = rand() % 255;
        g = rand() % 255;
        b = rand() % 255;
        vector<Point2f> aline = group.getMergedLine();  //合并线组，得到一条线
        line(clone,aline.at(0),aline.at(1),Scalar(r,g,b),4);
        it++;
    }

    imwrite("C:/Users/Administrator/Desktop/repaired.JPG",clone);
    sort(m_linesX.begin(), m_linesX.end());//升序
    sort(m_linesY.begin(), m_linesY.end());//纸张边缘外不能有线
}

//相近线分为一组
void MeasureTool::line2Group(vector<linesGroup> &lineGroups, vector<Vec4i> &lines, double angle, double dis)
{
    if(angle == 0 || dis == 0)
    {
        angle = m_iAngle;
        dis = m_iDistance;
    }
    vector<Vec4i>::reverse_iterator iter = lines.rbegin();
    while (iter != lines.rend())
    {
        linesGroup group;
        Vec4i aline = *iter;
        group.setLine(aline);
        iter = vector<Vec4i>::reverse_iterator(lines.erase((++iter).base()));
        vector<Vec4i>::reverse_iterator iter2 = lines.rbegin();
        while (iter2 != lines.rend())
        {
            if (group.addLine(*iter2, angle, dis))
                iter2 = vector<Vec4i>::reverse_iterator(lines.erase((++iter2).base()));  //从lines中移除相近线
            else
                iter2++;
        }
        group.group2Line();
        lineGroups.push_back(group);
        iter = lines.rbegin();
    }
}

void MeasureTool::clearOneLineGroup(vector<linesGroup> &lineGroups)
{
    vector<linesGroup>::reverse_iterator iter = lineGroups.rbegin();
    while (iter != lineGroups.rend())
    {
        linesGroup group = *iter;
        int linesCount = group.getLinesCount();
        if(linesCount == 1)
        {
            iter = vector<linesGroup>::reverse_iterator(lineGroups.erase((++iter).base()));
        }
        else
            iter++;
    }
}

//两条线段是否有交点，如果有，求出交点
bool MeasureTool::get_line_intersection(float p0_x, float p0_y, float p1_x, float p1_y,
    float p2_x, float p2_y, float p3_x, float p3_y, float *i_x, float *i_y)
{
    float s1_x, s1_y, s2_x, s2_y;
    s1_x = p1_x - p0_x;     s1_y = p1_y - p0_y;
    s2_x = p3_x - p2_x;     s2_y = p3_y - p2_y;

    float s, t;
    s = (-s1_y * (p0_x - p2_x) + s1_x * (p0_y - p2_y)) / (-s2_x * s1_y + s1_x * s2_y);
    t = (s2_x * (p0_y - p2_y) - s2_y * (p0_x - p2_x)) / (-s2_x * s1_y + s1_x * s2_y);

    if (s >= 0 && s <= 1 && t >= 0 && t <= 1)
    {
        // Collision detected
        if (i_x != NULL)
            *i_x = p0_x + (t * s1_x);
        if (i_y != NULL)
            *i_y = p0_y + (t * s1_y);
        return 1;
    }

    return 0; // No collision
}

//二值图
vector<Point2f> MeasureTool::findEdgePoints(Mat img, double &width, double &height)
{
    Mat element3 = getStructuringElement(MORPH_RECT, Size(3,3));
    Mat element5 = getStructuringElement(MORPH_RECT, Size(5,5));
    Mat element7 = getStructuringElement(MORPH_RECT, Size(7,7));
    dilate(img, img, element5);   // 膨胀

    vector<Point2f> crossPoints;

    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;

    findContours(img, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));

    int length = 600;
    double maxArea = 0;
    int j=0;
    for(int i = 0; i < contours.size(); i++)
    {
        double l = arcLength(contours[i], true); //是否为封闭曲线
        double area = contourArea(contours[i]);
        if(l > length)  //过滤小的形状
        {
            if(area < 500)
                continue;
            if(area > maxArea)
            {
                maxArea = area;
                j = i;
            }
        }
    }
    Mat maxContour = img.clone();

    vector<vector<Point>> maxcon;
    //相近线分组
    vector<linesGroup> lineGroups;

    maxcon.push_back(contours[j]);
    drawContours(maxContour, maxcon, 0, Scalar(255), CV_FILLED);
    erode(maxContour, maxContour, element7);

    Canny(maxContour, maxContour, 40, 80);
    dilate(maxContour, maxContour, element3);   // 膨胀
    vector<Vec4i> lines;
    HoughLinesP(maxContour, lines, 1, CV_PI / 180, 220, 100,50);
    line2Group(lineGroups,lines,5,50);

    //一组线合并成一条线
    linesGroup top;
    linesGroup bottom;
    linesGroup left;
    linesGroup right;
    double minX=img.cols, maxX=0, minY=img.rows, maxY=0;
    vector<linesGroup>::iterator it = lineGroups.begin();
    int r=40,g,b;
    int num = 0;
    while (it != lineGroups.end())
    {
        num++;
        qsrand(num);
        r = qrand()%255;
        g = qrand()%255;
        b = qrand()%255;
        linesGroup group = *it;
        vector<Point2f> aline= group.getMergedLine();
     //   line(m_srcImage,aline[0],aline[1],Scalar(r,g,b),4);

        if(group.getMidX() < minX)
        {
            minX = group.getMidX();
            left = group;
        }
        if(group.getMidX() > maxX)
        {
            maxX = group.getMidX();
            right = group;
        }
        if(group.getMidY() < minY)
        {
            minY = group.getMidY();
            top = group;
        }
        if(group.getMidY() > maxY)
        {
            maxY = group.getMidY();
            bottom = group;
        }
        it++;
    }
    vector<Point2f>  topLine;
    topLine = top.getMergedLine();
    Point2f topA = topLine.at(0);
    Point2f topB = topLine.at(1);
    double topL = sqrt(pow(topA.x - topB.x, 2.0) + pow(topA.y - topB.y, 2.0));

    vector<Point2f>  bottomLine;
    bottomLine = bottom.getMergedLine();
    Point2f bottomA = bottomLine.at(0);
    Point2f bottomB = bottomLine.at(1);
    double bottomL = sqrt(pow(bottomA.x - bottomB.x, 2.0) + pow(bottomA.y - bottomB.y, 2.0));

    //得到最长的横线长
    if (topL >= bottomL)
        width = topL;
    else
        width = bottomL;

    vector<Point2f>  leftLine;
    leftLine = left.getMergedLine();
    Point2f leftA = leftLine.at(0);
    Point2f leftB = leftLine.at(1);
    double leftL = sqrt(pow(leftA.x - leftB.x, 2.0) + pow(leftA.y - leftB.y, 2.0));

    vector<Point2f>  rightLine;
    rightLine = right.getMergedLine();
    Point2f rightA = rightLine.at(0);
    Point2f rightB = rightLine.at(1);
    double rightL = sqrt(pow(rightA.x - rightB.x, 2.0) + pow(rightA.y - rightB.y, 2.0));
    //得到最长的竖线长
    if (leftL >= rightL)
        height = leftL;
    else
        height = rightL;
    // ///////////////////////////////求四个交点///////////////////////////////////
    Point2f topLeft = getCrossPoint(topA, topB, leftA, leftB);
    Point2f topRight = getCrossPoint(topA, topB, rightA, rightB);
    Point2f bottomRight = getCrossPoint(bottomA, bottomB, rightA, rightB);
    Point2f bottomLeft = getCrossPoint(bottomA, bottomB, leftA, leftB);

    crossPoints.push_back(topLeft);
    crossPoints.push_back(topRight);
    crossPoints.push_back(bottomRight);
    crossPoints.push_back(bottomLeft);
    return crossPoints;
}

Point2f MeasureTool::getCrossPoint(Point2f p1, Point2f p2, Point2f p3, Point2f p4)
{
    double ka, kb;
    ka = (double)(p2.y - p1.y) / (double)(p2.x - p1.x); //求出LineA斜率
    kb = (double)(p4.y - p3.y) / (double)(p4.x - p3.x); //求出LineB斜率

    Point2f crossPoint;
    crossPoint.x = (ka*p1.x - p1.y - kb*p3.x + p3.y) / (ka - kb);
    crossPoint.y = (ka*kb*(p1.x - p3.x) + ka*p3.y - kb*p1.y) / (ka - kb);
    return crossPoint;
}

double MeasureTool::findHRuler(Mat &img, vector<linesGroup> rulerHGroups,vector<double> linesY)
{
    double hRuler = 1;
    linesGroup group;
    if (rulerHGroups.size() > 2)
    {
        for (int i = 0; i < rulerHGroups.size(); i++)
        {
            group = rulerHGroups.at(i);
            int groupX = group.getMidX();
            if(groupX > img.cols/2)   //尝试过滤标尺右侧的横线
                continue;

            if (linesY.at(linesY.size()-2) == group.getMidY()) //倒数第二根横线
            {
                vector<Point2f>  mergedLine = group.getMergedLine();
                Point a = mergedLine.at(0);
                Point a2 = mergedLine.at(1);
                double x1, y1, x2, y2;
                x1 = a.x;
                y1 = a.y;
                x2 = a2.x;
                y2 = a2.y;

                hRuler = sqrt(pow(x1 - x2, 2.0) + pow(y1 - y2, 2.0));

                putText(img, "Ruler", Point(group.getMidX(), group.getMidY())
                        , cv::FONT_HERSHEY_COMPLEX, m_fontScale, Scalar(10, 255, 255), 2, 8, 0);
            }
        }
    }
    return hRuler;
}

double MeasureTool::findVRuler(Mat &img, vector<linesGroup> rulerVGroups,vector<double> linesX)
{
    double vRuler = 1;
    linesGroup group;
    if (rulerVGroups.size() > 2)
    {
        for (int i = 0; i < rulerVGroups.size(); i++)
        {
            group = rulerVGroups.at(i);
            int groupY = group.getMidY();
            if(groupY < img.rows/2)   //尝试过滤标尺上侧的竖线
                continue;

            if (linesX.at(1) == group.getMidX()) //第二根竖线
            {
                vector<Point2f>  mergedLine = group.getMergedLine();
                Point a = mergedLine.at(0);
                Point a2 = mergedLine.at(1);
                double x1, y1, x2, y2;
                x1 = a.x;
                y1 = a.y;
                x2 = a2.x;
                y2 = a2.y;

                vRuler = sqrt(pow(x1 - x2, 2.0) + pow(y1 - y2, 2.0));

                putText(img, "Ruler", Point(group.getMidX(), group.getMidY())
                        , cv::FONT_HERSHEY_COMPLEX,m_fontScale, Scalar(10, 255, 255), 2, 8, 0);
            }
        }
    }
    return vRuler;
}

Mat MeasureTool::measureLength(Mat &img)
{
    Mat imgClone = repairedBiImg.clone();
    vector<linesGroup>::iterator it = m_lineGroups.begin();

    while (it != m_lineGroups.end())
    {
        linesGroup group = *it;
        vector<Point2f>  mergedLine = group.getMergedLine();
        Point a = mergedLine.at(0);
        Point a2 = mergedLine.at(1);
        double x1, y1, x2, y2;
        x1 = a.x;
        y1 = a.y;
        x2 = a2.x;
        y2 = a2.y;

        line(img, Point(x1, y1), Point(x2, y2), Scalar(0,50,200), 2, CV_AA);

        //通过覆盖，消除直线。。。
        Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));//参数设置要防止曲线被消除
        // 腐蚀操作
      //  erode(imgClone, imgClone, element);
        line(imgClone, Point(x1, y1), Point(x2, y2), Scalar(0), m_iLineWidth, CV_AA); //调整参数
        // 膨胀
     //   dilate(imgClone, imgClone, element);
        imwrite("C:/Users/Administrator/Desktop/repairedBiSubLines.JPG", imgClone);

        double factor = 1;
        double length = 0;
        double aveRuler = (m_dHRuler + m_dVRuler) / 2;
        if(m_dWidth == 0) //纸张参数未知
        {
            if(m_dHRuler > m_dVRuler)
            {
                factor = m_dHRuler / m_dVRuler;
                if(m_dHRuler == 1 || m_dVRuler == 1)
                    factor = 1;
                length = sqrt(pow(x1 - x2, 2.0) + pow((y1 - y2)*factor, 2.0));
            }
            else
            {
                factor = m_dVRuler / m_dHRuler;
                if(m_dHRuler == 1 || m_dVRuler == 1)
                    factor = 1;
                length = sqrt(pow((x1 - x2)*factor, 2.0) + pow(y1 - y2, 2.0));
            }
            length = (length / aveRuler) * m_dActualRuler;
        }
        string strLength = to_string(length);
        putText(img, strLength, group.getMidPosition(), cv::FONT_HERSHEY_COMPLEX,m_fontScale, Scalar(10, 10, 255), 2, 8, 0);
        it++;
    }
    return imgClone;
}

// ///////////////////两线有交点，计算夹角//////////////
void MeasureTool::measureAngle(Mat &img)
{
    vector<Point> intersections;
    for (int i = 0; i < m_lineGroups.size(); i++)
    {
        linesGroup group = m_lineGroups.at(i);
        float x1, y1, x2, y2, a1, b1, a2, b2, xi, yi;
        vector<Point2f> aline = group.getMergedLine();
        x1 = aline.at(0).x;
        y1 = aline.at(0).y;
        x2 = aline.at(1).x;
        y2 = aline.at(1).y;
        if(i == m_lineGroups.size()-1)
            break;
        for (int j = i + 1; j < m_lineGroups.size(); j++)
        {
            linesGroup anoGroup = m_lineGroups.at(j);
            vector<Point2f> anoline = anoGroup.getMergedLine();
            a1 = anoline.at(0).x;
            b1 = anoline.at(0).y;
            a2 = anoline.at(1).x;
            b2 = anoline.at(1).y;
            if (get_line_intersection(x1, y1, x2, y2, a1, b1, a2, b2, &xi, &yi))
            {
                intersections.push_back(Point(xi, yi));
                double angle1 = atan((double)(y2 - y1) / (x2 - x1));
                angle1 = angle1 * 180 / CV_PI;
                double angle2 = atan((double)(b2 - b1) / (a2 - a1));
                angle2 = angle2 * 180 / CV_PI;
                double angle = abs(angle2 - angle1);
                if (angle > 90)
                    angle = 180 - angle;

                string strAngle = to_string(angle);
                putText(img, strAngle, Point(xi + 30, yi + 30), cv::FONT_HERSHEY_COMPLEX, m_fontScale, Scalar(255, 10, 10), 2, 8, 0);
                circle(img, Point(xi, yi), 10, Scalar(240, 20, 50), 10, 8);
            }
        }
    }
}

void MeasureTool::findCurve(Mat BiImg, Mat &img2Draw)
{
    Mat element = getStructuringElement(MORPH_RECT, Size(3, 3));
    erode(BiImg, BiImg, element);    // 腐蚀
    dilate(BiImg, BiImg, element);   // 膨胀

    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(BiImg, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));


    vector<RotatedRect> minRects;

    QList<double> areas;
    for(int i = 0; i < contours.size(); i++)
    {
        double l = arcLength(contours[i], true); //是否为封闭曲线
        if(l > m_iContourLength)  //过滤小的形状
        {
            RotatedRect rect = minAreaRect(Mat(contours[i]));
            minRects.push_back(rect);
            Point2f rectPoints[4];
            rect.points(rectPoints);

            vector<Point2f> p;
            p.push_back(rectPoints[0]);
            p.push_back(rectPoints[1]);
            p.push_back(rectPoints[2]);
            p.push_back(rectPoints[3]);
            double area = contourArea(p);
            areas.append(area);
        }
    }
    double averageArea = 0;
    for(int i=0; i< areas.count(); i++)
    {
        averageArea += areas.at(i);
    }
    averageArea = averageArea / areas.count();

    vector<RotatedRect> targetRects;

    for(int i=0; i<minRects.size(); i++)
    {
        Point2f rectPoints[4];
        minRects[i].points(rectPoints);

        vector<Point> p;
        p.push_back(rectPoints[0]);
        p.push_back(rectPoints[1]);
        p.push_back(rectPoints[2]);
        p.push_back(rectPoints[3]);

        double area = contourArea(p);
        if(area > averageArea)
        {
            targetRects.push_back(minRects[i]);
            for (int j = 0; j < 4; j++)
            {
                line(img2Draw, rectPoints[j], rectPoints[(j+1)%4], Scalar(0,200,0), 2, 8, 0);
            }
        }
    }
    // /////////////////////////////
    QList<Mat> roiList;
    QList<CircleData> circleList;
    for(int i=0;i<targetRects.size(); i++)
    {
        Point2f rectPoints[4];
        targetRects[i].points(rectPoints);

        vector<Point> p;
        p.push_back(rectPoints[0]);
        p.push_back(rectPoints[1]);
        p.push_back(rectPoints[2]);
        p.push_back(rectPoints[3]);

        Mat roi;
        Mat mask = Mat::zeros(BiImg.rows,BiImg.cols, CV_8UC1);
        vector<vector<Point>> contour0;
        contour0.push_back(p);
        drawContours(mask, contour0, 0, Scalar(255), CV_FILLED);
        BiImg.copyTo(roi, mask);
        roiList.append(roi);

        double k1 = (rectPoints[0].y - rectPoints[1].y) / (rectPoints[0].x - rectPoints[1].x);
        double a1 = atan(k1) * 180 / CV_PI;
        double l1 = sqrt(pow(rectPoints[0].x - rectPoints[1].x, 2.0) + pow(rectPoints[0].y - rectPoints[1].y, 2.0));
        double l2 = sqrt(pow(rectPoints[2].x - rectPoints[1].x, 2.0) + pow(rectPoints[2].y - rectPoints[1].y, 2.0));

        vector<Point> threePoints;
        CircleData circle;
// ///////////////////////////////////////
        //横型用竖扫描线，竖型用横扫描线
        //扫描结果有问题，目前不用
//        if(abs(a1) <= 45)
//        {
//            if(l1 >= l2)
//                threePoints = scanToThreePoints(roi,true,p);//横
//            else
//                threePoints = scanToThreePoints(roi,false,p);//竖
//        }
//        else
//        {
//            if(l1 >= l2)
//                threePoints = scanToThreePoints(roi,false,p);//竖
//            else
//                threePoints = scanToThreePoints(roi,true,p);//横
//        }
// //////////////////////////////////////////////////////////
        if(l1 >= l2)
        {//这里要改参数,不确定曲线方向
            threePoints.push_back(rectPoints[2]);
            threePoints.push_back(rectPoints[3]);
            int x = (rectPoints[1].x + rectPoints[0].x) / 2;
            int y = (rectPoints[1].y + rectPoints[0].y) / 2;
            threePoints.push_back(Point(x,y));
        }
        else
        {//这里要改参数,不确定曲线方向
            threePoints.push_back(rectPoints[1]);
            threePoints.push_back(rectPoints[2]);
            int x = (rectPoints[0].x + rectPoints[3].x) / 2;
            int y = (rectPoints[0].y + rectPoints[3].y) / 2;
            threePoints.push_back(Point(x,y));
        }
        if(threePoints.size() == 3)
        {
            line(img2Draw,threePoints[0],threePoints[1],Scalar(255,0,0),4);
            line(img2Draw,threePoints[2],threePoints[1],Scalar(255,0,0),4);
            line(img2Draw,threePoints[0],threePoints[2],Scalar(255,0,0),4);
            circle = findCircle1(threePoints);
            circleList.append(circle);
        }
        // /////////////////
    }

    for(int i=0; i<circleList.count(); i++)
    {
        CircleData circle0 = circleList.at(i);
//        circle(img2Draw,circle0.center,circle0.radius,Scalar(0,255,0),4);
//        circle(img2Draw,circle0.center,8,Scalar(0,255,0),4);
        putText(m_srcImage,to_string(circle0.radius), circle0.center
                , cv::FONT_HERSHEY_COMPLEX, m_fontScale, Scalar(10, 255, 255), 2, 8, 0);
    }
}


//存在问题，目前不用
vector<Point> MeasureTool::scanToThreePoints(Mat img, bool bHori, vector<Point> points)
{
    vector<Point> threePoints;
    QList<Point> pList;
    Point p0 = points[0];
    Point p1 = points[1];
    Point p2 = points[2];
    bool bFind = false;
    int x ,y;
    Point p;
    if(bHori)
    {
        int minX = p0.x, maxX = p0.x, midX = 0;
        if(p1.x < minX)
            minX = p1.x;
        if(p2.x < minX)
            minX = p2.x;
        if(p1.x > maxX)
            maxX = p1.x;
        if(p2.x > maxX)
            maxX = p2.x;
        midX = (minX + maxX) / 2;
        // 1
        for(int i=minX; i<img.cols; i++) //横向移动
        {
            for(int j=0; j<img.rows; j++)
            {
                int value = img.at<int>(i,j);
                if(value == 1)
                {
                    bFind = true;
//                    for(int k=0; k<pList.count(); k++)
//                    {
//                        if(pList.at(k).y)
                    pList.append(Point(i,j));
 //                   }
                }
            }
            if(bFind)
                break;
        }
        if(pList.count() == 0)
            return threePoints;
        x = pList.at(0).x;
        if(pList.count() > 1)
            y = (pList.at(0).y + pList.at(pList.count()-1).y) / 2;
        else
            y = pList.at(0).y;
        p = Point(x,y);
        threePoints.push_back(p);
        pList.clear();
        bFind = false;
        //2
        for(int i=maxX; i<img.cols; i--) //横向移动
        {
            for(int j=0; j<img.rows; j++)
            {
                int value = img.at<int>(i,j);
                if(value == 1)
                {
                    bFind = true;
                    pList.append(Point(i,j));
                }
            }
            if(bFind)
                break;
        }

        x = pList.at(0).x;
        if(pList.count() > 1)
            y = (pList.at(0).y + pList.at(pList.count()-1).y) / 2;
        else
            y = pList.at(0).y;
        p = Point(x,y);
        threePoints.push_back(p);
        pList.clear();
        bFind = false;
        //3
        for(int i=midX; i<img.cols; i++) //横向移动
        {
            for(int j=0; j<img.rows; j++)
            {
                int value = img.at<int>(i,j);
                if(value == 1)
                {
                    bFind = true;
                    pList.append(Point(i,j));
                }
            }
            if(bFind)
                break;
        }

        x = pList.at(0).x;
        if(pList.count() > 1)
            y = (pList.at(0).y + pList.at(pList.count()-1).y) / 2;
        else
            y = pList.at(0).y;
        p = Point(x,y);
        threePoints.push_back(p);
        pList.clear();
        return threePoints;
    }
    else
    {
        int minY = p0.y, maxY = p0.y, midY = 0;
        if(p1.y < minY)
            minY = p1.y;
        if(p2.y < minY)
            minY = p2.y;
        if(p1.y > maxY)
            maxY = p1.y;
        if(p2.y > maxY)
            maxY = p2.y;
        midY = (minY + maxY) / 2;
        // 1
        for(int i=minY; i<img.rows; i++) //竖向移动
        {
            for(int j=0; j<img.cols; j++)
            {
                int value = img.at<int>(j,i);
                if(value == 1)
                {
                    bFind = true;
                    pList.append(Point(j,i));
                }
            }
            if(bFind)
                break;
        }
        if(pList.count() == 0)
            return threePoints;
        y = pList.at(0).y;
        if(pList.count() > 1)
            x = (pList.at(0).x + pList.at(pList.count()-1).x) / 2;
        else
            x = pList.at(0).x;
        p = Point(x,y);
        threePoints.push_back(p);
        pList.clear();
        bFind = false;
        //2
        for(int i=maxY; i<img.rows; i--) //竖向移动
        {
            for(int j=0; j<img.cols; j++)
            {
                int value = img.at<int>(j,i);
                if(value == 1)
                {
                    bFind = true;
                    pList.append(Point(j,i));
                }
            }
            if(bFind)
                break;
        }

        y = pList.at(0).y;
        if(pList.count() > 1)
            x = (pList.at(0).x + pList.at(pList.count()-1).x) / 2;
        else
            x = pList.at(0).x;
        p = Point(x,y);
        threePoints.push_back(p);
        pList.clear();
        bFind = false;
        //3
        for(int i=midY; i<img.rows; i++) //竖向移动
        {
            for(int j=0; j<img.cols; j++)
            {
                int value = img.at<int>(j,i);
                if(value == 1)
                {
                    bFind = true;
                    pList.append(Point(j,i));
                }
            }
            if(bFind)
                break;
        }

        y = pList.at(0).y;
        if(pList.count() > 1)
            x = (pList.at(0).x + pList.at(pList.count()-1).x) / 2;
        else
            x = pList.at(0).x;
        p = Point(x,y);
        threePoints.push_back(p);
        pList.clear();
        return threePoints;
    }
}

CircleData MeasureTool::findCircle1(vector<Point> threePoints)
{
    Point pt1 = threePoints[0];
    Point pt2 = threePoints[1];
    Point pt3 = threePoints[2];
    //定义两个点，分别表示两个中点
    Point midpt1, midpt2;
    //求出点1和点2的中点
    midpt1.x = (pt2.x + pt1.x) / 2;
    midpt1.y = (pt2.y + pt1.y) / 2;
    //求出点3和点1的中点
    midpt2.x = (pt3.x + pt1.x) / 2;
    midpt2.y = (pt3.y + pt1.y) / 2;
    //求出分别与直线pt1pt2，pt1pt3垂直的直线的斜率
    float k1 = -(float)(pt2.x - pt1.x) / (pt2.y - pt1.y);
    float k2 = -(float)(pt3.x - pt1.x) / (pt3.y - pt1.y);
    //然后求出过中点midpt1，斜率为k1的直线方程（既pt1pt2的中垂线）：y - midPt1.y = k1( x - midPt1.x)
    //以及过中点midpt2，斜率为k2的直线方程（既pt1pt3的中垂线）：y - midPt2.y = k2( x - midPt2.x)
    //定义一个圆的数据的结构体对象CD
    CircleData CD;
    //连立两条中垂线方程求解交点得到：
    CD.center.x = (midpt2.y - midpt1.y - k2* midpt2.x + k1*midpt1.x) / (float)(k1 - k2);
    CD.center.y = midpt1.y + k1*(midpt2.y - midpt1.y - k2*midpt2.x + k2*midpt1.x) /(float) (k1 - k2);
    //用圆心和其中一个点求距离得到半径：
    CD.radius = sqrtf((CD.center.x - pt1.x)*(CD.center.x - pt1.x) + (CD.center.y - pt1.y)*(CD.center.y - pt1.y));
    return CD;
}
