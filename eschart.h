#ifndef ESCHART_H
#define ESCHART_H

#include <vector>
using namespace std;

#include <QColor>
#include <QPainter>
#include <QPainterPath>
#include <QRect>
#include <QtDebug>
#include <QPaintEvent>
#include <QtWidgets>
#include "pubfun.h"

typedef struct _ESPoint
{
    int index;
    QColor color;
    int paintWidth;
    double radius;

    bool bSelPoint;


    QPointF pos;

    _ESPoint(QPointF pf):
        radius(1),
        color(QColor(255,0,0)),
        paintWidth(5),
        bSelPoint(false)
    {
        pos = pf;
    }

}ItemPoint;

typedef struct _ESCurve
{
    QString sName;
    vector<_ESPoint> ESpoints;
    QColor color;
    int penWidth;
    QString sCaption;

    //temp variant
    QRect rcMain;
    vector<_ESPoint> ESWndPoint;
    vector<_ESPoint> ESWndBezierPoint;
    bool bSmooth;

    _ESCurve()
        :bSmooth(true)
        ,penWidth(1)
    {

    }

    void draw(QPainter &p)
    {
        if(bSmooth)
        {
            size_t n = ESWndBezierPoint.size();
            if(n <= 1)
                return;

            p.setPen(QPen(color,penWidth));

            for(size_t i=1;i<n;i++){
                p.drawLine(ESWndBezierPoint[i-1].pos, ESWndBezierPoint[i].pos);
            }
            p.setPen(QPen(color,penWidth*2));
            for(size_t i=1;i<n;i++){
                p.drawPoint(ESWndBezierPoint[i-1].pos);
            }
        }
        else
        {
            size_t n = ESWndPoint.size();
            if(n <= 1)
                return;

            p.setPen(QPen(color,penWidth));

            for(size_t i=1;i<n;i++){
                p.drawLine(ESWndPoint[i-1].pos, ESWndPoint[i].pos);
            }
            p.setPen(QPen(color,penWidth+5));
            for(size_t i=1;i<n;i+=5){
                p.drawPoint(ESWndPoint[i-1].pos);
            }
        }

    }

}_ESCurve;

typedef struct _ItemCurve
{
    //property
    QString sName;
    vector<QPointF> vectOriPoint;
    QColor color;
    int penWidth;
    QString sCaption;

    //temp variant
    QRect rcMain;
    vector<QPointF> vectWndPoint;
    vector<QPointF> vectWndBezierPoint;
    bool bSmooth;

    _ItemCurve()
        :bSmooth(true)
        ,penWidth(1)
    {

    }

//    void draw(QPainterPath* p)
//    {
//        if(p == nullptr)
//            return;

//        size_t n = vectWndPoint.size();
//        if(n <= 1)
//            return;

//        p->moveTo(vectWndPoint[0]);
//        for(size_t i=1;i<n;i++)
//            p->lineTo(vectWndPoint[i]);
//    }

    void draw(QPainter &p)
    {
        if(bSmooth)
        {
            size_t n = vectWndBezierPoint.size();
            if(n <= 1)
                return;

            p.setPen(QPen(color,penWidth));

            for(size_t i=1;i<n;i++){
                p.drawLine(vectWndBezierPoint[i-1], vectWndBezierPoint[i]);
            }
            p.setPen(QPen(color,penWidth*2));
            for(size_t i=1;i<n;i++){
                p.drawPoint(vectWndBezierPoint[i-1]);
            }
        }
        else
        {
            size_t n = vectWndPoint.size();
            if(n <= 1)
                return;

            p.setPen(QPen(color,penWidth));

            for(size_t i=1;i<n;i++){
                p.drawLine(vectWndPoint[i-1], vectWndPoint[i]);
            }
            p.setPen(QPen(color,penWidth+5));
            for(size_t i=1;i<n;i+=5){
                p.drawPoint(vectWndPoint[i-1]);
            }
        }

    }

    static bool getYOnLineByX(double x0, double y0, double x1, double y1, double x, double &y)
    {
        double A = x0 - x1;
        double B = x - x0;
        if(abs(A-B) < 0.00001)
            return false;

        y = (y0*A - y1*B)/(A-B);
        return true;
    }

    bool getValueByX(double xValue, double &yValue)
    {
        size_t n = vectOriPoint.size();
        for(size_t i=0;i<n-1;i++)
        {
            if(xValue >= vectOriPoint[i].x() && xValue <= vectOriPoint[i+1].x())
            {
                return getYOnLineByX(vectOriPoint[i].x(),
                              vectOriPoint[i].y(),
                              vectOriPoint[i+1].x(),
                              vectOriPoint[i+1].y(),
                              xValue,
                              yValue);
            }
        }

        return false;
    }


    bool smooth()
    {
        vector<int> mapIDs;

        int n = vectWndPoint.size();
        if(n < 3)
            return false;

        //smooth:
        PubFun::SmoothPolyLine2(vectWndPoint, 170.0);

        //bezier:
        n = vectWndPoint.size();
        vector<Ogre::Vector2> ov;
        for(int i=0;i<n;i++)
        {
            Ogre::Vector2 t(vectWndPoint[i].x(), vectWndPoint[i].y());
            ov.push_back(t);
        }

        vector<Ogre::Vector2> tResult;

        PubFun::bezier_parsePolyline(ov,n,tResult,0.2,mapIDs);

        vectWndBezierPoint.clear();
        size_t m = tResult.size();
        for(size_t i=0;i<m;i++)
        {
            vectWndBezierPoint.push_back(QPointF(tResult[i].x, tResult[i].y));
        }

        //thin:
        PubFun::thinPolyLineByAngle(vectWndBezierPoint);

        return true;
    }

}ItemCurve;

typedef struct _ItemLine
{
    //property
    QString sName;
    double xOriPos;
    QColor color;
    int penWidth;
    QString sCaption;

    QString sLeftLineName;
    QString sRightLineName;

    //temp variant
    QRect rcMain;
    double xWndPos;
    double wndTop;
    double wndBtm;

    void draw(QPainterPath* p)
    {
        if(p == nullptr)
            return;

        p->moveTo(xWndPos, wndTop);
        p->moveTo(xWndPos, wndBtm);
    }

    void draw(QPainter& p)
    {
        //draw line
        p.setPen(QPen(color,penWidth));
        p.drawLine(QPointF(xWndPos, wndTop), QPointF(xWndPos, wndBtm));

        //draw caption
        if(sCaption.isEmpty() == false)
        {
            QRect txtRc;
            txtRc.setLeft(xWndPos);
            txtRc.setRight(xWndPos + 100);
            txtRc.setTop(wndBtm - 100);
            txtRc.setBottom(wndBtm);

            p.drawText(txtRc, Qt::AlignLeft|Qt::AlignBottom, sCaption);
        }

    }

    bool isPointOnMe(const QPoint &pt)
    {
        //qDebug() << "pt = " << pt << "; x="<< xWndPos<<"; Y0=" << wndTop << "; Y1=" << wndBtm;

        int r = 3;
        if(pt.x() < xWndPos-r || pt.x() > xWndPos+r)
            return false;

        if(pt.y() < wndTop || pt.y() > wndBtm)
            return false;

        return true;
    }

}ItemLine;

class ESChart : public QObject
{
    Q_OBJECT

public:
    ESChart(QWidget *parent);

    ~ESChart();

private: //data

    //curves
    vector< ItemCurve* > m_vectCurve;

    //lines
    vector<ItemLine*> m_vectLine;

signals:
    void lineMoved(QString sLineName, double lineValueX);

public: //property:

    //which widget the chart draw on?
    QWidget *m_parent;

    //lines can be moved by mouse?
    bool m_bEnableMoveLine;

    //is visiable ?
    bool m_bShow;

    //rect on the widget
    QRect m_rc;

    //Curve Expain Width
    int m_nCurveExpainW;

    //tick mark text, tick mark line, curve explain text color
    QColor m_rulerColor;

    //main area background color
    QColor m_backColor;

    //title text color
    QString m_sRulerTitleFontName;
    //title text font size
    int m_nRulerTitleFontSize;

    //Tick text font size
    int m_nTickMarkFontSize;
    //length of short TickMark line
    int m_nTickMarkLineShortLength;
    //length of long TickMark line
    int m_nTickMarkLineLongLength;

    //number count after dot for hor tick mark text
    int m_nHorValueDot;
    //hor value start on ruler
    double m_dHorValueStart;
    //hor value end on ruler
    double m_dHorValueEnd;
    //hor title
    QString m_sHorTitle;
    //length of short(small) cell on ruler
    double m_dHorStepShort;
    //length of long(big) cell on ruler,we will draw tick mark text at this position
    double m_dHorStepLong;

    int m_nVerValueDot;
    double m_dVerValueStart;
    double m_dVerValueEnd;
    QString m_sVerTitle;
    double m_dVerStepShort;
    double m_dVerStepLong;

private:
    bool m_bValid;

    //rects:
    QRect m_rcHorTitleText;
    QRect m_rcHorRulerText;
    QRect m_rcHorRulerLine;

    QRect m_rcVerTitleText;
    QRect m_rcVerRulerText;
    QRect m_rcVerRulerLine;

    QRect m_rcMain;

    QRect m_rcCurveExpain;

    //temp var:
    int m_tickTxtWidth;
    int m_tickTxtHeight;

    int m_nPressLineID;
    QPoint m_downPoint;

    QList<_ESPoint*> pointList;
    _ESPoint* currentPoint;
private:
    void calculateRect();

    void drawVerText(QPainter &p);

    void drawVerRuler(QPainter &p);
    void drawHorRuler(QPainter &p);

    void drawExplain(QPainter &p);

    QString number2Txt(double value, int nValueDot);

    void frameRc(QPainter &p, QRect &rc, QColor clor,bool bFill=false);

    void convertOri2Wnd(double* pX, double* pY);
    void convertOri2Wnd(int* pX, int* pY);

    void convertWnd2Ori(double* pX, double* pY);
    void convertWnd2Ori(int* pX, int* pY);

    void convertOri2WndAll();

    int getLineByName(QString sLineName);

    int getCurveByName(QString sCurveName);

public:

    void setStyle(QColor rulerColor,QColor backColor, QString rulerTitleFontName,int rulerTitleFontSize,int tickMarkFontSize,int tickMarkLineShortLength,int tickMarkLineLongLength);
    bool setHorRuler(QString sHorTitle, int nHorValueDot, double dHorValueStart, double dHorValueEnd, double dHorStepShort, double dHorStepLong);
    bool setVerRuler(QString sVerTitle, int nVerValueDot, double dVerValueStart, double dVerValueEnd, double dVerStepShort, double dVerStepLong);
    void setLineMovable(bool bMovable);

    void setRect(QRect rc);

    void clear();

    bool addCurve(QString sName,QString sCaption,QColor color, int lineWidth, vector<QPointF> &vectPoint,bool bSmooth);

    bool addLine(QString sName,QString sCaption,QColor color, int lineWidth, double xPos, QString sLeftLineName="",QString sRightLineName="");

    void draw(QPainter &p);

    bool OnMousePress(QMouseEvent* e);
    bool OnMouseMove(QMouseEvent* e);
    bool OnMouseRelease(QMouseEvent* e);

    bool PressOnPoint(QMouseEvent* e);

    bool getYByXOnCurve(QString sCurveName, double xValue, double &yValue);

    void show(bool bShow);

    void readChartFromCSV(QString filePath);

    void drawESPoint(QPainter& p);
    bool updateCurrentPoint(QPointF p);
    void drawESCurve(QPainter& p);
};

#endif // ESCHART_H
