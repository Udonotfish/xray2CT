#include "eschart.h"

ESChart::ESChart(QWidget *parent)
    :m_parent(parent)
    ,m_bEnableMoveLine(true)
    ,m_bShow(true)
    ,m_nCurveExpainW(60)
    ,m_rulerColor(QColor(200,200,200))
    ,m_backColor(QColor(80,80,80))
    ,m_sRulerTitleFontName("Arial")
    ,m_nRulerTitleFontSize(11)
    ,m_nTickMarkFontSize(8)
    ,m_nTickMarkLineShortLength(3)
    ,m_nTickMarkLineLongLength(6)
    ,m_nHorValueDot(1)
    ,m_dHorValueStart(0.00)
    ,m_dHorValueEnd(100.00)
    ,m_sHorTitle("Horizontal Information")
    ,m_dHorStepShort(5.00)
    ,m_dHorStepLong(m_dHorStepShort*4)
    ,m_nVerValueDot(1)
    ,m_dVerValueStart(0.00)
    ,m_dVerValueEnd(100.00)
    ,m_sVerTitle("Vertical Information")
    ,m_dVerStepShort(5.00)
    ,m_dVerStepLong(m_dVerStepShort*4)
    ,m_bValid(false)
    ,m_tickTxtWidth(0)
    ,m_tickTxtHeight(0)
    ,m_nPressLineID(-1)
{
    _ESPoint* point = new _ESPoint(QPointF(100,100));
    pointList.push_back(point);
}

ESChart::~ESChart()
{
    clear();
}


void ESChart::calculateRect()
{
    //check member variants valid
    m_bValid = false;

    if(m_nHorValueDot < 0 ||
        abs(m_dHorValueEnd - m_dHorValueStart) < 0.0001 ||
        m_dHorStepShort < 0.0001 ||
        m_dHorStepLong < m_dHorStepShort ||
        m_nVerValueDot < 0 ||
        abs(m_dVerValueEnd - m_dVerValueStart) < 0.0001 ||
        m_dVerStepShort < 0.0001 ||
        m_dVerStepLong < m_dVerStepShort)
    {
        return;
    }
    else
    {
        m_bValid = true;
    }

    int nRulerLineH = m_nTickMarkLineLongLength + 1;

    //the left text or the bottom text size
    int rulerExplainTxtHeight = 0;
    {
        QFont ft(m_sRulerTitleFontName,m_nRulerTitleFontSize);
        QFontMetrics fm(ft);
        rulerExplainTxtHeight = fm.height() + 2;
    }

    //the tick mark text width and height
    m_tickTxtWidth = 0;
    m_tickTxtHeight = 0;
    {
        QFont ft(m_sRulerTitleFontName,m_nTickMarkFontSize);
        QFontMetrics fm(ft);

        QString sTickMarkMax = number2Txt(m_dHorValueEnd, m_nHorValueDot);
        m_tickTxtWidth = fm.width(sTickMarkMax)*1.2;
        m_tickTxtHeight = fm.height() + 2;
    }


    //m_rcMain
    int b = 4;
    m_rcMain.setLeft(m_rc.left() + rulerExplainTxtHeight + m_tickTxtWidth + nRulerLineH + 2*b);
    m_rcMain.setTop(m_rc.top() + m_tickTxtHeight);
    m_rcMain.setRight(m_rc.right() - m_nCurveExpainW - b);
    m_rcMain.setBottom(m_rc.bottom() - rulerExplainTxtHeight - m_tickTxtHeight - nRulerLineH - b);

    {
        //adjust main height
        {
            int nVer = (m_dVerValueEnd - m_dVerValueStart)/m_dVerStepShort;
            int nWndStep = m_rcMain.height() / nVer;
            int nMainHeight = nVer*nWndStep;

            m_rcMain.setTop(m_rcMain.bottom() - nMainHeight);
        }

        //adjust main width
        {
           // qDebug() << m_dHorValueEnd <<" " << m_dHorValueStart <<" " << m_dHorStepShort;
            int nHor = (m_dHorValueEnd - m_dHorValueStart)/m_dHorStepShort;
            int nWndStep = m_rcMain.width() / nHor;
            int nMainWidth = nHor*nWndStep;

            m_rcMain.setRight(m_rcMain.left() + nMainWidth);
        }

    }

    //m_rcHorExplainText
    m_rcHorTitleText.setLeft(m_rcMain.left());
    m_rcHorTitleText.setTop(m_rc.bottom() - rulerExplainTxtHeight);
    m_rcHorTitleText.setRight(m_rcMain.right());
    m_rcHorTitleText.setBottom(m_rc.bottom());

    //m_rcHorRulerText
    m_rcHorRulerText.setLeft(m_rcMain.left() - m_tickTxtHeight);
    m_rcHorRulerText.setTop(m_rc.bottom() - rulerExplainTxtHeight - m_tickTxtHeight);
    m_rcHorRulerText.setRight(m_rcMain.right() + b);
    m_rcHorRulerText.setBottom(m_rc.bottom() - rulerExplainTxtHeight);

    //m_rcHorRulerLine
    m_rcHorRulerLine.setLeft(m_rcMain.left());
    m_rcHorRulerLine.setTop(m_rcMain.bottom() + b);
    m_rcHorRulerLine.setRight(m_rcMain.right());
    m_rcHorRulerLine.setBottom(m_rcHorRulerLine.top() + nRulerLineH);

    //m_rcVerExplainText
    m_rcVerTitleText.setLeft(m_rc.left());
    m_rcVerTitleText.setTop(m_rc.top());
    m_rcVerTitleText.setRight(m_rcVerTitleText.left() + rulerExplainTxtHeight);
    m_rcVerTitleText.setBottom(m_rcMain.bottom());

    //m_rcVerRulerText
    m_rcVerRulerText.setLeft(m_rcVerTitleText.right());
    m_rcVerRulerText.setTop(m_rc.top());
    m_rcVerRulerText.setRight(m_rcVerRulerText.left() + m_tickTxtWidth);
    m_rcVerRulerText.setBottom(m_rcMain.bottom() + b);

    //m_rcVerRulerLine
    m_rcVerRulerLine.setLeft(m_rcMain.left() - b - nRulerLineH);
    m_rcVerRulerLine.setTop(m_rcMain.top());
    m_rcVerRulerLine.setRight(m_rcVerRulerLine.left() + nRulerLineH);
    m_rcVerRulerLine.setBottom(m_rcMain.bottom());

    //m_rcCurveExpain
    m_rcCurveExpain.setLeft(m_rc.right() - m_nCurveExpainW);
    m_rcCurveExpain.setTop(m_rc.top());
    m_rcCurveExpain.setRight(m_rc.right());
    m_rcCurveExpain.setBottom(m_rc.bottom());
}

void ESChart::drawVerText(QPainter &p)
{
    QFont ft(m_sRulerTitleFontName, m_nRulerTitleFontSize);
    ft.setBold(true);
    p.setFont(ft);

    p.rotate(-90.00);
    QFontMetrics fm(ft);
    double fW = fm.width(m_sVerTitle);
    double fH = fm.height();

    int y = (m_rcVerTitleText.top() + m_rcVerTitleText.bottom() - fW)/2;
    int x = (m_rcVerTitleText.left() + m_rcVerTitleText.right() - fH)/2;
    p.drawText(-m_rcVerTitleText.bottom()+y, m_rcVerTitleText.right()-x ,m_sVerTitle);

    p.rotate(90.00);
}

void ESChart::drawVerRuler(QPainter &p)
{
    p.drawLine(QPoint(m_rcVerRulerLine.right(), m_rcVerRulerLine.top()), QPoint(m_rcVerRulerLine.right(), m_rcVerRulerLine.bottom()));
    if(m_dVerStepShort <= 0)
        return;

    int n = (m_dVerValueEnd - m_dVerValueStart)/m_dVerStepShort;
    int nWndStep = m_rcVerRulerLine.height() / n;

    double x1 = m_rcVerRulerLine.right();
    double x0 = x1 - m_nTickMarkLineShortLength;
    double X0 = x1 - m_nTickMarkLineLongLength;

    int k = m_dVerStepLong/m_dVerStepShort;

    for(int i=0;i<=n; i++)
    {
        double y = m_rcVerRulerLine.bottom() - i*nWndStep;
        if(i%k == 0)
        {
            p.drawLine(QPoint(X0,y), QPoint(x1,y));

            //draw tick mark text
            QRect txtRc = {static_cast<int>(X0-200), static_cast<int>(y-m_tickTxtHeight/2), 200-2, m_tickTxtHeight};
            QString sTxtTickMark = number2Txt(m_dVerValueStart + i*m_dVerStepShort, m_nVerValueDot);
            p.drawText(txtRc, Qt::AlignRight|Qt::AlignVCenter, sTxtTickMark);
        }
        else
        {
            p.drawLine(QPoint(x0,y), QPoint(x1,y));
        }
    }
}

void ESChart::drawHorRuler(QPainter &p)
{
    //main line
    p.drawLine(QPoint(m_rcHorRulerLine.left(), m_rcHorRulerLine.top()), QPoint(m_rcHorRulerLine.right(), m_rcHorRulerLine.top()));

    if(m_dHorStepShort <= 0)
        return;

    int n = (m_dHorValueEnd - m_dHorValueStart)/m_dHorStepShort;
    int nWndStep = m_rcHorRulerLine.width() / n;

    double y1 = m_rcHorRulerLine.top();
    double y0 = y1 + m_nTickMarkLineShortLength;
    double Y0 = y1 + m_nTickMarkLineLongLength;

    int k = m_dHorStepLong/m_dHorStepShort;
    for(int i=0;i<=n; i++)
    {
        double x = m_rcHorRulerLine.left() + i*nWndStep;
        if(i%k == 0)
        {
            //draw tick mark line
            p.drawLine(QPoint(x,y1), QPoint(x,Y0));

            //draw tick mark text
            QRect txtRc = {static_cast<int>(x-100), static_cast<int>(Y0+2), 200, m_tickTxtHeight};
            QString sTxtTickMark = number2Txt(m_dHorValueStart + i*m_dHorStepShort, m_nHorValueDot);
            p.drawText(txtRc, Qt::AlignCenter|Qt::AlignVCenter, sTxtTickMark);
        }
        else
        {
            p.drawLine(QPoint(x,y1), QPoint(x,y0));
        }
    }
}

void ESChart::drawExplain(QPainter &p)
{
    QFont ft(m_sRulerTitleFontName, m_nTickMarkFontSize);
    ft.setBold(false);
    p.setFont(ft);

    p.setPen(QPen(m_rulerColor,1));

    int rowSpace = 4;
    int y0 = m_rcMain.top() + rowSpace;
    int x0 = m_rcMain.right() + 8;

    int nRowH = m_tickTxtHeight;
    int h = m_tickTxtHeight/3;
    int w = h;

    size_t nCurve = m_vectCurve.size();
    for(size_t i=0;i<nCurve;i++)
    {
        int y = static_cast<int>(y0+i*(nRowH + rowSpace));

        //draw color rect
        QRect colorRc = {x0,y + (nRowH-h)/2+1, w, h};
        p.fillRect(colorRc, m_vectCurve[i]->color);

        //draw explain text
        QRect txtRc = {colorRc.right()+4, y, 1234, nRowH};
        p.drawText(txtRc, Qt::AlignLeft|Qt::AlignVCenter, m_vectCurve[i]->sCaption);
    }
}

QString ESChart::number2Txt(double value, int nValueDot)
{
    char cc[100] = {0};
    if(nValueDot == 0)
    {
        sprintf(cc, "%d", (int)value);
        return cc;
    }
    else if(nValueDot > 0)
    {
        char cFmt[20] = {0};
        sprintf(cFmt, "%%.%df", nValueDot);

        sprintf(cc, cFmt, value);
        return cc;
    }
}

void ESChart::frameRc(QPainter &p, QRect &rc, QColor clor,bool bFill)
{
    QPen pen(clor, 1);
    pen.setStyle(Qt::SolidLine);
    p.setPen(pen);
    p.drawRect(rc.adjusted(0, 0, -pen.width(), -pen.width()));

    if(bFill)
    {
        p.fillRect(rc, clor);
    }
}

void ESChart::convertOri2Wnd(double *pX, double *pY)
{
    if(pX)
        *pX = (*pX - m_dHorValueStart)*m_rcMain.width()/(m_dHorValueEnd - m_dHorValueStart) + m_rcMain.left();

    if(pY)
        *pY = m_rcMain.bottom() - (*pY - m_dVerValueStart)*m_rcMain.height()/(m_dVerValueEnd - m_dVerValueStart);
}

void ESChart::convertOri2Wnd(int *pX, int *pY)
{
    if(pX)
        *pX = static_cast<int>((*pX - m_dHorValueStart)*m_rcMain.width()/(m_dHorValueEnd - m_dHorValueStart) + m_rcMain.left());

    if(pY)
        *pY = static_cast<int>(m_rcMain.bottom() - (*pY - m_dVerValueStart)*m_rcMain.height()/(m_dVerValueEnd - m_dVerValueStart));
}

void ESChart::convertWnd2Ori(double *pX, double *pY)
{
    if(pX)
        *pX = (*pX - m_rcMain.left()) * (m_dHorValueEnd - m_dHorValueStart) / m_rcMain.width() + m_dHorValueStart;

    if(pY)
        *pY = (m_rcMain.bottom() - *pY) * (m_dVerValueEnd - m_dVerValueStart) / m_rcMain.height() + m_dVerValueStart;
}

void ESChart::convertWnd2Ori(int *pX, int *pY)
{
    if(pX)
        *pX = static_cast<int>((*pX - m_rcMain.left()) * (m_dHorValueEnd - m_dHorValueStart) / m_rcMain.width() + m_dHorValueStart );

    if(pY)
        *pY = static_cast<int>((m_rcMain.bottom() - *pY) * (m_dVerValueEnd - m_dVerValueStart) / m_rcMain.height() + m_dVerValueStart);
}

void ESChart::convertOri2WndAll()
{
    //curves
    {
        size_t n = m_vectCurve.size();
        for(size_t i=0;i<n;i++)
        {
            ItemCurve* t = m_vectCurve[i];
            if(t->rcMain == m_rcMain)
                continue;

            //clear old wnd points
            t->vectWndPoint.clear();

            //calculate new wnd points by ori points and rcMain
            size_t nPt = t->vectOriPoint.size();
            for(size_t j=0; j<nPt; j++)
            {
                double fx = t->vectOriPoint[j].x();
                double fy = t->vectOriPoint[j].y();
                convertOri2Wnd(&fx, &fy);
                t->vectWndPoint.push_back(QPointF(fx, fy));

                //qDebug() << "ox=" << t->vectOriPoint[j].x() << " oy=" << t->vectOriPoint[j].y();
                //qDebug() << "fx=" << fx << " fy=" << fy;
            }

            //save current rcMain
            t->rcMain = m_rcMain;
        }
    }

    //lines:
    {
        size_t n = m_vectLine.size();
        for(size_t i=0;i<n;i++)
        {
            double x = m_vectLine[i]->xOriPos;
            convertOri2Wnd(&x, nullptr);
            m_vectLine[i]->xWndPos = x;

            m_vectLine[i]->wndBtm = m_rcMain.bottom();
            m_vectLine[i]->wndTop = m_rcMain.top() + 1;

            m_vectLine[i]->rcMain = m_rcMain;
        }
    }

}

int ESChart::getLineByName(QString sLineName)
{
    if(sLineName.isEmpty() == true)
        return -1;

    size_t n = m_vectLine.size();
    for(size_t i=0;i<n;i++)
    {
        if(sLineName == m_vectLine[i]->sName)
            return i;
    }

    return -1;
}

int ESChart::getCurveByName(QString sCurveName)
{
    if(sCurveName.isEmpty() == true)
        return -1;

    size_t n = m_vectCurve.size();
    for(size_t i=0;i<n;i++)
    {
        if(sCurveName == m_vectCurve[i]->sName)
            return i;
    }

    return -1;
}

void ESChart::setStyle(QColor rulerColor, QColor backColor, QString rulerTitleFontName, int rulerTitleFontSize, int tickMarkFontSize, int tickMarkLineShortLength, int tickMarkLineLongLength)
{
    m_rulerColor = rulerColor;
    m_backColor = backColor;
    m_sRulerTitleFontName = rulerTitleFontName;
    m_nRulerTitleFontSize = rulerTitleFontSize;
    m_nTickMarkFontSize = tickMarkFontSize;
    m_nTickMarkLineShortLength = tickMarkLineShortLength;
    m_nTickMarkLineLongLength = tickMarkLineLongLength;


}

bool ESChart::setHorRuler(QString sHorTitle, int nHorValueDot, double dHorValueStart, double dHorValueEnd, double dHorStepShort, double dHorStepLong)
{
    m_nHorValueDot = nHorValueDot;
    m_dHorValueStart = dHorValueStart;
    m_dHorValueEnd = dHorValueEnd;
    m_sHorTitle = sHorTitle;
    m_dHorStepShort = dHorStepShort;
    m_dHorStepLong = dHorStepLong;

    if(m_nHorValueDot < 0) return false;
    if(abs(m_dHorValueEnd - m_dHorValueStart) < 0.0001 ) return false;
    if(m_dHorStepShort < 0.0001) return false;
    if(m_dHorStepLong < m_dHorStepShort) return false;

    return true;
}

bool ESChart::setVerRuler(QString sVerTitle, int nVerValueDot, double dVerValueStart, double dVerValueEnd, double dVerStepShort, double dVerStepLong)
{
    m_nVerValueDot = nVerValueDot;
    m_dVerValueStart = dVerValueStart;
    m_dVerValueEnd = dVerValueEnd;
    m_sVerTitle = sVerTitle;
    m_dVerStepShort = dVerStepShort;
    m_dVerStepLong = dVerStepLong;

    if(m_nVerValueDot < 0) return false;
    if(abs(m_dVerValueEnd - m_dVerValueStart) < 0.0001 ) return false;
    if(m_dVerStepShort < 0.0001) return false;
    if(m_dVerStepLong < m_dVerStepShort) return false;

    return true;
}

void ESChart::setLineMovable(bool bMovable)
{
    m_bEnableMoveLine = bMovable;
}

void ESChart::setRect(QRect rc)
{
    m_rc = rc;
    calculateRect();
    convertOri2WndAll();
}

void ESChart::clear()
{
    size_t n = m_vectCurve.size();
    for(size_t i=0;i<n;i++)
        delete m_vectCurve[i];
    m_vectCurve.clear();

    n = m_vectLine.size();
        for(size_t i=0;i<n;i++)
            delete m_vectLine[i];
    m_vectLine.clear();

    if(m_parent)
        m_parent->update();
}

bool ESChart::addCurve(QString sName, QString sCaption, QColor color, int lineWidth, vector<QPointF> &vectPoint,bool bSmooth)
{
    int nID = getCurveByName(sName);
    if(nID >= 0)
    {
        qDebug() << "Curve name already exist!";
        return false;
    }

    ItemCurve* pCurve = new ItemCurve;
    pCurve->sName = sName;
    pCurve->sCaption = sCaption;
    pCurve->color = color;
    pCurve->penWidth = lineWidth;
    pCurve->vectOriPoint = vectPoint;
    pCurve->bSmooth = bSmooth;

    m_vectCurve.push_back(pCurve);

    convertOri2WndAll();

    if(bSmooth)
    {
        pCurve->smooth();

    }

//    ItemPoint* pPoint = new ItemPoint;
//    pPoint->color = QColor(255,0,0);
//    pPoint->index=0;
//    pPoint->paintWidth=1;
//    pPoint->pos = vectPoint[pPoint->index];
//    pPoint->radius = 1;
//    pPoint->draw();

    return true;
}

bool ESChart::addLine(QString sName, QString sCaption, QColor color, int lineWidth, double xPos, QString sLeftLineName, QString sRightLineName)
{
    int nID = getLineByName(sName);
    if(nID >= 0)
    {
        qDebug() << "Line name already exist!";
        return false;
    }

    if(!sLeftLineName.isEmpty() && sLeftLineName == sName)
    {
        qDebug() << "The left line name can not be itself name!";
        return false;
    }

    if(!sRightLineName.isEmpty() && sRightLineName == sName)
    {
        qDebug() << "The right line name can not be itself name!";
        return false;
    }

    if(!sLeftLineName.isEmpty() && !sRightLineName.isEmpty())
    {
        qDebug() << "The right line name can not be as same as the left line name!";
        return false;
    }

    ItemLine* pLine = new ItemLine;
    pLine->sName = sName;
    pLine->sCaption = sCaption;
    pLine->color = color;
    pLine->penWidth = lineWidth;
    pLine->xOriPos = xPos;
    pLine->sLeftLineName = sLeftLineName;
    pLine->sRightLineName = sRightLineName;

    m_vectLine.push_back(pLine);

    convertOri2WndAll();

    return true;
}

void ESChart::draw(QPainter &p)
{
    if(m_bValid == false || m_bShow == false)
        return;

    //p.setRenderHint(QPainter::SmoothPixmapTransform, true);
    //p.setRenderHint(QPainter::TextAntialiasing, true);

    p.setRenderHint(QPainter::Antialiasing, false);

    QPen pen(m_rulerColor, 1);
    p.setPen(pen);

    //draw ruler title text
    {
        QFont ft(m_sRulerTitleFontName, m_nRulerTitleFontSize);
        ft.setBold(true);
        p.setFont(ft);

        //draw ver text
        drawVerText(p);

        //draw hor text
        p.drawText(m_rcHorTitleText ,Qt::AlignVCenter|Qt::AlignCenter, m_sHorTitle);
    }

    //draw ruler tick mark line and text
    {
        QFont ft(m_sRulerTitleFontName, m_nTickMarkFontSize);
        ft.setBold(false);
        p.setFont(ft);

        drawVerRuler(p);
        drawHorRuler(p);
    }

    //main area background
    p.fillRect(m_rcMain, m_backColor);

    p.setRenderHint(QPainter::Antialiasing, true);

    {
        QFont ft(m_sRulerTitleFontName, m_nTickMarkFontSize);
        ft.setBold(false);
        p.setFont(ft);

        //draw curve items
        size_t nCurve = m_vectCurve.size();
        for(size_t i=0;i<nCurve; i++)
            m_vectCurve[i]->draw(p);

        //draw line items
        size_t nLine = m_vectLine.size();
        for(size_t i=0;i<nLine; i++)
            m_vectLine[i]->draw(p);
    }

    //draw explain
    drawExplain(p);
}

bool ESChart::OnMousePress(QMouseEvent *e)
{
    qDebug() << "do mouse press";
    if(m_bValid == false || m_bEnableMoveLine==false)
        return false;

    m_nPressLineID = -1;

    if(m_rcMain.contains(e->pos()) == false)
        return false;

    m_downPoint = e->pos();

    //which line was clicked? I need it's id in m_vectorLine
    size_t n = m_vectLine.size();
    for(size_t i=0;i<n;i++)
    {
        if(m_vectLine[i]->isPointOnMe(e->pos()) == true)
        {
            m_nPressLineID = i;

            if(m_parent)
                m_parent->setCursor(Qt::ClosedHandCursor);

            return true;
        }
    }

    return false;
}

bool ESChart::OnMouseMove(QMouseEvent *e)
{
    qDebug() << "do mouse move";
    if(m_bValid == false || m_bEnableMoveLine==false)
        return false;

    if(m_nPressLineID >= 0)
    {
        ItemLine* pLine = m_vectLine[m_nPressLineID];

        int cx = e->pos().x() - m_downPoint.x();
        int x = pLine->xWndPos + cx;

        //limits for x:

        //don't smaller than rcMain.left
        if(x < m_rcMain.left())
            x = m_rcMain.left();

        //don't bigger than rcMain.right
        if(x > m_rcMain.right())
            x = m_rcMain.right();

        //don't smaller than left limit line
        int nLeftLineID = getLineByName(pLine->sLeftLineName);
        if(nLeftLineID >= 0)
        {
            if(x < m_vectLine[nLeftLineID]->xWndPos + 1)
                x = m_vectLine[nLeftLineID]->xWndPos + 1;
        }

        //don't bigger than right limit line
        int nRightLineID = getLineByName(pLine->sRightLineName);
        if(nRightLineID >= 0)
        {
            if(x > m_vectLine[nRightLineID]->xWndPos - 1)
                x = m_vectLine[nRightLineID]->xWndPos - 1;
        }

        //set the pressed line new position
        pLine->xWndPos = x;

        //calculate ori position
        double fx = x*1.00;
        convertWnd2Ori(&fx, nullptr);
        pLine->xOriPos = fx;

        //update m_downPoint
        m_downPoint = e->pos();
        m_downPoint.setX(x);

        //set cursor
        if(m_parent)
            m_parent->setCursor(Qt::ClosedHandCursor);

        //emit signal
        emit lineMoved(pLine->sName, fx);

        return true;
    }
    else
    {
        //if current mouse is over any line, change cursor
        if(m_parent)
        {
            size_t n = m_vectLine.size();
            for(size_t i=0;i<n;i++)
            {
                if(m_vectLine[i]->isPointOnMe(e->pos()) == true)
                {
                    m_parent->setCursor(Qt::OpenHandCursor);
                    return false;
                }
            }
        }

    }

    if(m_parent)
        m_parent->setCursor(Qt::ArrowCursor);

    return false;
}

bool ESChart::OnMouseRelease(QMouseEvent *e)
{
    qDebug() << "do mouse release";
    if(m_bValid == false || m_bEnableMoveLine==false)
        return false;

    if(m_nPressLineID >= 0)
    {
        if(m_parent)
            m_parent->setCursor(Qt::OpenHandCursor);

        m_nPressLineID = -1;

        return true;
    }
    else
    {
        if(m_parent)
            m_parent->setCursor(Qt::ArrowCursor);
    }

    return false;
}

bool ESChart::PressOnPoint(QMouseEvent *e)
{
    qDebug() << "do Press On Point";
    currentPoint = nullptr;
    foreach(auto& point, pointList)
    {
//        if(point->pos.x() == e->pos().x() && point->pos.y() == e->pos().y())
        if(1)
        {
            currentPoint = point;
            qDebug() << "Press On Point!";
            return true;
        }
    }
    return false;
}

bool ESChart::getYByXOnCurve(QString sCurveName, double xValue, double &yValue)
{
    int idCurve = getCurveByName(sCurveName);
    if(idCurve < 0)
        return false;

    return m_vectCurve[idCurve]->getValueByX(xValue, yValue);
}

void ESChart::show(bool bShow)
{
    m_bShow = bShow;
    if(m_parent)
        m_parent->update();
}

void ESChart::readChartFromCSV(QString filePath)
{
    QFile csv(filePath);
    if(csv.open(QIODevice::ReadOnly|QIODevice::Text))
    {
        qDebug()<< "Read file success";
    }
    else
    {
        qDebug() << "Failed to read csv file";
    }
    QTextStream out(&csv);
    QStringList head = out.readLine().split(",");
    vector<QPointF> HU_mean;
    vector<QPointF> HU_mean_smooth;
    vector<QPointF> HU_std;
    vector<QPointF> HU_std_smooth;
    vector<QPointF> Lower_threshold;
    vector<QPointF> Upper_threshold;
    int lineNum = 0;
    while(!out.atEnd())
    {
        QStringList lines = out.readLine().split(",");
        HU_mean.push_back(QPointF(lineNum, lines.at(0).toDouble()));
        HU_mean_smooth.push_back(QPointF(lineNum, lines.at(1).toDouble()));
        HU_std.push_back(QPointF(lineNum, lines.at(2).toDouble()));
        HU_std_smooth.push_back(QPointF(lineNum, lines.at(3).toDouble()));
        Lower_threshold.push_back(QPointF(lineNum, lines.at(4).toDouble()));
        Upper_threshold.push_back(QPointF(lineNum, lines.at(5).toDouble()));
        lineNum++;
    }
    this->addCurve(head.at(0),head.at(0),QColor(Qt::red),1,HU_mean,0);
    this->addCurve(head.at(1),head.at(1),QColor(Qt::green),1,HU_mean_smooth,0);
    this->addCurve(head.at(2),head.at(2),QColor(Qt::yellow),1,HU_std,0);
    this->addCurve(head.at(3),head.at(3),QColor(Qt::blue),1,HU_std_smooth,0);
    this->addCurve(head.at(4),head.at(4),QColor(Qt::black),5,Lower_threshold,0);
    this->addCurve(head.at(5),head.at(5),QColor(Qt::white),5,Upper_threshold,0);
}

void ESChart::drawESPoint(QPainter& p)
{
    _ESPoint* point = pointList[0];
    QPen pen(point->color,point->paintWidth);
    p.setPen(pen);
    p.drawPoint(point->pos);

}

bool ESChart::updateCurrentPoint(QPointF p)
{
    if(currentPoint)
    {
        currentPoint->pos.setX(p.x());
        currentPoint->pos.setY(p.y());
        return true;
    }
    else return false;
}

void ESChart::drawESLine(QPainter &p)
{
    drawESCurve
}

