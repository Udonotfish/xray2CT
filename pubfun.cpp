#include "pubfun.h"

#include <QDirIterator>

int PubFun::Bezier3DefaultPointNum = 3;
PubFun::PubFun()
{
}
//bool PubFun::ApartRect(FilmFormat &fmt, const QRect &rc, int nHorInterv,int nVerInterv, vector <QRect> &RetImgRc)
//{
//    RetImgRc.clear();

//    if (fmt.sType.compare("STANDARD")==0)
//    {
//        if(fmt.nRow<=0 || fmt.nCol<=0)
//            return false;

//        int nRow = fmt.nRow;
//        int nCol = fmt.nCol;

//        int nUnitWidth = static_cast<int>((   rc.width() - (nCol+1)*nHorInterv   )/nCol   +0.5);//每个格就这么宽
//        int nUnitHeight= static_cast<int>((   rc.height() - (nRow+1)*nVerInterv   )/nRow   +0.5);//每个格就这么高

//        int nNewWidth = nUnitWidth *nCol + nHorInterv*(nCol+1);
//        int nNewHeight= nUnitHeight *nRow + nVerInterv*(nRow+1);

//        //rest length
//        int nColRest = rc.width() - nNewWidth;
//        int nRowRest = rc.height() - nNewHeight;

//        int tID=0;
//        int r,c;
//        int nTop = 0;	int nBtm = 0;	int nLft = 0;	int nRgt = 0;
//        for (r=0;r<nRow;r++)
//        {
//            if(r==0)
//                nTop = rc.top();
//            else
//                nTop = nBtm;

//            nTop += nVerInterv;
//            nBtm = nTop + nUnitHeight;

//            if(r<nRowRest)
//                nBtm +=1;

//            for (c=0;c<nCol;c++)
//            {
//                if(c==0)
//                    nLft = rc.left();
//                else
//                    nLft = nRgt;

//                nLft += nHorInterv;
//                nRgt = nLft + nUnitWidth;

//                if(c<nColRest)
//                    nRgt +=1;

//                QRect trc = {nLft,nTop,nRgt-nLft,nBtm-nTop};
//                RetImgRc.push_back(trc);

//                tID++;
//            }
//        }

//        return true;
//    }
//    if (fmt.sType.compare("COL")==0)
//    {
//        if(fmt.countV.empty()==true)
//            return false;

//        int nCol = int(fmt.countV.size());
//        int nRow=0;
//        int ID=0;

//        int nUnitWidth = static_cast<int>((   rc.width() - (nCol+1)*nHorInterv   )/nCol   +0.5);//每个格就这么宽
//        int nNewWidth = nUnitWidth *nCol + nHorInterv*(nCol+1);


//        int nHRest = rc.width() - nNewWidth;

//        int nLft=0;int nRgt=0;
//        for (int col=0;col<nCol;col++)
//        {
//            if(col==0)
//                nLft = rc.left();
//            else
//                nLft = nRgt;

//            nLft += nHorInterv;
//            nRgt = nLft + nUnitWidth;
//            if(col<nHRest)
//                nRgt++;



//            nRow = fmt.countV.at(col);
//            int nUnitHeight= static_cast<int>((   rc.height() - (nRow+1)*nVerInterv   )/nRow   +0.5);//每个格就这么高
//            int nNewHeight= nUnitHeight *nRow + nVerInterv*(nRow+1);


//            int nVRest = rc.height() - nNewHeight;

//            int nTop=0;int nBtm=0;
//            for(int row=0;row<nRow;row++)
//            {
//                if(row==0)
//                    nTop = rc.top();
//                else
//                    nTop = nBtm;

//                nTop += nVerInterv;
//                nBtm = nTop + nUnitHeight;
//                if(row<nVRest)
//                    nBtm++;

//                QRect trc = {nLft,nTop,nRgt-nLft,nBtm-nTop};


//                RetImgRc.push_back(trc);


//                ID++;
////                if(ID==81)
////                    break;
//            }

////            if(ID==81)
////                break;
//        }



//        return true;
//    }

//    if (fmt.sType.compare("ROW")==0)
//    {
//        if(fmt.countV.empty()==true)
//            return false;

//        int nRow = int(fmt.countV.size());
//        int nCol=0;
//        int ID=0;

//        int nUnitHeight= static_cast<int>((   rc.height() - (nRow+1)*nVerInterv   )/nRow   +0.5);//每个格就这么高
//        int nNewHeight= nUnitHeight *nRow + nVerInterv*(nRow+1);


//        int nVRest = rc.height() - nNewHeight;


//        int nTop=0;int nBtm=0;
//        for (int row=0;row<nRow;row++)
//        {
//            if(row==0)
//                nTop = rc.top();
//            else
//                nTop = nBtm;

//            nTop+=nVerInterv;
//            nBtm = nTop + nUnitHeight;

//            if(row<nVRest)
//                nBtm++;



//            nCol = fmt.countV.at(row);

//            int nUnitWidth = static_cast<int>((   rc.width() - (nCol+1)*nHorInterv   )/nCol   +0.5);//每个格就这么宽
//            int nNewWidth = nUnitWidth *nCol + nHorInterv*(nCol+1);


//            int nHRest = rc.width() - nNewWidth;

//            int nLft=0;int nRgt=0;
//            for(int col=0;col<nCol;col++)
//            {
//                if(col==0)
//                    nLft = rc.left();
//                else
//                    nLft = nRgt;

//                nLft += nHorInterv;
//                nRgt = nLft + nUnitWidth;

//                if(col<nHRest)
//                    nRgt++;


//                QRect trc = {nLft,nTop,nRgt-nLft,nBtm-nTop};
//                RetImgRc.push_back(trc);


//                ID++;
////                if(ID==81)
////                    break;
//            }
////            if(ID==81)
////                break;
//        }

//        return true;
//    }

//    return false;
//}

//bool PubFun::ApartStandardRect(int nRow, int nCol, const QRect &rc, int nHorInterv,int nVerInterv, vector<QRect> &RetImgRc)
//{
//    FilmFormat fmt;
//    fmt.nCol = nCol;
//    fmt.nRow = nRow;
//    fmt.sType = "STANDARD";

//    return ApartRect(fmt, rc, nHorInterv, nVerInterv, RetImgRc);
//}

//bool PubFun::ApartStandardRect2D(int nRow, int nCol, const QRect &rc, int nInterv, vector<vector<QRect> > &RetImgRc)
//{
//    FilmFormat fmt;
//    fmt.nCol = nCol;
//    fmt.nRow = nRow;
//    fmt.sType = "STANDARD";

//    vector<QRect> allImgRc;
//    if(false == ApartRect(fmt, rc, nInterv, nInterv, allImgRc))
//        return false;

//    size_t n = allImgRc.size();
//    size_t nRowID = 0;
//    for (size_t i=0;i<n;i++)
//    {
//        if(i%nCol == 0)
//        {
//            nRowID++;
//            vector<QRect> vct;
//            RetImgRc.push_back(vct);
//        }

//        RetImgRc[nRowID-1].push_back(allImgRc[i]);
//    }

//    return true;
//}

bool PubFun::ApartStandardRectByWH2D(int nCellWidth, int nCellHeight, const QRect &rc, int nHorInterv, int nVerInterv, vector<vector<QRect> > &RetImgRc)
{
    if(nCellWidth > rc.width() || nCellHeight > rc.height())
        return false;

    int w = nHorInterv + nCellWidth;
    int h = nVerInterv + nCellHeight;

    int top = rc.top();
    while(1)
    {
        int btm = top + h;

        if(btm-nVerInterv > rc.bottom())
            return true;

        int lft = rc.left();
        vector<QRect> vectRow;
        while(1)
        {
            int rgt = lft + w;

            if(rgt-nHorInterv > rc.right())
                break;

            vectRow.push_back(QRect(lft+nHorInterv, top+nVerInterv, nCellWidth, nCellHeight));
            lft += w;
        }

        RetImgRc.push_back(vectRow);
        top += h;
    }

}

void** PubFun::ArrayNew(unsigned long nRow, unsigned long nCol, unsigned long nSize,bool bFill4Byte, unsigned long *nNewRow, unsigned long* nNewCol)
{
    if (nRow==0 || nCol==0 || nSize==0)
        return nullptr;

    if(bFill4Byte == true)
        nCol = ((nCol*8)+31)/32*4;

    unsigned long nDataLen = nRow*nCol*nSize;
    uchar* pData = new uchar[nDataLen];
    memset(pData,0,nDataLen);

    uchar** pAdd = new uchar*[nRow];
    for(size_t i=0;i<nRow;i++)
        pAdd[i] = static_cast<uchar*>(&(pData[i*nCol*nSize]));

    if (nNewRow) *nNewRow = nRow;
    if (nNewCol) *nNewCol = nCol;

    return (void**)pAdd;

}

void PubFun::ArrayFree(void **pArr,int iflag)
{
    //if(iflag)
    //	AtlTrace("\r\n---ArrFree:%x  %d",LONG(pArr),iflag);

    if(pArr==nullptr)
        return;

    uchar *pData = (uchar*)(pArr[0]);
    delete[]pData;

    uchar **pAdd = (uchar**)pArr;
    delete []pAdd;
}

void** PubFun::ArrayCopy(void** pSrcArr,unsigned long nSrcRow, unsigned long nSrcCol, unsigned long nSrcSize)
{
    if (nSrcRow==0 || nSrcCol==0 || nSrcSize==0 || pSrcArr==nullptr)
        return nullptr;

    //nSrcCol = ((nSrcCol*8)+31)/32*4;

    unsigned long nDataLen = nSrcRow*nSrcCol*nSrcSize;
    uchar* pDstData = new uchar[nDataLen];
    memcpy(pDstData,pSrcArr[0],nDataLen);

    uchar** pAdd = new uchar*[nSrcRow];
    for(size_t i=0;i<nSrcRow;i++)
        pAdd[i] = (uchar*)(&(pDstData[i*nSrcCol*nSrcSize]));

    return (void**)pAdd;
}

QRect PubFun::GetShowRcByImgSize(QRect rc, double ImgWidth, double ImgHeight)
{
    QRect reRC = {0,0,0,0};

    if (rc.right() == rc.left())
        return reRC;


    if ((rc.bottom()-rc.top())*ImgWidth >= ImgHeight*(rc.right()-rc.left()))//<=(cy/cx)>=(ImgHeight/ImgWidth)
    {// hor
        double fNewH=0;
        reRC.setLeft(rc.left());
        reRC.setRight(rc.right());
        fNewH = ( ImgHeight * (rc.right()-rc.left()) )/ImgWidth;
        reRC.setTop( int((rc.bottom() + rc.top()-fNewH)/2) );
        reRC.setBottom( reRC.top()+(long)(fNewH+0.5) );
    }
    else
    {//ver
        double fNewW=0;
        reRC.setTop(rc.top());
        reRC.setBottom( rc.bottom() );
        fNewW = ( (rc.bottom()-rc.top())*ImgWidth)/ImgHeight;
        reRC.setLeft(int((rc.right()+rc.left()-fNewW)/2));
        reRC.setRight(reRC.left()+(long)(fNewW+0.5));
    }

    return reRC;
}

QRectF PubFun::GetShowRcByImgSizeF(QRect rc, double ImgWidth, double ImgHeight)
{
    QRectF reRC = {0.00, 0.00, 0.00, 0.00};

    if (rc.right() == rc.left())
        return reRC;


    if ((rc.bottom()-rc.top())*ImgWidth >= ImgHeight*(rc.right()-rc.left()))//<=(cy/cx)>=(ImgHeight/ImgWidth)
    {// hor
        double fNewH=0;
        reRC.setLeft(rc.left());
        reRC.setRight(rc.right());
        fNewH = ( ImgHeight * (rc.right()-rc.left()) )/ImgWidth;
        reRC.setTop( int((rc.bottom() + rc.top()-fNewH)/2) );
        reRC.setBottom( reRC.top()+(long)(fNewH+0.5) );
    }
    else
    {//ver
        double fNewW=0;
        reRC.setTop(rc.top());
        reRC.setBottom( rc.bottom() );
        fNewW = ( (rc.bottom()-rc.top())*ImgWidth)/ImgHeight;
        reRC.setLeft(int((rc.right()+rc.left()-fNewW)/2));
        reRC.setRight(reRC.left()+(long)(fNewW+0.5));
    }

    return reRC;
}

bool PubFun::tool_IsPtOnLine( QPoint DstPt,const QPoint &pt1,const QPoint &pt2 )
{
    int m = 4;

    QPoint pt[5];
    if(pt2.x() == pt1.x())
    {
        QRect rc = {pt1.x()-m/2, min(pt1.y(), pt2.y()), m, abs(pt2.y() - pt1.y())};
        return rc.contains(DstPt);
    }
    else if(pt2.y() == pt1.y())
    {
        QRect rc = {min(pt1.x(), pt2.x()), pt1.y()-m/2, abs(pt2.x() - pt1.x()), m};
        return rc.contains(DstPt);
    }
    else
    {
        double k = static_cast<double>((pt2.y() - pt1.y()))/(pt2.x() - pt1.x());
        if(k < 0.001)//clock wise to add point
        {
            int t = static_cast<int>(pt1.x() - m);
            pt[0].setX(t);
            pt[4].setX(t);

            t = static_cast<int>(pt1.y() - m);
            pt[0].setY(t);
            pt[4].setY(t);

            pt[1].setX(static_cast<int>(pt2.x() - m));
            pt[1].setY(static_cast<int>(pt2.y() - m));
            pt[2].setX(static_cast<int>(pt2.x() + m));
            pt[2].setY(static_cast<int>(pt2.y() + m));
            pt[3].setX(static_cast<int>(pt1.x() + m));
            pt[3].setY(static_cast<int>(pt1.y() + m));
        }
        else
        {
            int t = static_cast<int>(pt1.x() + m);
            pt[0].setX(t);
            pt[4].setX(t);

            t = static_cast<int>(pt1.y() - m);
            pt[0].setY(t);
            pt[4].setY(t);

            pt[1].setX(static_cast<int>(pt2.x() + m));
            pt[1].setY(static_cast<int>(pt2.y() - m));
            pt[2].setX(static_cast<int>(pt2.x() - m));
            pt[2].setY(static_cast<int>(pt2.y() + m));
            pt[3].setX(static_cast<int>(pt1.x() - m));
            pt[3].setY(static_cast<int>(pt1.y() + m));
        }
    }

    QPolygon rgn;
    for(int i=0;i<5;i++)
        rgn.setPoint(i,pt[i].x(), pt[i].y());

    bool b = rgn.containsPoint(DstPt,Qt::FillRule::WindingFill);

    return b;
}

bool PubFun::tool_IsPtOnLineF(QPoint DstPt, const QPointF &pt1, const QPointF &pt2)
{
    double m = 4;

    QPoint pt[5];
    if(abs(pt2.x() - pt1.x()) < 0.001 )
    {
        QRectF rc = {pt1.x()-m/2, min(pt1.y(), pt2.y()), m, abs(pt2.y() - pt1.y())};
        return rc.contains(DstPt);
    }
    else if(abs(pt2.y() - pt1.y()) < 0.001 )
    {
        QRectF rc = {min(pt1.x(), pt2.x()), pt1.y()-m/2, abs(pt2.x() - pt1.x()), m};
        return rc.contains(DstPt);
    }
    else
    {
        double k = static_cast<double>((pt2.y() - pt1.y()))/(pt2.x() - pt1.x());
        if(k < 0.001)//clock wise to add point
        {
            int t = static_cast<int>(pt1.x() - m);
            pt[0].setX(t);
            pt[4].setX(t);

            t = static_cast<int>(pt1.y() - m);
            pt[0].setY(t);
            pt[4].setY(t);

            pt[1].setX(static_cast<int>(pt2.x() - m));
            pt[1].setY(static_cast<int>(pt2.y() - m));
            pt[2].setX(static_cast<int>(pt2.x() + m));
            pt[2].setY(static_cast<int>(pt2.y() + m));
            pt[3].setX(static_cast<int>(pt1.x() + m));
            pt[3].setY(static_cast<int>(pt1.y() + m));
        }
        else
        {
            int t = static_cast<int>(pt1.x() + m);
            pt[0].setX(t);
            pt[4].setX(t);

            t = static_cast<int>(pt1.y() - m);
            pt[0].setY(t);
            pt[4].setY(t);

            pt[1].setX(static_cast<int>(pt2.x() + m));
            pt[1].setY(static_cast<int>(pt2.y() - m));
            pt[2].setX(static_cast<int>(pt2.x() - m));
            pt[2].setY(static_cast<int>(pt2.y() + m));
            pt[3].setX(static_cast<int>(pt1.x() - m));
            pt[3].setY(static_cast<int>(pt1.y() + m));
        }

    }


    QPolygon rgn;
    for(int i=0;i<5;i++)
    {
        int p[2] = {pt[i].x() , pt[i].y() };
        rgn.putPoints(i,1,p);
    }

    bool b = rgn.containsPoint(DstPt,Qt::FillRule::WindingFill);

    return b;
}

bool PubFun::tool_GetLineAcrossRc( QRect rc,QPoint p1,QPoint p2,QPoint &retP1,QPoint &retP2 )
{//这个函数记得完善一下，retPt1应该是靠近p1的，retPt2应该是靠近p2的
    //判断这两点在同一条边上
    if(p1.y() == rc.top() && p1.y() == p2.y()) return false;
    if(p1.y() == rc.bottom() && p1.y() == p2.y()) return false;
    if(p1.x() == rc.left() && p1.x() == p2.x()) return false;
    if(p1.x() == rc.right() && p1.x() == p2.x()) return false;

    do
    {
        if (p1.y() == p2.y())//hor
        {

            retP1.setX(rc.left());
            retP1.setY(p1.y());

            retP2.setX(rc.right());
            retP2.setY(p1.y());
        }
        else if (p1.x() == p2.x())//ver
        {
            retP1.setX(p1.x());
            retP1.setY(rc.top());

            retP2.setX(p1.x());
            retP2.setY(rc.bottom());
        }
        else
        {
            long x1 = p1.x(); long y1 = p1.y();
            long x2 = p2.x(); long y2 = p2.y();

            bool b1 = false;//第一个交点找到没

            //1.求与top的交点,已知y = rc.top;
            if (1)
            {
                long y = rc.top();
                long x =  (long)( (x2 - x1) * (y - y1) * 1.00 / (y2 - y1) + x1 );
                if(x >= rc.left() && x <= rc.right())
                {
                    retP1.setX(x);
                    retP1.setY(y);
                    b1 = true;
                }
            }

            //2.求与bottom的交点,已知y = rc.bottom;
            if (2)
            {
                long y = rc.bottom();
                long x =  (long)( (x2 - x1) * (y - y1) * 1.00 / (y2 - y1) + x1 );
                if(x>=rc.left() && x<=rc.right())//有交点了
                {
                    if (b1 == false)//赋值要讲究一下
                    {
                        retP1.setX(x);
                        retP1.setY(y);

                        b1 = true;
                    }
                    else
                    {
                        retP2.setX(x);
                        retP2.setY(y);

                        break;
                    }

                }
            }

            //3.求与left的交点,已知x = rc.left;
            if (3)
            {
                long x = rc.left();
                long y = (long)( (x - x1) * (y2 - y1)* 1.00 / (x2 - x1) + y1 );

                if(y>=rc.top() && y<=rc.bottom())//有交点了
                {
                    if (b1 == false)//赋值要讲究一下
                    {
                        retP1.setX(x);
                        retP1.setY(y);

                        b1 = true;
                    }
                    else
                    {
                        retP2.setX(x);
                        retP2.setY(y);

                        break;
                    }

                }
            }

            //4.求与left的交点,已知x = rc.right;
            if (4)
            {
                long x = rc.right();
                long y = (long)( (x - x1) * (y2 - y1)* 1.00 / (x2 - x1) + y1 );

                if(y>=rc.top() && y<=rc.bottom())//有交点了
                {
                    if (b1 == false)//赋值要讲究一下
                    {
                        return false;//到最后这里了，这才找到一个交点--实际上这是不可能的
                    }
                    else
                    {
                        retP2.setX(x);
                        retP2.setY(y);

                        break;
                    }

                }
            }
        }

    } while (false);

    int R1ToP1 = (retP1.x() - p1.x())*(retP1.x() - p1.x()) + (retP1.y() - p1.y())*(retP1.y() - p1.y());//RetP1到p1的距离的平方
    int R2ToP1 = (retP2.x() - p1.x())*(retP2.x() - p1.x()) + (retP2.y() - p1.y())*(retP2.y() - p1.y());//RetP2到p1的距离的平方

    if (R1ToP1 > R2ToP1)//应该是(RetP1到p1的距离)小于(RetP2到p1的距离),若不是的话，互换
        swap(retP1,retP2);

    return true;
}

bool PubFun::getNormalLine(const vector<QPointF> &vectCenterPoint, const vector<double> &vectRadius, vector<QPointF> &vectContours)
{
    size_t n = vectCenterPoint.size();
    Q_ASSERT(n == vectRadius.size());
    if(n <= 1)
        return false;

    vector<QPointF> vectRight;
    XSPoint2 <double>p0;
    XSPoint2 <double>p1;
    for(size_t i=0; i<n; i++)
    {
        p0.x = vectCenterPoint[i].x();
        p0.y = vectCenterPoint[i].y();


        if(i == n-1)
        {
            p1.x = vectCenterPoint[i-1].x();
            p1.y = vectCenterPoint[i-1].y();
        }
        else
        {
            p1.x = vectCenterPoint[i+1].x();
            p1.y = vectCenterPoint[i+1].y();
        }

        XSPoint2 <double>ptLeft =  p0.NormalVectorLeft(p1, vectRadius[i]);
        vectContours.push_back(QPointF(ptLeft.X(), ptLeft.Y()));

        XSPoint2 <double>ptRight =  p0.NormalVectorRight(p1, vectRadius[i]);
        vectRight.insert(vectRight.begin(), QPointF(ptRight.X(), ptRight.Y()));
    }

    vectContours.insert(vectContours.end(), vectRight.begin(), vectRight.end() );

    return true;
}

//count为插入点数, outPoints为输出点集合，长度为count + 2(含首尾)
void PubFun::bezier_parseBezier(const Ogre::Vector2& start, const Ogre::Vector2& end,  const Ogre::Vector2& control1, const Ogre::Vector2& control2, int count, std::vector<Ogre::Vector2>& outPoints)
{
    if(count < 0)
        count = Bezier3DefaultPointNum;

    outPoints.push_back(start);
    for(int i = 1; i<=count; i++)
    {
        double st = (double) i/(count+1);
        double dt = (1-st);

        //二次项
        double st2 = st * st;
        double dt2 = dt * dt;

        double t0 = dt * dt2;
        double t1 = dt2 * st * 3;
        double t2 = dt * st2 * 3;
        double t3 = st * st2;

        outPoints.push_back(start * t0 + control1 * t1 + control2 * t2 + end * t3);
    }

    outPoints.push_back(end);
}

//根据当前点pt,前一点p1, 后一点p2计算当前点对应的控制点control1 control2
void PubFun::bezier_getControlPoint(const Ogre::Vector2& pt, const Ogre::Vector2& p1, const Ogre::Vector2& p2, Ogre::Vector2& control1, Ogre::Vector2& control2, double ratio)
{
    double length1 = (p1 - pt).length();
    double length2 = (p2 - pt).length();

    double v = length2/(length1+ 0.000001);

    Ogre::Vector2 delta;
    if(v>1)
    {
        delta = p1 - (pt + ( p2 - pt) / (v + 0.000001));
    }
    else
    {
        delta = pt + (p1-pt) * v - p2 ;
    }

    delta *= ratio;
    control1 = pt + delta;
    control2 = pt - delta;
}

void PubFun::bezier_parsePolyline(const std::vector<Ogre::Vector2>& inPoints, int count, std::vector<Ogre::Vector2>& outPoints, double ratio,vector<int> &mapInToOut)
{
    std::vector<Ogre::Vector2>::size_type pointsSize = inPoints.size();
    if(pointsSize < 3 )//插值至少需要三点
    {
        for(int i = 0; i<pointsSize; i++)
        {
            mapInToOut.push_back(i);//add by wxs
            outPoints.push_back(inPoints[i]);
        }
    }
    else
    {
        Ogre::Vector2  control1, control2;    //顶点对应的前后控制点

        bezier_getControlPoint(inPoints[1], inPoints[0], inPoints[2], control1, control2, ratio); //首端
        mapInToOut.push_back(outPoints.size());//add by wxs
        bezier_1_parseBezier(inPoints[0], inPoints[1], inPoints[0], control1, count, outPoints);

        for(int i = 2; i<pointsSize -1; i++) //根据中间各点生成与前一点连线
        {
            Ogre::Vector2 lastControl = control2;
            bezier_getControlPoint(inPoints[i], inPoints[i-1], inPoints[i+1], control1, control2, ratio);
            mapInToOut.push_back(outPoints.size());//add by wxs
            bezier_1_parseBezier(inPoints[i - 1], inPoints[i], lastControl, control1, count, outPoints);
        }

        mapInToOut.push_back(outPoints.size());//add by wxs
        bezier_1_parseBezier(inPoints[pointsSize-2], inPoints[pointsSize-1], control2, inPoints[pointsSize-1], count, outPoints);

        mapInToOut.push_back(outPoints.size());//add by wxs
        outPoints.push_back(inPoints[pointsSize-1]);
    }
}

void PubFun::bezier_1_parseBezier(const Ogre::Vector2& start, const Ogre::Vector2& end,  const Ogre::Vector2& control1, const Ogre::Vector2& control2, int count, std::vector<Ogre::Vector2>& outPoints)
{
    if(count < 0)
        count = Bezier3DefaultPointNum;

    outPoints.push_back(start);
    for(int i = 1; i<=count; i++)
    {
        double st = (double) i/(count+1);
        double dt = (1-st);

        //二次项
        double st2 = st * st;
        double dt2 = dt * dt;

        double t0 = dt * dt2;
        double t1 = dt2 * st * 3;
        double t2 = dt * st2 * 3;
        double t3 = st * st2;

        outPoints.push_back(start * t0 + control1 * t1 + control2 * t2 + end * t3);
    }
}

void PubFun::bezier_parsePolygon(const std::vector<Ogre::Vector2>& inPoints, int count, std::vector<Ogre::Vector2>& outPoints, double ratio,vector<int> &mapInToOut)
{
    std::vector<Ogre::Vector2>::size_type pointsSize = inPoints.size();
    if(pointsSize < 3 )//插值至少需要三点
    {
        for(int i = 0; i<pointsSize; i++)
        {
            mapInToOut.push_back(i);//add by wxs
            outPoints.push_back(inPoints[i]);
        }
    }
    else
    {
        Ogre::Vector2  control1, control2;    //顶点对应的前后控制点
        Ogre::Vector2 firstControl;
        Ogre::Vector2 lastControl;

        //首尾
        bezier_getControlPoint(inPoints[pointsSize-1], inPoints[pointsSize-2], inPoints[0], firstControl, lastControl, ratio);
        bezier_getControlPoint(inPoints[0], inPoints[pointsSize-1], inPoints[1], control1, control2, ratio);
        //mapInToOut.push_back(outPoints.size());//add by wxs
        bezier_1_parseBezier(inPoints[pointsSize-1], inPoints[0], lastControl, control1, count, outPoints);

        for(int i = 1; i<pointsSize-1; i++) //根据中间各点，生成与前一点连线
        {
            lastControl = control2;
            bezier_getControlPoint(inPoints[i], inPoints[i-1], inPoints[i+1], control1, control2, ratio);
            mapInToOut.push_back(outPoints.size());//add by wxs
            bezier_1_parseBezier(inPoints[i-1], inPoints[i], lastControl, control1, count, outPoints);

        }

        mapInToOut.push_back(outPoints.size());//add by wxs
        bezier_parseBezier(inPoints[pointsSize-2], inPoints[pointsSize-1], control2, firstControl, count, outPoints); //末端
        outPoints.pop_back();

        mapInToOut.push_back(0);//add by wxs
    }
}

double PubFun::distanceFromPointToLine(QPointF &p, QPointF &p1, QPointF &p2)
{
    return abs( ((p.x()-p1.x())*(p2.y()-p1.y()) - (p2.x()-p1.x())*(p.y()-p1.y()))/sqrt((p2.x()-p1.x())*(p2.x()-p1.x())+(p2.y()-p1.y())*(p2.y()-p1.y())) );
}

double PubFun::distanceFromPointToLine(Vector2 &p, Vector2 &p1, Vector2 &p2)
{
    return abs( ((p.x-p1.x)*(p2.y-p1.y) - (p2.x-p1.x)*(p.y-p1.y))/sqrt((p2.x-p1.x)*(p2.x-p1.x)+(p2.y-p1.y)*(p2.y-p1.y)) );
}

int PubFun::thinPolyLineByDistance(QList<QPointF> &srcPolyLine, double fThreshold)
{
    int n = srcPolyLine.size();
    if(n < 4)
        return 0;

    QList<QPointF> &a = srcPolyLine;
    QList<QPointF> b;// = outPolyLine;

    b.push_back(a[0]);
    for(int i=1;i<n-1;i++)
    {
        //double d = distanceFromPointToLine(a[i], a[i-1], a[i+1]);
        double d = distanceFromPointToLine(a[i], b[b.size()-1], a[i+1]);
        if(d > fThreshold)
            b.push_back(a[i]);
    }

    //save last point
    b.push_back(a[n-1]);

    srcPolyLine.clear();
    srcPolyLine = b;
    return 0;
}

int PubFun::thinPolyLineByDistance(vector<QPointF> &srcPolyLine, double fThreshold)
{
    int n = srcPolyLine.size();
    if(n < 4)
    {
        return 0;
    }

    vector<QPointF> &a = srcPolyLine;
    vector<QPointF> b;

    b.push_back(a[0]);
    for(int i=1;i<n-1;i++)
    {
        //double d = distanceFromPointToLine(a[i], a[i-1], a[i+1]);
        double d = distanceFromPointToLine(a[i], b[b.size()-1], a[i+1]);
        if(d > fThreshold)
            b.push_back(a[i]);
    }

    //save last point
    b.push_back(a[n-1]);

    srcPolyLine.clear();
    srcPolyLine = b;
    return 0;
}

int PubFun::thinPolyLineByDistance(vector<Vector2> &srcPolyLine, double fThreshold)
{
    int n = srcPolyLine.size();
    if(n < 4)
    {
        return 0;
    }

    vector<Vector2> &a = srcPolyLine;
    vector<Vector2> b;// = outPolyLine;

    b.push_back(a[0]);
    for(int i=1;i<n-1;i++)
    {
        //double d = distanceFromPointToLine(a[i], a[i-1], a[i+1]);
        double d = distanceFromPointToLine(a[i], b[b.size()-1], a[i+1]);
        if(d > fThreshold)
            b.push_back(a[i]);
    }

    //save last point
    b.push_back(a[n-1]);

    srcPolyLine.clear();
    srcPolyLine = b;
    return 0;
}

double PubFun::getAngleB(const QPointF &A, const QPointF &B, const QPointF &C)
{
    double fAB = pow(( pow(B.x() - A.x(),2.00) + pow(B.y() - A.y(),2.00) ), 0.5);
    double fCB = pow(( pow(C.x() - B.x(),2.00) + pow(C.y() - B.y(),2.00) ), 0.5);

    double fCosB = ( (A.x()-B.x())*(C.x()-B.x()) + (A.y()-B.y())*(C.y()-B.y()) )/(fAB * fCB);
    double fAngleB = acos(fCosB) * 180 /3.1415926;

    return fAngleB;
}

double PubFun::getAngleB(const Vector2 &A, const Vector2 &B, const Vector2 &C)
{
    double fAB = pow(( pow(B.x - A.x,2.00) + pow(B.y - A.y,2.00) ), 0.5);
    double fCB = pow(( pow(C.x - B.x,2.00) + pow(C.y - B.y,2.00) ), 0.5);

    double fCosB = ( (A.x-B.x)*(C.x-B.x) + (A.y-B.y)*(C.y-B.y) )/(fAB * fCB);
    double fAngleB = acos(fCosB) * 180 /3.1415926;

    return fAngleB;
}

int PubFun::thinPolyLineByAngle(vector<QPointF> &srcPolyLine, double fThreshold)
{
    int n = srcPolyLine.size();
    if(n < 4)
    {
        return 0;
    }

    vector<QPointF> &a = srcPolyLine;
    vector<QPointF> b;

    b.push_back(a[0]);
    for(int i=1;i<n-1;i++)
    {
        double d = getAngleB(b[b.size()-1], a[i],  a[i+1]);
        if(d < fThreshold)
            b.push_back(a[i]);
    }

    //save last point
    b.push_back(a[n-1]);

    srcPolyLine.clear();
    srcPolyLine = b;

    return 0;
}

int PubFun::thinPolyLineByAngle(QList<QPointF> &srcPolyLine, double fThreshold)
{
    int n = srcPolyLine.size();
    if(n < 4)
    {
        return 0;
    }

    QList<QPointF> &a = srcPolyLine;
    QList<QPointF> b;

    b.push_back(a[0]);
    for(int i=1;i<n-1;i++)
    {
        double d = getAngleB(b[b.size()-1], a[i],  a[i+1]);
        if(d < fThreshold)
            b.push_back(a[i]);
    }

    //save last point
    b.push_back(a[n-1]);

    srcPolyLine.clear();
    srcPolyLine = b;

    return 0;
}

int PubFun::thinPolyLineByAngle(vector<Vector2> &srcPolyLine, double fThreshold)
{
    int n = srcPolyLine.size();
    if(n < 4)
    {
        return 0;
    }

    vector<Vector2> &a = srcPolyLine;
    vector<Vector2> b;

    b.push_back(a[0]);
    for(int i=1;i<n-1;i++)
    {
        double d = getAngleB(b[b.size()-1], a[i],  a[i+1]);
        if(d < fThreshold)
            b.push_back(a[i]);
    }

    //save last point
    b.push_back(a[n-1]);

    srcPolyLine.clear();
    srcPolyLine = b;

    return 0;
}

bool PubFun::isSamePoint(QPointF &a, QPointF &b)
{
    if(pow((a.x()-b.x()),2.00) + pow((a.y()-b.y()),2.00) < 1)
        return true;

    return false;
}

bool PubFun::SmoothPolyLine2(QList<QPointF> &lst, double fAngle, double fStep)
{
    int n = lst.size();
    if(n < 3)
        return false;

    for(int i=1; i<lst.size()-1;i++)
    {
        QPointF &A = lst[i-1];
        QPointF &B = lst[i];
        QPointF &C = lst[i+1];

        if(isSamePoint(A,B))
        {
            lst.erase(lst.begin()+i);
            i--;
            continue;
        }

        if(isSamePoint(B,C))
        {
            lst.erase(lst.begin()+i+1);
            i--;
            continue;
        }

        if(getAngleB(A, B, C) > fAngle)
            continue;

        QPointF newPt0((A.x()+B.x())*fStep, (A.y()+B.y())*fStep);
        QPointF newPt1((C.x()+B.x())*fStep, (C.y()+B.y())*fStep);

        lst.erase(lst.begin()+i);
        lst.insert(lst.begin()+i+1,newPt1);
        lst.insert(lst.begin()+i+1,newPt0);
        i--;
    }

    return false;
}

bool PubFun::SmoothPolyLine2(vector<QPointF> &lst, double fAngle, double fStep)
{
    int n = lst.size();
    if(n < 3)
        return false;

    for(int i=1; i<lst.size()-1;i++)
    {
        QPointF &A = lst[i-1];
        QPointF &B = lst[i];
        QPointF &C = lst[i+1];

        if(isSamePoint(A,B))
        {
            lst.erase(lst.begin()+i);
            i--;
            continue;
        }

        if(isSamePoint(B,C))
        {
            lst.erase(lst.begin()+i+1);
            i--;
            continue;
        }

        if(getAngleB(A, B, C) > fAngle)
            continue;

        QPointF newPt0((A.x()+B.x())*fStep, (A.y()+B.y())*fStep);
        QPointF newPt1((C.x()+B.x())*fStep, (C.y()+B.y())*fStep);

        lst.erase(lst.begin()+i);
        lst.insert(lst.begin()+i+1,newPt1);
        lst.insert(lst.begin()+i+1,newPt0);
        i--;
    }

    return false;
}

double PubFun::PointToSegDistance(double x, double y, double x1, double y1, double x2, double y2)
{
    double cross = (x2 - x1) * (x - x1) + (y2 - y1) * (y - y1);
    if (cross <= 0)
    {
        qDebug() << "tp1";
        return sqrt((x - x1) * (x - x1) + (y - y1) * (y - y1));
    }

    double d2 = (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1);
    if (cross >= d2)
    {
        qDebug() << "tp2";
        return sqrt((x - x2) * (x - x2) + (y - y2) * (y - y2));
    }

    double r = cross / d2;
    double px = x1 + (x2 - x1) * r;
    double py = y1 + (y2 - y1) * r;
    return sqrt((x - px) * (x - px) + (py - y) * (py - y));
}

double PubFun::PointToSegDistance(double x, double y, double x1, double y1, double x2, double y2, QPointF &closestPt)
{
    XSPoint2<double> s0(x1, y1), s1(x2, y2), p(x, y), closest;
    double r = SegmentPointDistance<double>(s0, s1, p, closest);
    closestPt.setX(closest.x);
    closestPt.setY(closest.y);

    return r;
}

#ifdef USE_OPENCV
bool PubFun::cvline_findNextPoint(vector<Point> &_neighbor_points, Mat &_image, Point _inpoint, int flag, Point& _outpoint, int &_outflag)
{
    int i = flag;
    int count = 1;
    bool success = false;

    while (count <= 7)
    {
        Point tmppoint = _inpoint + _neighbor_points[i];
        if (tmppoint.x > 0 && tmppoint.y > 0 && tmppoint.x < _image.cols&&tmppoint.y < _image.rows)
        {
            if (_image.at<uchar>(tmppoint) == 255)
            {
                _outpoint = tmppoint;
                _outflag = i;
                success = true;
                _image.at<uchar>(tmppoint) = 0;
                break;
            }
        }
        if (count % 2)
        {
            i += count;
            if (i > 7)
            {
                i -= 8;
            }
        }
        else
        {
            i += -count;
            if (i < 0)
            {
                i += 8;
            }
        }
        count++;
    }
    return success;
}

bool PubFun::cvline_findFirstPoint(Mat &_inputimg, Point &_outputpoint)
{
    bool success = false;
    for (int i = 0; i < _inputimg.rows; i++)
    {
        uchar* data = _inputimg.ptr<uchar>(i);
        for (int j = 0; j < _inputimg.cols; j++)
        {
            if (data[j] == 255)
            {
                success = true;
                _outputpoint.x = j;
                _outputpoint.y = i;
                data[j] = 0;
                break;
            }
        }
        if (success)
            break;
    }
    return success;
}

void PubFun::cvline_findLines(Mat &_inputimg, vector<deque<Point>> &_outputlines)
{
    vector<Point> neighbor_points = { Point(-1,-1),Point(0,-1),Point(1,-1),Point(1,0),Point(1,1),Point(0,1),Point(-1,1),Point(-1,0) };
    Point first_point;
    while (cvline_findFirstPoint(_inputimg, first_point))
    {
        deque<Point> line;
        line.push_back(first_point);
        //由于第一个点不一定是线段的起始位置，双向找
        Point this_point = first_point;
        int this_flag = 0;
        Point next_point;
        int next_flag;
        while (cvline_findNextPoint(neighbor_points, _inputimg, this_point, this_flag, next_point, next_flag))
        {
            line.push_back(next_point);
            this_point = next_point;
            this_flag = next_flag;
        }
        //找另一边
        this_point = first_point;
        this_flag = 0;
        //cout << "flag:" << this_flag << endl;
        while (cvline_findNextPoint(neighbor_points, _inputimg, this_point, this_flag, next_point, next_flag))
        {
            line.push_front(next_point);
            this_point = next_point;
            this_flag = next_flag;
        }
        if (line.size() > 10)
        {
            _outputlines.push_back(line);
        }
    }
}

void PubFun::cvline_findLines(Mat &mat, vector<QList<QPointF> > &_outputlines)
{
    if(mat.empty() == true)
        return;

    vector<deque<Point>> lines;
    cvline_findLines(mat, lines);

    int nLine = lines.size();
    for(int l=0;l<nLine;l++)
    {
        deque<Point> &dq = lines[l];

        int nPt = dq.size();
        QList<QPointF> lst;
        for(int p=0;p<nPt;p++)
        {
            lst.push_back(QPointF(dq[p].x, dq[p].y));
        }

        _outputlines.push_back(lst);
    }
}

void PubFun::cvline_findLines(QString sImgName, vector<QList<QPointF> > &_outputlines)
{
    cv::Mat mat = cv::imread(sImgName.toStdString().c_str(), cv::IMREAD_GRAYSCALE);
    cvline_findLines(mat, _outputlines);
}

void PubFun::cvline_findContours(Mat &mat, vector<QList<QPointF> > &vectLines)
{
    if(mat.type() != CV_8UC1)
        return;

    if(mat.empty() == true)
    {
        qDebug() << "cvline_findContours(): invalid image";
        return;
    }

    //cv::OutputArrayOfArrays contours;//
    vector< vector<Point> > contours;

    cv::findContours(
                        mat,
                        contours,
                        cv::RETR_EXTERNAL,
                        cv::CHAIN_APPROX_NONE
                    );

    int nCountour = contours.size();
    if(nCountour<=0)
    {
        qDebug() << "cvline_findContours: no countours found";
        return;
    }

    for(int c=0; c<nCountour; c++)
    {
        vector<Point> &v = contours[0];
        int nPt = v.size();

        QList<QPointF> listPt;
        for(int p=0; p<nPt; p++)
        {
            listPt.push_back(QPointF(v[p].x, v[p].y));
        }

        vectLines.push_back(listPt);
    }

}
#endif

void PubFun::str_TrimLeft(string &strOri, string strTrim)
{
    size_t strLen = strTrim.size();
    while(1)
    {
        int p = strOri.find(strTrim);
        if(p != 0)
        {
            break;
        }
        else
        {
            string ret;
            size_t n = strOri.size();
            for(size_t i=strLen;i<n;i++)
            {
                ret += strOri[i];
            }

            strOri = ret;
        }
    }

}

void PubFun::str_TrimRight(string &strOri, string strTrim)
{
    while (1)
    {
        int p = strOri.rfind(strTrim);

        if(p==string::npos || p != strOri.size() - strTrim.size())
            break;

        strOri = strOri.substr(0, p);
    }
}

void PubFun::str_Replace(string &strOri, string strOld, string strNew)
{
    if(strOri.empty() || strOld.empty())
        return;

    int p = -1;
    while(1)
    {
        p = strOri.find(strOld, p+1);
        if(p == string::npos)
            break;

        strOri.replace(p, strOld.size(), strNew);
    }
}

bool PubFun::copyDir(const QString &fromDir, const QString &toDir, bool coverFileIfExist)
{
    QDir sourceDir(fromDir);
    QDir targetDir(toDir);
    if(!targetDir.exists())
    {
        if(!targetDir.mkdir(targetDir.absolutePath()))
            return false;
    }

    QFileInfoList fileInfoList = sourceDir.entryInfoList();
    foreach(QFileInfo fileInfo, fileInfoList)
    {
        if(fileInfo.fileName() == "." || fileInfo.fileName() == "..")
            continue;

        if(fileInfo.isDir())
        {
            if(!copyDir(fileInfo.filePath(),
                targetDir.filePath(fileInfo.fileName()),
                coverFileIfExist))
                return false;
        }
        else
        {
            if(coverFileIfExist && targetDir.exists(fileInfo.fileName()))
            {
                targetDir.remove(fileInfo.fileName());
            }

            //copy file
            if(!QFile::copy(fileInfo.filePath(), targetDir.filePath(fileInfo.fileName())))
            {
                    return false;
            }
        }
    }

    return true;
}

bool PubFun::clearDir(const QString &dirName, bool bRemove)
{
    bool result = true;
    QDir dir(dirName);

    if (dir.exists())
    {
        foreach (QFileInfo info, dir.entryInfoList(QDir::NoDotAndDotDot | QDir::System | QDir::Hidden | QDir::AllDirs | QDir::Files, QDir::DirsFirst))
        {
            if (info.isDir())
            {
                result = clearDir(info.absoluteFilePath(), true);
            }
            else
            {
                result = QFile::remove(info.absoluteFilePath());
                            }

            if (!result)
            {
                qDebug() << " clearDir failed : " << info.absoluteFilePath();
                return result;
            }
        }

        if(bRemove)
        {
            QDir parentDir(QFileInfo(dirName).absolutePath());
            result = parentDir.rmdir(dirName);
        }

    }

    return result;
}

QRect PubFun::getRcMapToParent(QWidget *childWgt)
{
    if(!childWgt)
        return QRect(0,0,0,0);

    QRect vrc = childWgt->rect();
    QPoint ptLT = childWgt->mapToParent(QPoint(vrc.left(), vrc.top()));
    QPoint ptRB = childWgt->mapToParent(QPoint(vrc.right(), vrc.bottom()));
    vrc.setTopLeft(ptLT);
    vrc.setBottomRight(ptRB);

    return vrc;
}

void PubFun::setWidgetBackColor(QWidget* pWidget, QColor clor)
{
    //backgrond clolor
    pWidget->setAutoFillBackground(true);
    pWidget->setAttribute(Qt::WA_StyledBackground);

    QPalette pal = pWidget->palette();
    pal.setColor(QPalette::Window, clor);
    pWidget->setPalette(pal);
}

