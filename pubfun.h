#ifndef PUBFUN_H
#define PUBFUN_H

//#include "predefine.h"
#include <QPainterPath>
#include <QRect>
#include <QtMath> // for qPow
#include <QDebug>
#include <QString>
#include <QDir>
#include <QWidget>
#include "ogre.hpp"
using namespace Ogre;

//--------------------------------------------------------------
template <class T>
class XSPoint2
{
public:
    typedef T ScalarType;

    union
    {
        T v[2];
        struct
        {
            T x;
            T y;
        };
    };

    inline XSPoint2() {}

    inline XSPoint2(const ScalarType nx, const ScalarType ny)
    {
        v[0] = nx;
        v[1] = ny;
    }

    inline XSPoint2(XSPoint2 const &p)
    {
        v[0] = p.v[0];
        v[1] = p.v[1];
    }

    inline const ScalarType &X() const
    {
        return v[0];
    }
    inline const ScalarType &Y() const
    {
        return v[1];
    }
    inline ScalarType &X()
    {
        return v[0];
    }
    inline ScalarType &Y()
    {
        return v[1];
    }

    inline const ScalarType *V() const
    {
        return v;
    }

    inline ScalarType *V()
    {
        return v;
    }

    inline ScalarType &V(const int i)
    {
        assert(i >= 0 && i < 2);
        return v[i];
    }

    inline const ScalarType &V(const int i) const
    {
        assert(i >= 0 && i < 2);
        return v[i];
    }

    inline XSPoint2 &operator=(XSPoint2 const &p)
    {
        v[0] = p.v[0];
        v[1] = p.v[1];
        return *this;
    }
    // dot product
    inline ScalarType operator*(XSPoint2 const &p) const
    {
        return (v[0] * p.v[0] + v[1] * p.v[1]);
    }

    inline ScalarType dot(const XSPoint2 &p) const
    {
        return (*this) * p;
    }

    // cross product
    inline ScalarType operator^(XSPoint2 const &p) const
    {
        return v[0] * p.v[1] - v[1] * p.v[0];
    }

    inline ScalarType cross(const XSPoint2 &p) const
    {
        return (*this) ^ p;
    }

    /* Linearity for 2d points (operators +, -, *, /, *= ...) */
    inline XSPoint2 operator+(XSPoint2 const &p) const
    {
        return XSPoint2<ScalarType>(v[0] + p.v[0], v[1] + p.v[1]);
    }

    inline XSPoint2 operator-(XSPoint2 const &p) const
    {
        return XSPoint2<ScalarType>(v[0] - p.v[0], v[1] - p.v[1]);
    }

    inline XSPoint2 operator*(const ScalarType s) const
    {
        return XSPoint2<ScalarType>(v[0] * s, v[1] * s);
    }

    inline XSPoint2 operator/(const ScalarType s) const
    {
        return XSPoint2<ScalarType>(v[0] / s, v[1] / s);
    }

    inline XSPoint2 &operator+=(XSPoint2 const &p)
    {
        v[0] += p.v[0];
        v[1] += p.v[1];
        return *this;
    }

    inline XSPoint2 &operator-=(XSPoint2 const &p)
    {
        v[0] -= p.v[0];
        v[1] -= p.v[1];
        return *this;
    }

    inline XSPoint2 &operator*=(const ScalarType s)
    {
        v[0] *= s;
        v[1] *= s;
        return *this;
    }

    inline XSPoint2 &operator/=(const ScalarType s)
    {
        v[0] /= s;
        v[1] /= s;
        return *this;
    }

    // returns the norm (Euclidian)
    inline ScalarType Norm(void) const
    {
        return std::sqrt(v[0] * v[0] + v[1] * v[1]);
    }

    // returns the squared norm (Euclidian)
    inline ScalarType SquaredNorm(void) const
    {
        return (v[0] * v[0] + v[1] * v[1]);
    }

    //modify norm
    XSPoint2 ResetNorm(XSPoint2 const &p, ScalarType newNorm) const
    {
       XSPoint2 t(p.v[0] - v[0], p.v[1] - v[1]);
       ScalarType tnom = t.Norm();
       assert(tnom > 0);
       double f = newNorm*1.00/tnom;

       t.v[0] = v[0] + t.v[0]*f;
       t.v[1] = v[1] + t.v[1]*f;

       return t;
    }

    /*               p
     *               |
     *               |
     *      left----this
    */
    //return the left normal vector across this with special norm
    XSPoint2 NormalVectorLeft(XSPoint2 const &p, ScalarType norm) const
    {
        //return Point2(v[0] + p.v[1] - v[1],  v[1] - p.v[0] + v[0]);
        XSPoint2 t(p.v[1] - v[1], v[0] - p.v[0]);
        ScalarType tnom = t.Norm();
        assert(tnom > 0);
        double f = norm*1.00/tnom;

        t.v[0] = v[0] + t.v[0]*f;
        t.v[1] = v[1] + t.v[1]*f;

        return t;
    }

    /*               p
     *               |
     *               |
     *             this----right
    */
    //return the right normal vector across this with special norm
    XSPoint2 NormalVectorRight(XSPoint2 const &p, ScalarType norm) const
    {
        //return Point2(v[0] - p.v[1] + v[1],  v[1] + p.v[0] - v[0]);
        XSPoint2 t( v[1] - p.v[1],  p.v[0] - v[0]);
        ScalarType tnom = t.Norm();
        assert(tnom > 0);
        double f = norm*1.00/tnom;

        t.v[0] = v[0] + t.v[0]*f;
        t.v[1] = v[1] + t.v[1]*f;

        return t;
    }
};

template <class T>
inline T Norm(XSPoint2<T> const &p)
{
    return p.Norm();
}

template <class T>
inline T Distance(XSPoint2<T> const &p1, XSPoint2<T> const &p2)
{
    return Norm(p1 - p2);
}

template <class T>
T SegmentPointDistance(const XSPoint2<T> &s0, const XSPoint2<T> &s1, const XSPoint2<T> &p, XSPoint2<T> &closest)
{
    T dist;
    XSPoint2<T> e = s1 - s0;
    T eSquaredNorm = e.SquaredNorm();
    if (eSquaredNorm < (std::numeric_limits<T>::min)())
    {
        closest = (s0 + s1) / T(2.0);
        dist = Distance(closest, p);
    }
    else
    {
        T t = ((p - s0) * e) / eSquaredNorm;
        if (t < 0)
            t = 0;
        else if (t > 1)
            t = 1;

        closest = s0 * (1.0 - t) + s1 * t; //improved precision for a closest point
        dist = Distance(p, closest);
        //qDebug()<< "tp2";
    }

    return dist;
}

typedef struct _POLYPOS
{
    //this is a point in bezier point set

    bool bValide; // is this point valid ?
    QPointF pt; // this point x,y position
    int bzPosStart;     //neighbour point:  previous point in bezier point set
    int bzPosEnd;       //neighbour point:  next point in bezier point set
    int oriPosStart;    //neighbour point:  previous point in original point set
    int oriPosEnd;      //neighbour point:  next point in original point set

    _POLYPOS()
        :bValide(false)
        ,pt(QPointF(0.0, 0.0))
        ,bzPosStart(-1)
        ,bzPosEnd(-1)
        ,oriPosStart(-1)
        ,oriPosEnd(-1)
    {

    }

    void reset()
    {
        bValide = false;
        pt = QPointF(0.0,0.0);
        bzPosStart = -1;
        bzPosEnd = -1;
        oriPosStart = -1;
        oriPosEnd = -1;
    }

}POLYPOS;

class PubFun
{
public:
    PubFun();


public:
//    static bool ApartRect(FilmFormat &fmt, const QRect &rc, int nHorInterv, int nVerInterv, vector <QRect> &RetImgRc);

//    static bool ApartStandardRect(int nRow, int nCol,  const QRect &rc, int nHorInterv,int nVerInterv, vector <QRect> &RetImgRc);

//    static bool ApartStandardRect2D(int nRow, int nCol,  const QRect &rc, int nInterv,vector < vector<QRect> > &RetImgRc);

    static bool ApartStandardRectByWH2D(int nCellWidth, int nCellHeight,  const QRect &rc, int nHorInterv,int nVerInterv, vector < vector<QRect> > &RetImgRc);

    //2D array
    static void** ArrayNew(unsigned long nRow, unsigned long nCol, unsigned long nSize,bool bFill4Byte, unsigned long *nNewRow, unsigned long* nNewCol);

    static void ArrayFree(void **pArr,int iflag=0);

    static void** ArrayCopy(void** pSrcArr,unsigned long nSrcRow, unsigned long nSrcCol, unsigned long nSrcSize);

    static QRect GetShowRcByImgSize(QRect rc, double ImgWidth, double ImgHeight);

    static QRectF GetShowRcByImgSizeF(QRect rc, double ImgWidth, double ImgHeight);

    static bool tool_IsPtOnLine( QPoint DstPt,const QPoint &pt1,const QPoint &pt2 );

    static bool tool_IsPtOnLineF( QPoint DstPt,const QPointF &pt1,const QPointF &pt2 );

    static bool tool_GetLineAcrossRc( QRect rc,QPoint p1,QPoint p2,QPoint &retP1,QPoint &retP2 );

    static bool getNormalLine(const vector<QPointF> &vectCenterPoint, const vector<double> &vectRadius, vector<QPointF> &vectContours);

    //--------bezier line-----------------------------------------------------------
    static int Bezier3DefaultPointNum;

    //count为插入点数, outPoints为输出点集合，长度为count + 2(含首尾)
    static void bezier_parseBezier(const Ogre::Vector2& start, const Ogre::Vector2& end,  const Ogre::Vector2& control1, const Ogre::Vector2& control2, int count, std::vector<Ogre::Vector2>& outPoints);

    //根据当前点pt,前一点p1, 后一点p2计算当前点对应的控制点control1 control2
    static void bezier_getControlPoint(const Ogre::Vector2& pt, const Ogre::Vector2& p1, const Ogre::Vector2& p2, Ogre::Vector2& control1, Ogre::Vector2& control2, double ratio);

    //ratio参数为调整系数，因为相对与总长，一般取0.1-0.3之间(mapInToOut:inPoints中的每个点在outPoints中的位置)
    static void bezier_parsePolyline(const std::vector<Ogre::Vector2>& inPoints, int count, std::vector<Ogre::Vector2>& outPoints, double ratio,vector<int> &mapInToOut);

    static void bezier_1_parseBezier(const Ogre::Vector2& start, const Ogre::Vector2& end,  const Ogre::Vector2& control1, const Ogre::Vector2& control2, int count, std::vector<Ogre::Vector2>& outPoints);

    //ratio参数为调整系数，因为相对与总长，一般取0.1-0.3之间(mapInToOut:inPoints中的每个点在outPoints中的位置)
    static void bezier_parsePolygon(const std::vector<Ogre::Vector2>& inPoints, int count, std::vector<Ogre::Vector2>& outPoints, double ratio,vector<int> &mapInToOut);

    //--------------------自定义抽稀方法------------------------------------------------
    //点p到直线p1-p2的距离
    static double distanceFromPointToLine(QPointF &p, QPointF &p1, QPointF &p2 );
    static double distanceFromPointToLine(Ogre::Vector2 &p, Ogre::Vector2 &p1, Ogre::Vector2 &p2 );

    //抽稀折线,依据点到线的距离
    static int thinPolyLineByDistance(QList<QPointF> &srcPolyLine, double fThreshold=1.0);
    static int thinPolyLineByDistance(vector<QPointF> &srcPolyLine, double fThreshold=1.0);
    static int thinPolyLineByDistance(vector<Ogre::Vector2> &srcPolyLine, double fThreshold=1.0);

    //抽稀折线,依据夹角
    static double getAngleB(const QPointF &A, const QPointF &B, const QPointF &C);
    static double getAngleB(const Ogre::Vector2 &A, const Ogre::Vector2 &B, const Ogre::Vector2 &C);
    static int thinPolyLineByAngle(vector<QPointF> &srcPolyLine, double fThreshold=170);
    static int thinPolyLineByAngle(QList<QPointF> &srcPolyLine, double fThreshold=170);
    static int thinPolyLineByAngle(vector<Ogre::Vector2> &srcPolyLine, double fThreshold=170);

    //-------------------typedef smooth methon---------------------------------------------------
     static bool isSamePoint(QPointF &a, QPointF &b);

     static bool SmoothPolyLine2(QList<QPointF> &lst, double fAngle=160.0, double fStep=0.5);
     static bool SmoothPolyLine2(vector<QPointF> &lst, double fAngle=160.0, double fStep=0.5);

     //-----------------math ------------------------------------------------------------
     static double PointToSegDistance(double x, double y, double x1, double y1, double x2, double y2);

     static double PointToSegDistance(double x, double y, double x1, double y1, double x2, double y2, QPointF &closestPt);



     //-----------------openCV ------------------------------------------------------------
#ifdef USE_OPENCV
private:
    static bool cvline_findNextPoint(vector<Point> &_neighbor_points, Mat &_image, Point _inpoint, int flag, Point& _outpoint, int &_outflag);
    static bool cvline_findFirstPoint(Mat &_inputimg, Point &_outputpoint);
    static void cvline_findLines(Mat &_inputimg, vector<deque<Point>> &_outputlines);
public:
    static void cvline_findLines(Mat &mat,vector<QList<QPointF>> &_outputlines);
    static void cvline_findLines(QString sImgName,vector<QList<QPointF>> &_outputlines);

    static void cvline_findContours(Mat &mat, vector< QList<QPointF> > &vectLines);
#endif

    //------------------some string process--------------------------------------------------
    static void str_TrimLeft(string &strOri, string strTrim);

    static void str_TrimRight(string &strOri, string strTrim);

    static void str_Replace(string &strOri, string strOld, string strNew);

    //------------------copy dir--------------------------------------------------
    static bool copyDir(const QString &fromDir, const QString &toDir, bool coverFileIfExist);
    static bool clearDir(const QString &dirName, bool bRemove=false);

    //------------------Qt Widget--------------------------------------------------
    static QRect getRcMapToParent(QWidget* childWgt);

    static void setWidgetBackColor(QWidget* pWidget, QColor clor);
};

#endif // PUBFUN_H
