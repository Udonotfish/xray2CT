#ifndef ESCHARTWIDGET_H
#define ESCHARTWIDGET_H
#include <QWidget>
class ESChart;

class ESChartWidget : public QWidget
{
    Q_OBJECT
public:
    explicit ESChartWidget(QWidget *parent = nullptr);
    void paintEvent(QPaintEvent *event);
signals:

public slots:
    void setESChartPointer(ESChart* chart);
protected:
//    virtual void paintEvent(QPaintEvent *event);
    virtual void keyPressEvent ( QKeyEvent * event );
    virtual void mouseMoveEvent ( QMouseEvent * event );
    virtual void mousePressEvent ( QMouseEvent * event );
    virtual void mouseReleaseEvent ( QMouseEvent * event );
    ESChart* m_chart;
    bool PointSelected = false;
};

#endif // ESCHARTWIDGET_H
