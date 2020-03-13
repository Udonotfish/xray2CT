#include "eschartwidget.h"
#include "eschart.h"
ESChartWidget::ESChartWidget(QWidget *parent): QWidget(parent)
{

}

void ESChartWidget::paintEvent(QPaintEvent *event)
{
    QPainter painter(this);
//    m_chart->draw(painter);
    m_chart->drawESPoint(painter);
    m_chart->drawESLine(painter);
}

void ESChartWidget::mouseMoveEvent(QMouseEvent *event)
{
    qDebug() << "mouse move event";
    if(1)
    {
        m_chart->updateCurrentPoint(event->pos());
        this->update();
    }

}

void ESChartWidget::mousePressEvent(QMouseEvent *event)
{
    qDebug() << "mouse press event";
    qDebug() << event->pos();
    PointSelected = m_chart->PressOnPoint(event);
}

void ESChartWidget::mouseReleaseEvent(QMouseEvent *event)
{

}

void ESChartWidget::setESChartPointer(ESChart *chart)
{
    m_chart = chart;
}

void ESChartWidget::keyPressEvent(QKeyEvent *event)
{

}
