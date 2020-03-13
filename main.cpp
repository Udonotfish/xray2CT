#include <QApplication>

#include <QVTKOpenGLWidget.h>
#include <vtkSmartPointer.h>
#include <vtk3DS.h>

#include "eschartwidget.h"
#include "eschart.h"
int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

//    //vtkSmartPointer<QVTKOpenGLWidget> v;
//    auto v= new QVTKOpenGLWidget;
//    v->show();
//    return a.exec();
    ESChartWidget* chart_widget = new ESChartWidget();
    ESChart* chart = new ESChart(chart_widget);
#if 0
    QFile file("1.csv");
    if(file.exists())
    {
        chart->setHorRuler("hor",2,0,400,20,20);
        chart->setVerRuler("ver",2,0,600,20,200);
        chart->setRect(QRect(0,0,1440,760));
        chart->readChartFromCSV("1.csv");
        chart_widget->setESChartPointer(chart);
        chart_widget->show();
    }
    else
    {
        qDebug() << "load file failed";
    }
#endif

#if 1
    chart->setHorRuler("hor",2,0,400,20,20);
    chart->setVerRuler("ver",2,0,600,20,200);
    chart->setRect(QRect(0,0,1440,760));
    chart_widget->setESChartPointer(chart);
    chart_widget->show();
#endif
    return a.exec();

}
