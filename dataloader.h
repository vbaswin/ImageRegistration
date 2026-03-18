#pragma once
#include <QObject>
#include "vtkFlyingEdges3D.h"
#include "vtkSTLReader.h"
#include <vtkColorTransferFunction.h>
#include <vtkDICOMDirectory.h>
#include <vtkDICOMReader.h>
#include <vtkImageData.h>
#include <vtkPiecewiseFunction.h>
#include <vtkPolyData.h>
#include <vtkPolyDataNormals.h>
#include <vtkProperty.h>
#include <vtkVolumeProperty.h>
#include <vtkWindowedSincPolyDataFilter.h>

class DataLoader : public QObject
{
    Q_OBJECT
public:
    explicit DataLoader(QObject *parent = nullptr);
    bool loadStl(QString &filePath);
    bool loadDicom(QString &filePath);
    void setupTransferFunctions();
    // bool loadCbct(QString &filePath);
    vtkSmartPointer<vtkPolyData> getStlData();
    vtkSmartPointer<vtkImageData> getDicomData();
    vtkSmartPointer<vtkVolumeProperty> getVolProps();

    vtkSmartPointer<vtkPolyData> getSurfaceData(double contourValue);
    vtkSmartPointer<vtkProperty> getSurfaceProps();
signals:
    void stlLoaded();
    void cbctLoaded();

private:
    vtkSmartPointer<vtkPolyData> m_stlData;
    vtkSmartPointer<vtkImageData> m_dicomData;

    vtkNew<vtkSTLReader> m_stlReader;
    vtkNew<vtkDICOMReader> m_dicomReader;
    vtkNew<vtkDICOMDirectory> m_dicomDir;

    vtkSmartPointer<vtkPiecewiseFunction> m_opacityPiecewiseFunction;
    vtkSmartPointer<vtkColorTransferFunction> m_colorTransferFunction;
    vtkSmartPointer<vtkVolumeProperty> m_prop;

    // isosurface extraction
    vtkNew<vtkFlyingEdges3D> m_isoAlgo;
    vtkSmartPointer<vtkProperty> m_isoProp;
    vtkNew<vtkWindowedSincPolyDataFilter> m_isoFilter;
    vtkNew<vtkPolyDataNormals> m_normalsCalc;
};
