#pragma once
#include <vtkColorTransferFunction.h>
#include <vtkDICOMDirectory.h>
#include <vtkDICOMReader.h>
#include <vtkImageGaussianSmooth.h>
#include <vtkImageData.h>
#include <vtkImageThreshold.h>
#include <vtkPiecewiseFunction.h>
#include <vtkPolyData.h>
#include <vtkPolyDataNormals.h>
#include <vtkProperty.h>
#include <vtkVolumeProperty.h>
#include <vtkWindowedSincPolyDataFilter.h>

#include <QObject>

#include "vtkFlyingEdges3D.h"
#include "vtkSTLReader.h"

class DataLoader : public QObject
{
    Q_OBJECT
public:
    explicit DataLoader(QObject *parent = nullptr);
    bool loadStl(QString &filePath);
    bool loadDicom(QString &filePath);
    void loadTestingDataset(int index);
    void setupTransferFunctions();
    // bool loadCbct(QString &filePath);
    vtkSmartPointer<vtkPolyData> getStlData();
    vtkSmartPointer<vtkImageData> getDicomData();
    vtkSmartPointer<vtkImageData> getDicomDataNoFilter();
    vtkSmartPointer<vtkVolumeProperty> getVolProps();

    vtkSmartPointer<vtkPolyData> getRawSurfaceData(double contourValue);
    vtkSmartPointer<vtkPolyData> getSurfaceData(double contourValue);
    vtkSmartPointer<vtkProperty> getSurfaceProps();
signals:
    void stlLoaded();
    void cbctLoaded();

private:
    vtkSmartPointer<vtkPolyData> m_stlData;
    vtkSmartPointer<vtkImageData> m_dicomData;
    vtkSmartPointer<vtkImageData> m_dicomDataNoFilter;

    vtkNew<vtkSTLReader> m_stlReader;
    vtkNew<vtkDICOMReader> m_dicomReader;
    vtkNew<vtkDICOMDirectory> m_dicomDir;

    vtkSmartPointer<vtkPiecewiseFunction> m_opacityPiecewiseFunction;
    vtkSmartPointer<vtkColorTransferFunction> m_colorTransferFunction;
    vtkSmartPointer<vtkVolumeProperty> m_prop;

    // isosurface extraction
    vtkNew<vtkImageGaussianSmooth> m_surfaceAntiAliasFilter;
    vtkNew<vtkFlyingEdges3D> m_isoAlgo;
    vtkSmartPointer<vtkProperty> m_isoProp;
    vtkNew<vtkWindowedSincPolyDataFilter> m_isoFilter;
    vtkNew<vtkPolyDataNormals> m_normalsCalc;
    vtkSmartPointer<vtkImageThreshold> m_thresholder;
};
