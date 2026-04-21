#pragma once
#include <QVTKOpenGLNativeWidget.h>
#include <vtkActor.h>
#include <vtkCellPicker.h>
#include <vtkGPUVolumeRayCastMapper.h>
#include <vtkProperty.h>
#include <vtkRenderWindowInteractor.h>.h>
#include <vtkSmartPointer.h>
#include <vtkSphereSource.h>

#include <QComboBox>
#include <QMainWindow>
#include <QPushButton>
#include <QSlider>
#include <memory>

#include "registerviewmodel.h"
#include "vtkFlyingEdges3D.h"
#include "vtkGenericOpenGLRenderWindow.h"
#include "vtkImageData.h"
#include "vtkPolyDataMapper.h"
#include "vtkRenderer.h"

struct PointMarker {
    vtkSmartPointer<vtkActor> actor;
    vtkRenderer* renderer = nullptr;
    bool isStl;
};

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(std::shared_ptr<RegisterViewModel> rVm, QWidget *parent = nullptr);
    ~MainWindow();

    void setupUI();
    void setupData();
public slots:
    void onSliderChanged(int val);
    void onAutoRegisterClicked(bool checked);
    void onDatasetChanged(int index);
    void onSelectPointsToggled(bool checked);

   private:
    void resetSelectPointsMode();
    bool eventFilter(QObject* watched, QEvent* event) override;
    bool handlePointSelectionClick(QMouseEvent* mouseEvent);
    void addPointMarker(vtkRenderer* renderer, const double point[3], bool);
    void clearPointMarkers();
    void setStlRegistrationState(vtkMatrix4x4* matrix, bool registered);

    vtkNew<vtkCellPicker> m_cellPicker;
    std::vector<PointMarker> m_pointMarkers;

    bool m_selectPointsMode = false;

    QWidget* m_container = nullptr;
    QVTKOpenGLNativeWidget *m_vtkWidget = nullptr;
    QWidget *m_rightControlsWidget = nullptr;
    QSlider *m_isoSlider = nullptr;
    QPushButton *m_autoRegisterBtn = nullptr;
    QPushButton* m_selectPointsBtn = nullptr;
    QPushButton* m_clearSelectPointsBtn = nullptr;
    QComboBox* m_datasetCombo;

    vtkNew<vtkGenericOpenGLRenderWindow> m_renderWindow;
    vtkNew<vtkRenderer> m_leftRenderer;
    vtkNew<vtkRenderer> m_rightRenderer;

    vtkNew<vtkPolyDataMapper> m_stlMapper;
    vtkNew<vtkActor> m_stlActor;

    vtkNew<vtkGPUVolumeRayCastMapper> m_dicomMapper;
    vtkNew<vtkVolume> m_dicomVolume;

    // iso surface
    vtkNew<vtkActor> m_dicomActor;
    vtkNew<vtkPolyDataMapper> m_dicomSurfaceMapper;

    std::shared_ptr<RegisterViewModel> m_regVM;
    double m_currentIso = 0;
};
