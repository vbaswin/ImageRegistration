#include "mainwindow.h"

#include <vtkSphereSource.h>

#include <QElapsedTimer>
#include <QEvent>
#include <QHBoxLayout>
#include <QMouseEvent>
#include <QPushButton>
#include <QSlider>

MainWindow::MainWindow(std::shared_ptr<RegisterViewModel> rVM, QWidget *parent)
    : QMainWindow(parent)
    , m_regVM(rVM)
{
    this->resize(1920, 1080);
    setupUI();
    setupData();

    connect(m_isoSlider, &QSlider::valueChanged, this, &MainWindow::onSliderChanged);
    connect(m_autoRegisterBtn, &QPushButton::toggled, this, &MainWindow::onAutoRegisterClicked);
    connect(m_datasetCombo, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &MainWindow::onDatasetChanged);
    connect(m_selectPointsBtn, &QPushButton::toggled, this,
            &MainWindow::onSelectPointsToggled);
    connect(m_clearSelectPointsBtn, &QPushButton::clicked, this,
            &MainWindow::clearPointMarkers);
}

MainWindow::~MainWindow() {}

void MainWindow::setupUI()
{
    m_container = new QWidget(this);
    this->setCentralWidget(m_container);
    QHBoxLayout *hLayout = new QHBoxLayout(m_container);

    m_vtkWidget = new QVTKOpenGLNativeWidget(m_container);
    m_rightControlsWidget = new QWidget(m_container);
    m_rightControlsWidget->setObjectName("rightControls");

    hLayout->addWidget(m_vtkWidget, 6);
    hLayout->addWidget(m_rightControlsWidget, 1);

    m_autoRegisterBtn = new QPushButton("Auto Register", m_rightControlsWidget);
    m_selectPointsBtn = new QPushButton("Select Points", m_rightControlsWidget);
    m_clearSelectPointsBtn =
        new QPushButton("Clear Points", m_rightControlsWidget);
    m_autoRegisterBtn->setCheckable(true);
    m_selectPointsBtn->setCheckable(true);

    m_datasetCombo = new QComboBox(m_rightControlsWidget);
    m_datasetCombo->addItems({"SAIFI (Iso: -999)", "Vijaya Upper (Iso: 1)",
                              "Rajeev Lower (Iso: 1)", "Unknown (Iso: 1)",
                              "Ashish Upper (Iso: 1)", "Parveen Lower"});
    // m_datasetCombo->

    m_isoSlider = new QSlider(Qt::Horizontal, m_rightControlsWidget);

    m_isoSlider->setMinimum(-1000);
    // m_isoSlider->setMaximum(100);
    m_isoSlider->setMaximum(1000);
    m_isoSlider->setValue(-650);
    m_isoSlider->setMinimumWidth(200);

    QVBoxLayout *vLayout = new QVBoxLayout(m_rightControlsWidget);

    vLayout->addWidget(m_datasetCombo);
    vLayout->addWidget(m_autoRegisterBtn);
    vLayout->addWidget(m_selectPointsBtn);
    vLayout->addWidget(m_clearSelectPointsBtn);
    vLayout->addWidget(m_isoSlider);

    m_vtkWidget->SetRenderWindow(m_renderWindow);
    m_vtkWidget->installEventFilter(this);
    m_cellPicker->SetTolerance(0.005);

    m_renderWindow->AddRenderer(m_leftRenderer);
    m_renderWindow->AddRenderer(m_rightRenderer);

    m_leftRenderer->SetBackground(2 / 255.0, 8 / 255.0, 5 / 255.0);
    m_rightRenderer->SetBackground(7 / 255.0, 3 / 255.0, 6 / 255.0);

    // [xmin, ymin, xmax, ymax]
    double leftViewPort[4] = {0.0, 0.0, 0.5, 1.0};
    double rightViewPort[4] = {0.5, 0.0, 1.0, 1.0};

    m_leftRenderer->SetViewport(leftViewPort);
    m_rightRenderer->SetViewport(rightViewPort);

    // m_currentIso = 1;
    // m_currentIso = -999;
    m_currentIso = 400;
}

void MainWindow::onSelectPointsToggled(bool checked) {
    m_selectPointsMode = checked;
    qDebug() << "Select points mode: " << (checked ? "ON" : "OFF");
}

bool MainWindow::eventFilter(QObject* watched, QEvent* event) {
    if (watched == m_vtkWidget && m_selectPointsMode &&
        event->type() == QEvent::MouseButtonPress) {
        auto* mouseEvent = static_cast<QMouseEvent*>(event);

        if (mouseEvent->button() == Qt::LeftButton) {
            return handlePointSelectionClick(mouseEvent);
        }
    }
    return QMainWindow::eventFilter(watched, event);
}

bool MainWindow::handlePointSelectionClick(QMouseEvent* mouseEvent) {
    const int x = mouseEvent->pos().x();
    const int y = m_vtkWidget->height() - mouseEvent->pos().y() - 1;

    vtkRenderWindowInteractor* interactor = m_renderWindow->GetInteractor();

    if (!interactor) {
        qDebug() << "No vtk interactor found";
        return true;
    }

    vtkRenderer* renderer = interactor->FindPokedRenderer(x, y);
    if (!renderer) {
        qDebug() << "No renderer found at click.";
        return true;
    }

    const int picked = m_cellPicker->Pick(x, y, 0.0, renderer);

    if (!picked) {
        qDebug() << "No surface picked.";
        return true;
    }

    vtkActor* pickedActor = m_cellPicker->GetActor();
    if (pickedActor != m_stlActor.GetPointer() &&
        pickedActor != m_dicomActor.GetPointer()) {
        qDebug() << "Picked actor is not STL or CBCT.";
        return true;
    }

    double point[3];
    m_cellPicker->GetPickPosition(point);

    addPointMarker(renderer, point);

    qDebug() << "Picked point: " << point[0] << point[1] << point[2];

    m_renderWindow->Render();
    return true;
}

void MainWindow::addPointMarker(vtkRenderer* renderer, const double point[3]) {
    vtkSmartPointer<vtkSphereSource> sphere =
        vtkSmartPointer<vtkSphereSource>::New();

    sphere->SetCenter(point[0], point[1], point[2]);
    sphere->SetRadius(1.0);
    sphere->SetThetaResolution(16);
    sphere->SetPhiResolution(16);
    sphere->Update();

    vtkSmartPointer<vtkPolyDataMapper> mapper =
        vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(sphere->GetOutputPort());

    vtkSmartPointer<vtkActor> marker = vtkSmartPointer<vtkActor>::New();

    marker->SetMapper(mapper);
    marker->GetProperty()->SetColor(0.0, 1.0, 0.0);
    marker->GetProperty()->SetAmbient(0.4);
    marker->GetProperty()->SetDiffuse(0.8);

    renderer->AddActor(marker);
    m_pointMarkers.push_back({marker, renderer});
}

void MainWindow::clearPointMarkers() {
    for (const auto& marker : m_pointMarkers) {
        if (marker.renderer) {
            marker.renderer->RemoveActor(marker.actor);
        }
    }

    m_pointMarkers.clear();
    m_renderWindow->Render();
}

void MainWindow::setupData() {
    // m_stlPolyData = m_regVM->getStlData();
    m_stlMapper->SetInputData(m_regVM->getStlData());
    m_stlActor->SetMapper(m_stlMapper);
    m_leftRenderer->AddActor(m_stlActor);

    /*
    m_dicomMapper->SetInputData(m_regVM->getDicomData());
    m_dicomVolume->SetMapper(m_dicomMapper);
    m_dicomVolume->SetProperty(m_regVM->getVolProps());
    m_rightRenderer->AddVolume(m_dicomVolume);
    */
    m_dicomSurfaceMapper->SetInputData(m_regVM->getSurfaceData(m_currentIso));
    m_dicomSurfaceMapper->SetInputData(m_regVM->getSurfaceData(m_currentIso));
    m_dicomSurfaceMapper->ScalarVisibilityOff();
    m_dicomActor->SetMapper(m_dicomSurfaceMapper);
    m_dicomActor->SetProperty(m_regVM->getSurfaceProps());
    m_rightRenderer->AddActor(m_dicomActor);

    m_leftRenderer->ResetCamera();
    m_rightRenderer->ResetCamera();

    m_renderWindow->Render();
}
void MainWindow::onDatasetChanged(int index) {
    clearPointMarkers();
    // 1. Tell ViewModel to load new datasets into VTK stream
    m_regVM->loadTestingDataset(index);

    // 2. Adjust target isovalue dynamically (SAIFI == -999, Others == 1)
    // m_currentIso = (index == 0) ? -999 : 1;
    m_regVM->runDiagnosticCropTest();

    // Qt Best Practice: Block signals so the slider update doesn't trigger
    // redundant logic execution
    m_isoSlider->blockSignals(true);
    m_isoSlider->setValue(m_currentIso);
    m_isoSlider->blockSignals(false);

    // 3. Safely swap graphics mapping pointers for the newly extracted VTK
    // polygon objects
    m_stlMapper->SetInputData(m_regVM->getStlData());
    m_dicomSurfaceMapper->SetInputData(m_regVM->getSurfaceData(m_currentIso));
    // m_dicomMapper

    // 4. Force optical realignment due to bounding box changes
    m_leftRenderer->ResetCamera();
    m_rightRenderer->ResetCamera();
    m_renderWindow->Render();
}

void MainWindow::onSliderChanged(int val)
{
    m_dicomSurfaceMapper->SetInputData(m_regVM->getSurfaceData(val));
    qDebug() << val;
    m_renderWindow->Render();
    // m_rightRenderer->Render();
}

void MainWindow::onAutoRegisterClicked(bool checked)
{
    if (checked) {
        m_leftRenderer->RemoveActor(m_stlActor);
        m_rightRenderer->AddActor(m_stlActor);

        // m_currentIso = -999
        // m_dicomSurfaceMapper->SetInputData(
        //     m_regVM->getSurfaceData(m_currentIso));

        QElapsedTimer timer;
        timer.start();
        vtkSmartPointer<vtkMatrix4x4> transformMatrix =
            m_regVM->performRegistration(m_currentIso);

        // --- STOP PROFILING ---
        qDebug() << "Execution Time:" << timer.elapsed() / 1000.0 << "s.";

        m_stlActor->SetUserMatrix(transformMatrix);

    } else {
        m_rightRenderer->RemoveActor(m_stlActor);
        m_leftRenderer->AddActor(m_stlActor);

        vtkNew<vtkMatrix4x4> identityMatrix;
        identityMatrix->Identity();
        m_stlActor->SetUserMatrix(identityMatrix); // undo registration
    }
    m_renderWindow->Render();
}
