#include "dataloader.h"

#include <vtkBox.h>
#include <vtkClipPolyData.h>
#include <vtkColorTransferFunction.h>
#include <vtkPolyDataConnectivityFilter.h>
#include <vtkStringArray.h>

#include <QDebug>
#include <QDir>
#include <QFile>

#include "vtkSmartPointer.h"

// QString testPatientStl = "SAIFI";
// QString testPatientCbct = "SAIFI/Man_mask";

// QString testPatientStl = "vijaya upper jaw";
// QString testPatientCbct = "vijaya/Max_mask";

// QString testPatientStl = "rajeev LowerJaw";
// QString testPatientCbct = "Rajeev 1 op/Man_mask";

// QString testPatientStl = "unknown";
// QString testPatientCbct = "unknown/Man_mask";

// QString testPatientStl = "ashish upper";
// QString testPatientCbct = "Ashish 1 op/Max_mask";

// QString stlFilePath =
//     "C:/Users/igrs/Desktop/Aswin/new_input_files/" + testPatientStl + ".stl";
// QString dicomFolderPath = "C:/Users/igrs/Desktop/Aswin/new_input_files/" +
//                           testPatientCbct;  // QString dicomFolderPath =
// "C:/Users/cdac/Projects/SE2dcm";

DataLoader::DataLoader(QObject *parent)
    : QObject{parent}
{
    m_opacityPiecewiseFunction = vtkSmartPointer<vtkPiecewiseFunction>::New();
    m_colorTransferFunction = vtkSmartPointer<vtkColorTransferFunction>::New();

    m_prop = vtkSmartPointer<vtkVolumeProperty>::New();
    m_isoProp = vtkSmartPointer<vtkProperty>::New();

    // 1. Turn on shading (Absolutely required for 3D depth)
    m_prop->ShadeOn();

    // 2. Ambient light (Keep low so the dark brown shadows we set earlier stay dark)
    m_prop->SetAmbient(0.15);

    // 3. Diffuse light (How much the main color shows up when hit by light)
    m_prop->SetDiffuse(0.80);

    // 4. Specular light (This creates the bright, glossy "shiny" reflections on the metal and bone edges)
    m_prop->SetSpecular(0.45);

    // 5. Specular Power (How sharp/pinpoint the shiny reflection is. Higher = glossier/more metallic)
    m_prop->SetSpecularPower(70.0);
    // m_prop->SetInterpolationTypeToNearest();

    m_isoProp->SetOpacity(1.0);
    m_isoProp->SetAmbient(0.1);
    m_isoProp->SetDiffuse(0.8);
    m_isoProp->SetSpecular(0.4);
    m_isoProp->SetSpecularPower(50.0);

    // loadStl(stlFilePath);
    // loadDicom(dicomFolderPath);
    loadTestingDataset(0);

    setupTransferFunctions();
}
struct TransferPoint
{
    double hu; // hounsfield units
    double opacity;
    double r, g, b;
};

// prevent multiple definition linker errors
inline const std::vector<TransferPoint> preset
    = {{-100, 0.00, 205, 153, 46}, // fat
       {-50, 0.2, 227, 211, 178},
       {10, 0.21, 222, 142, 142}, // muscle/ blood/ soft tissue
       {40, 0.4, 215, 109, 84},
       {150, 0.41, 254, 210, 161}, // spongy bone/ cartilage
       {300, 0.6, 219, 183, 127},
       {700, 0.61, 251, 236, 206}, // dense cortical bone
       {1500, 0.8, 167, 163, 152},
       {2000, 0.81, 230, 230, 232}, // solid metal
       {3000, 1.0, 111, 105, 107}};

void DataLoader::setupTransferFunctions()
{
    m_opacityPiecewiseFunction->RemoveAllPoints();
    m_colorTransferFunction->RemoveAllPoints();

    // --- DIAGNOSTIC MAPPING (BROAD RANGE) ---
    // If data is negative (-1000) or a mask (255), we make sure it forces visibility.
    // 0 or -1000 is still invisible (bg), but everything else shows up.
    // m_opacityPiecewiseFunction->AddPoint(-1500, 0.0); // True Air
    // m_opacityPiecewiseFunction->AddPoint(0, 0.0);     // Background/Water
    // m_opacityPiecewiseFunction->AddPoint(150, 0.8);   // Tissue / Mask Value Start
    // m_opacityPiecewiseFunction->AddPoint(1000, 0.9);  // Bone
    // m_opacityPiecewiseFunction->AddPoint(3000, 1.0);  // Metal
    // m_colorTransferFunction->AddRGBPoint(-1500, 0.0, 0.0, 0.0);
    // m_colorTransferFunction->AddRGBPoint(0, 0.0, 0.0, 0.0);
    // m_colorTransferFunction->AddRGBPoint(150, 0.85, 0.55, 0.55);  // Flesh tone
    // m_colorTransferFunction->AddRGBPoint(1000, 0.95, 0.85, 0.75); // Bone
    // m_colorTransferFunction->AddRGBPoint(3000, 1.0, 1.0, 1.0);    // Metal

    for (const auto &point : preset) {
        // double shiftedHU = point.hu + m_rangeStart;
        // qDebug() << shiftedHU << " : " << point.hu;

        m_opacityPiecewiseFunction->AddPoint(point.hu, point.opacity);
        m_colorTransferFunction->AddRGBPoint(point.hu,
                                             static_cast<double>(point.r) / 255.0,
                                             static_cast<double>(point.g) / 255.0,
                                             static_cast<double>(point.b) / 255.0);
    }

    m_prop->SetScalarOpacity(m_opacityPiecewiseFunction);
    m_prop->SetColor(m_colorTransferFunction);

    // isoSurface properties
    // m_isoProp->SetColor(255.0 / 255.0, 255.0 / 255.0, 255.0 / 255.0);
    m_isoProp->SetColor(1.0, 0.0, 0.0);
}
bool DataLoader::loadStl(QString &filePath)
{
    m_stlReader->SetFileName(filePath.toUtf8().constData());
    m_stlReader->Update();
    m_stlData = m_stlReader->GetOutput();
    if (m_stlData == nullptr || m_stlData->GetNumberOfPoints() == 0)
        return false;
    return true;
}

bool DataLoader::loadDicom(QString &folderPath)
{
    QDir dir(folderPath);
    dir.setNameFilters(QStringList() << "*.dcm" << "*.ima");
    dir.setFilter(QDir::Files | QDir::NoSymLinks);
    dir.setSorting(QDir::Name);

    QFileInfoList fileList = dir.entryInfoList();

    // if no files have extensions, grab everything
    if (fileList.isEmpty()) {
        dir.setNameFilters(QStringList());
        fileList = dir.entryInfoList();
    }
    if (fileList.isEmpty()) {
        qWarning() << "No files found in dir";
        return false;
    }

    qDebug() << "No of files: " << fileList.size();
    vtkNew<vtkStringArray> fileNames;
    for (const QFileInfo &fileInfo : fileList) {
        fileNames->InsertNextValue(fileInfo.absoluteFilePath().toUtf8().constData());
    }

    m_dicomReader->SetFileNames(fileNames);
    m_dicomReader->Update();
    // --- Teeth Isolation Guardrail ---
    m_thresholder = vtkSmartPointer<vtkImageThreshold>::New();
    m_thresholder->SetInputConnection(m_dicomReader->GetOutputPort());
    // Isolate dense structures. (Empirically adjust 1500 for your specific
    // CBCT calibration)
    m_thresholder->ThresholdBetween(1500.0, 6000.0);
    // We replace the outside tissue with -1000 (Air). We keep the inside
    // teeth
    // as their original HU values. This prevents severe "staircasing" in
    // Marching Cubes.
    m_thresholder->ReplaceOutOn();
    m_thresholder->SetOutValue(-1000);
    m_thresholder->Update();
    // ---------------------------------
    m_dicomData = m_thresholder->GetOutput();
    double range[2];
    m_dicomData->GetScalarRange(range);
    qDebug() << range[0] << " , " << range[1];

    // qDebug() << "File numbers: "
    // << fileNames->GetNumberOf

    if (m_dicomData == nullptr || m_dicomData->GetNumberOfPoints() == 0) {
        qDebug() << "Empty points returned!";
        return false;
    }
    // MANDATORY 3D Validation for vtkFlyingEdges3D
    int dims[3];
    m_dicomData->GetDimensions(dims);
    qDebug() << "DICOM Grid Dimensions: X=" << dims[0] << " Y=" << dims[1] << " Z=" << dims[2];
    if (dims[2] <= 1) {
        qWarning()
            << "CRITICAL: vtkFlyingEdges3D requires a 3D volume, but received 2D data (Z <= 1). "
            << "Check if multiple DICOM slices exist in the folder.";
        return false;
    }

    return true;
}

void DataLoader::loadTestingDataset(int index) {
    QString testPatientStl, testPatientCbct;
    switch (index) {
        case 0:
            testPatientStl = "SAIFI";
            testPatientCbct = "SAIFI/Mandible";
            break;
        case 1:
            testPatientStl = "vijaya upper jaw";
            testPatientCbct = "vijaya/Maxilla";
            break;
        case 2:
            testPatientStl = "rajeev LowerJaw";
            testPatientCbct = "Rajeev 1 op/Mandible";
            break;
        case 3:
            testPatientStl = "unknown";
            testPatientCbct = "unknown/Mandible";
            break;
        case 4:
            testPatientStl = "ashish upper";
            testPatientCbct = "Ashish 1 op/Maxilla";
            break;
        default:
            return;  // Fail fast pattern
    }

    QString stlFilePath =
        "C:/Users/igrs/Desktop/Aswin/vr_inputs/" + testPatientStl + ".stl";
    // QString dicomFolderPath =
    //     "C:/Users/igrs/Desktop/Aswin/new_input_files/" + testPatientCbct;
    QString dicomFolderPath =
        "C:/Users/igrs/Desktop/Aswin/vr_inputs/" + testPatientCbct;

    loadStl(stlFilePath);
    loadDicom(dicomFolderPath);
}

vtkSmartPointer<vtkPolyData> DataLoader::getStlData()
{
    return m_stlData;
}
vtkSmartPointer<vtkImageData> DataLoader::getDicomData()
{
    return m_dicomData;
}

vtkSmartPointer<vtkVolumeProperty> DataLoader::getVolProps()
{
    return m_prop;
}

vtkSmartPointer<vtkPolyData> DataLoader::getSurfaceData(double contourValue)
{
    // m_isoAlgo->SetInputConnection(m_dicomReader->GetOutputPort());
    m_isoAlgo->SetInputConnection(m_thresholder->GetOutputPort());
    m_isoAlgo->SetValue(0, contourValue);
    m_isoAlgo->Update();

    vtkNew<vtkPolyDataConnectivityFilter> connectivityFilter;
    connectivityFilter->SetInputConnection((m_isoAlgo->GetOutputPort()));
    connectivityFilter->SetExtractionModeToLargestRegion();


    // m_isoAlgo->GetOutput();
    // m_isoFilter->SetInputConnection(connectivityFilter->GetOutputPort());

    // // Higher iterations = more processing time but smoother results. 15-20 is a standard baseline.
    // m_isoFilter->SetNumberOfIterations(20);

    // // A lower passband value (e.g., 0.001 to 0.1) creates more smoothing (lets lower frequency structures through).
    // m_isoFilter->SetPassBand(0.01);

    // // Essential flags to prevent mesh distortion and calculation errors
    // m_isoFilter->BoundarySmoothingOff();
    // m_isoFilter->FeatureEdgeSmoothingOff();
    // m_isoFilter->NonManifoldSmoothingOn();
    // m_isoFilter->NormalizeCoordinatesOn();

    // Update the pipeline from the smoother, not the isoAlgo
    // m_isoFilter->Update();
    // return m_isoFilter->GetOutput();

    m_normalsCalc->SetInputConnection(m_isoAlgo->GetOutputPort());
    m_normalsCalc->SetFeatureAngle(
        60.0); // Smooths angles below 60 degrees, preserves sharp structural edges above
    m_normalsCalc->ComputePointNormalsOn();
    m_normalsCalc->ComputeCellNormalsOff();
    m_normalsCalc->ConsistencyOn(); // Enforces identical ordering for consistent outer lighting
    m_normalsCalc->SplittingOff();

    m_normalsCalc->Update();
    return m_normalsCalc->GetOutput();
    // return m_isoAlgo->GetOutput();
}

vtkSmartPointer<vtkProperty> DataLoader::getSurfaceProps()
{
    return m_isoProp;
}
