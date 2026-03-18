#include "dataloader.h"
#include <QDebug>
#include <QDir>
#include <QFile>
#include "vtkSmartPointer.h"
#include <vtkStringArray.h>

QString stlFilePath = "C:/Users/cdac/Official-projects/Input-files/SAIFI.stl";

QString dicomFolderPath = "C:/Users/cdac/Official-projects/Input-files/Man_Mask";
// QString dicomFolderPath = "C:/Users/cdac/Projects/SE2dcm";

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

    loadStl(stlFilePath);
    loadDicom(dicomFolderPath);

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
    m_isoProp->SetColor(255.0 / 255.0, 255.0 / 255.0, 255.0 / 255.0);
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
    /*
    m_dicomDir->SetDirectoryName(folderPath.toUtf8().constData());
    m_dicomDir->Update();
    if (m_dicomDir->GetNumberOfSeries() == 0) {
        qDebug() << "No dicom series found in the directory ";
        return false;
    } else {
        qDebug() << "Series found!, no: " << m_dicomDir->GetNumberOfSeries();
    }
    vtkStringArray *fileNames = m_dicomDir->GetFileNamesForSeries(2);

    if (!fileNames || fileNames->GetNumberOfValues() == 0) {
        qWarning() << "Series 0 alloted but 0 files!";
        return false;
    } else {
        qDebug() << "file names no: " << fileNames->GetNumberOfValues();
    }

    qDebug() << "first file path" << fileNames->GetValue(0).c_str();
    
*/

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
    m_dicomData = m_dicomReader->GetOutput();

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
    m_isoAlgo->SetInputConnection(m_dicomReader->GetOutputPort());
    m_isoAlgo->SetValue(0, contourValue);
    m_isoAlgo->Update();
    // m_isoAlgo->GetOutput();
    m_isoFilter->SetInputConnection(m_isoAlgo->GetOutputPort());

    // Higher iterations = more processing time but smoother results. 15-20 is a standard baseline.
    m_isoFilter->SetNumberOfIterations(20);

    // A lower passband value (e.g., 0.001 to 0.1) creates more smoothing (lets lower frequency structures through).
    m_isoFilter->SetPassBand(0.005);

    // Essential flags to prevent mesh distortion and calculation errors
    m_isoFilter->BoundarySmoothingOff();
    m_isoFilter->FeatureEdgeSmoothingOff();
    m_isoFilter->NonManifoldSmoothingOn();
    m_isoFilter->NormalizeCoordinatesOn();

    // Update the pipeline from the smoother, not the isoAlgo
    // m_isoFilter->Update();
    // return m_isoFilter->GetOutput();

    m_normalsCalc->SetInputConnection(m_isoFilter->GetOutputPort());
    m_normalsCalc->SetFeatureAngle(
        60.0); // Smooths angles below 60 degrees, preserves sharp structural edges above
    m_normalsCalc->ComputePointNormalsOn();
    m_normalsCalc->ComputeCellNormalsOff();
    m_normalsCalc->ConsistencyOn(); // Enforces identical ordering for consistent outer lighting
    m_normalsCalc->SplittingOff();

    m_normalsCalc->Update();
    return m_normalsCalc->GetOutput();
}

vtkSmartPointer<vtkProperty> DataLoader::getSurfaceProps()
{
    return m_isoProp;
}
