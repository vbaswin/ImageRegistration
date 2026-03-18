#include "dataloader.h"
#include <QDebug>
#include "vtkSmartPointer.h"
#include <vtkStringArray.h>

DataLoader::DataLoader(QObject *parent)
    : QObject{parent}
{
    m_opacityPiecewiseFunction = vtkSmartPointer<vtkPiecewiseFunction>::New();
    m_colorTransferFunction = vtkSmartPointer<vtkColorTransferFunction>::New();

    m_prop = vtkSmartPointer<vtkVolumeProperty>::New();

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
    m_dicomDir->SetDirectoryName(folderPath.toUtf8().constData());
    m_dicomDir->Update();
    if (m_dicomDir->GetNumberOfSeries() == 0) {
        std::cerr << "No dicom series found in the directory ";
        return false;
    }
    vtkStringArray *fileNames = m_dicomDir->GetFileNamesForSeries(0);

    // if (!fileNames || )

    m_dicomReader->SetFileNames(fileNames);
    m_dicomReader->Update();
    m_dicomData = m_dicomReader->GetOutput();

    double range[2];
    m_dicomData->GetScalarRange(range);
    qDebug() << range[0] << " , " << range[1];

    // qDebug() << "File numbers: "
    // << fileNames->GetNumberOf

    if (m_dicomData == nullptr || m_dicomData->GetNumberOfPoints() == 0)
        return false;
    return true;
}
vtkSmartPointer<vtkPolyData> DataLoader::getStlData()
{
    QString filePath = "C:/Users/cdac/Official-projects/Input-files/SAIFI.stl";
    loadStl(filePath);
    return m_stlData;
}
vtkSmartPointer<vtkImageData> DataLoader::getDicomData()
{
    QString folderPath = "C:/Users/cdac/Official-projects/Input-files/Man_Mask";
    // QString folderPath = "C:/Users/cdac/Projects/SE2dcm";
    loadDicom(folderPath);
    return m_dicomData;
}

vtkSmartPointer<vtkVolumeProperty> DataLoader::getVolProps()
{
    return m_prop;
}
