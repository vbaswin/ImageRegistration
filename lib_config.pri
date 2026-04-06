
QT       += core gui widgets opengl
greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++17
# PRECOMPILED_HEADER = precomp.h

# =============================================================================
# "Release With Debug Info" — the correct strategy when linking against
# pre-built Release third-party libraries (VTK, vtkDICOM).
#
# WHY: Mixing Debug Qt DLLs (your app) with Release Qt DLLs (inside VTK)
# creates two separate qApp globals in one process — VTK sees null and crashes.
# This config uses the SAME Release CRT as VTK while retaining full debuggability
# of YOUR source code.
#
# /Od  = No optimization  → source lines map 1:1 to machine code → step works
# /Zi  = Full debug info  → breakpoints, variable inspection work
# /MD  = Release CRT      → same allocator/runtime as VTK DLLs (no mismatch)
# =============================================================================
CONFIG += release force_debug_info
# QMAKE_CXXFLAGS_RELEASE -= /O2 /O1          # Strip default release optimizations
# QMAKE_CXXFLAGS_RELEASE += /Od /Zi          # No-opt + full debug symbols
# QMAKE_LFLAGS_RELEASE    += /DEBUG          # Linker emits .pdb for the executable
# Strip optimizations from BOTH flag sets that force_debug_info uses
# QMAKE_CXXFLAGS_RELEASE                  -= /O2 /O1
QMAKE_CXXFLAGS_RELEASE_WITH_DEBUGINFO   -= /O2 /O1
# QMAKE_CXXFLAGS_RELEASE                  += /Od
QMAKE_CXXFLAGS_RELEASE_WITH_DEBUGINFO   += /Od
QMAKE_LFLAGS_RELEASE                    += /DEBUG

DEFINES	+= QT_DEPRECATED_WARNINGS
VTK_INSTALL =C:/Users/igrs/Desktop/Aswin/ext_libs/VTK8.2/Release

INCLUDEPATH += $$VTK_INSTALL/include/vtk-8.2
DEPENDPATH  += $$VTK_INSTALL/include/vtk-8.2
QMAKE_LIBDIR += $$VTK_INSTALL/lib

LIBS += \
-lvtkCommonCore-8.2 \
-lvtkCommonDataModel-8.2 \
-lvtkCommonExecutionModel-8.2 \
-lvtkCommonMath-8.2 \
-lvtkCommonTransforms-8.2 \
-lvtkCommonMisc-8.2 \
-lvtkRenderingCore-8.2 \
-lvtkRenderingImage-8.2 \
-lvtkRenderingAnnotation-8.2 \
-lvtkRenderingOpenGL2-8.2 \
-lvtkRenderingFreeType-8.2 \
-lvtkInteractionStyle-8.2 \
-lvtkInteractionImage-8.2 \
-lvtkGUISupportQt-8.2 \
-lvtkFiltersSources-8.2 \
-lvtkIOImage-8.2 \
-lvtkIOCore-8.2 \
-lvtkImagingCore-8.2 \
-lvtkImagingColor-8.2 \
-lvtkInteractionWidgets-8.2 \
-lvtkFiltersCore-8.2 \
-lvtkRenderingVolume-8.2 \
-lvtkRenderingVolumeOpenGL2-8.2 \
-lvtkImagingHybrid-8.2 \
-lvtkIOGeometry-8.2 \
-lvtkFiltersGeneral-8.2 \
-lvtkFiltersHybrid-8.2 \
-ladvapi32

# --- vtkDICOM 0.8.13 (MSVC 2019-compatible x64 build) ---
VTKDICOM_INSTALL = C:/Users/igrs/Desktop/Aswin/ext_libs/VTKDicom-9.5.2/Release

INCLUDEPATH += $$VTKDICOM_INSTALL/include
QMAKE_LIBDIR += $$VTKDICOM_INSTALL/lib

LIBS += -lvtkDICOM-8.2.0


# PCL integration
PCL_INSTALL = "c:/Program Files/PCL 1.11.1"

INCLUDEPATH += "$$PCL_INSTALL/include/pcl-1.11"
INCLUDEPATH += "$$PCL_INSTALL/3rdParty/Boost/include/boost-1_74"  # Check your exact boost version!
INCLUDEPATH += "$$PCL_INSTALL/3rdParty/Eigen/eigen3"
INCLUDEPATH += "$$PCL_INSTALL/3rdParty/FLANN/include"
INCLUDEPATH += "$$PCL_INSTALL/3rdParty/FLANN/include"


# 2. Linker Directories
QMAKE_LIBDIR += "$$PCL_INSTALL/lib"
QMAKE_LIBDIR += "$$PCL_INSTALL/3rdParty/Boost/lib"
QMAKE_LIBDIR += "$$PCL_INSTALL/3rdParty/FLANN/lib"

LIBS +=     \
    -lpcl_common \
    -lpcl_kdtree \
    -lpcl_search \
    -lpcl_features \
    -lpcl_filters \
    -lpcl_registration \
    -lpcl_io \
    -lpcl_surface \
    -lflann_cpp_s

LIBS += -lpcl_common \
                -lpcl_search \
                -lpcl_kdtree \
                -lpcl_segmentation

# ***** forces to use 32  bit alignment but pre-built pcl uses 16 bits
# cauisng crashes
# QMAKE_CXXFLAGS += /arch:AVX2

# -----------------------------------------------------------------------------
# --- ITK 5.4 Integration (Volumetric Distance-Field Registration) ---
# -----------------------------------------------------------------------------
# FIXED PATH: Pointing exactly to the Release directory where the binaries exist
ITK_INSTALL = C:/Users/igrs/Desktop/Aswin/ext_libs/ITK-5.4.5/Release

INCLUDEPATH += $$ITK_INSTALL/include/ITK-5.4
DEPENDPATH  += $$ITK_INSTALL/include/ITK-5.4
QMAKE_LIBDIR += $$ITK_INSTALL/lib

# ITK 5.4 Base Engine
LIBS += \
-lITKCommon-5.4 \
-litksys-5.4 \
-lITKVNLInstantiation-5.4 \
-litkvnl-5.4 \
-litkvnl_algo-5.4 \
-litkv3p_netlib-5.4 \
-litkNetlibSlatec-5.4 \
-lITKSmoothing-5.4 \
-lITKStatistics-5.4



# ITK 5.4 Volumetric Registration Core
LIBS += \
-lITKRegistrationMethodsv4-5.4 \
-lITKOptimizersv4-5.4 \
-lITKTransform-5.4 \
-lITKTransformFactory-5.4 \
-lITKSpatialObjects-5.4

# ITK 5.4 Distance Maps and VTK Memory Bridge
LIBS += \
-lITKVTK-5.4 \
-lITKVtkGlue-5.4


