QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets


include(./lib_config.pri)

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    dataloader.cpp \
    main.cpp \
    mainwindow.cpp \
    registerviewmodel.cpp \
    registration/dentalregistrationengine.cpp \
    registration/registrationpointevaluator.cpp \
    registration/vtkimagesurfaceextractor.cpp \
    registration/registrationmodel.cpp

HEADERS += \
    dataloader.h \
    mainwindow.h \
    registerviewmodel.h \
    registration/RegistrationTypes.h \
    registration/dentalregistrationengine.h \
    registration/registrationpointevaluator.h \
    registration/vtkimagesurfaceextractor.h \
    registration/registrationmodel.h

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

RESOURCES += \
    resources/resources.qrc

DISTFILES += \
    lib_config.pri



CONFIG += c++17
# Disable strict C++ compliance to allow PCL 1.11.1 legacy template configurations to compile
win32: QMAKE_CXXFLAGS += /permissive

