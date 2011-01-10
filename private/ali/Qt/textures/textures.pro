HEADERS = glwidget.h \
    window.h \
    EulerAngles.hpp
SOURCES = glwidget.cpp \
    main.cpp \
    window.cpp \
    EulerAngles.cpp
RESOURCES = textures.qrc
QT += opengl

# install
target.path = $$[QT_INSTALL_EXAMPLES]/opengl/textures
sources.files = $$SOURCES \
    $$HEADERS \
    $$RESOURCES \
    $$FORMS \
    textures.pro \
    images
sources.path = $$[QT_INSTALL_EXAMPLES]/opengl/textures
INSTALLS += target \
    sources
symbian:include($$QT_SOURCE_TREE/examples/symbianpkgrules.pri)
