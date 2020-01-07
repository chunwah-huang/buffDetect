TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle

SOURCES += main.cpp \
#    buff_detect.cpp \
#    solve_pnp.cpp \
    detect_buff/buff_detect.cpp \
    solve_buff/solve_pnp.cpp

INCLUDEPATH += /usr/local/include \
                /usr/local/include/opencv
                /usr/local/include/opencv2

LIBS += /usr/local/lib/libopencv_* \
        /home/hzh/camera_rcx/linuxSDK/lib/x64/libMVSDK.so

HEADERS += \
#    buff_detect.h \
#    solve_pnp.h \
    detect_buff/buff_detect.h \
    solve_buff/solve_pnp.h \
    base.h
