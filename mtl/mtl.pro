#-------------------------------------------------
#
# Project created by QtCreator 2016-01-05T22:44:02
#
#-------------------------------------------------

TARGET = mtl
TEMPLATE = lib
CONFIG += c++11
CONFIG -= qt

DEFINES += MTL_EXPORT USE_BOOST_THREAD_LIB STD_COUT

INCLUDEPATH += $$PWD

android {
    INCLUDEPATH += $$NDK_ROOT/sources/crystax/include \
                   $$NDK_ROOT/sources/boost/1.59.0/include
    LIBS += -L"$$NDK_ROOT/sources/crystax/libs/$$ANDROID_TARGET_ARCH"
#    ANDROID_EXTRA_LIBS = $$NDK_ROOT/sources/crystax/libs/$$ANDROID_TARGET_ARCH/libcrystax.so \
#            $$NDK_ROOT/sources/boost/1.59.0/libs/$$ANDROID_TARGET_ARCH/gnu-4.9/libboost_system.so
#    ANDROID_PACKAGE_SOURCE_DIR = $$PWD/android
}

win32 {
    INCLUDEPATH += C:/boost/boost_1_61_0
    LIBS += -LC:/boost/boost_1_61_0/stage/lib
}

SOURCES += \
    mtl/core/detail/impl/dllmain.cpp \
    mtl/task/detail/impl/task.cpp \
    mtl/task/detail/impl/task_group.cpp \
    mtl/network/detail/impl/byte_stream.cpp \
    mtl/network/detail/impl/in_stream.cpp \
    mtl/network/detail/impl/in_request.cpp \
    mtl/network/detail/impl/out_stream.cpp \
    mtl/network/detail/impl/out_request.cpp \
    mtl/network/detail/impl/singleton_buffer_pool.cpp \
    mtl/network/detail/impl/unfixable_buffer_pool.cpp \
    mtl/network/detail/impl/io_service_pool.cpp \
    mtl/network/p2p/detail/impl/p2p_client.cpp \
    mtl/network/p2p/detail/impl/p2p_group_packet_filter.cpp \
    mtl/network/p2p/detail/impl/p2p_server.cpp \
    mtl/network/p2p/detail/impl/p2p_broadcast_task.cpp \
    mtl/network/p2p/detail/impl/p2p_graph.cpp \
    mtl/network/udp/detail/impl/dgram_packet_record.cpp \
    mtl/network/udp/detail/impl/dgram.cpp \
    mtl/network/udp/detail/impl/dgram_group_recv_task.cpp \
    mtl/network/udp/detail/impl/dgram_group_send_task.cpp \
    mtl/network/tcp/detail/impl/tcp_client.cpp \
    mtl/network/tcp/detail/impl/tcp_connection.cpp \
    mtl/network/tcp/detail/impl/tcp_server.cpp

HEADERS += \
    mtl/mtl.hpp \
    mtl/singleton/singleton.hpp \
    mtl/task/task.hpp \
    mtl/task/task_group.hpp \
    mtl/network/protocol.hpp \
    mtl/network/in_request.hpp \
    mtl/network/out_request.hpp \
    mtl/network/in_stream.hpp \
    mtl/network/out_stream.hpp \
    mtl/network/detail/unfixable_buffer_pool.hpp \
    mtl/network/singleton_buffer_pool.hpp \
    mtl/network/io_service_pool.hpp \
    mtl/network/shared_buffer.hpp \
    mtl/network/detail/byte_order.hpp \
    mtl/network/detail/byte_stream.hpp \
    mtl/network/p2p/protocol.hpp \
    mtl/network/p2p/client.hpp \
    mtl/network/p2p/server.hpp \
    mtl/network/p2p/detail/graph.hpp \
    mtl/network/p2p/detail/graph_relationships.hpp \
    mtl/network/p2p/detail/p2p_group_packet_filter.hpp \
    mtl/network/p2p/detail/p2p_broadcast_task.hpp \
    mtl/network/udp/detail/dgram_packet_record.hpp \
    mtl/network/udp/protocol.hpp \
    mtl/network/udp/dgram.hpp \
    mtl/network/udp/detail/dgram_group_recv_task.hpp \
    mtl/network/udp/detail/dgram_group_send_task.hpp \
    mtl/network/tcp/client.hpp \
    mtl/network/tcp/connection.hpp \
    mtl/network/tcp/server.hpp

unix {
    target.path = /usr/lib
    INSTALLS += target
}
