#-------------------------------------------------
#
# Project created by QtCreator 2016-01-05T22:44:02
#
#-------------------------------------------------

TARGET = mtl
TEMPLATE = lib
CONFIG += staticlib c++11
CONFIG -= qt

DEFINES += USE_BOOST_THREAD_LIB STD_COUT _WIN32_WINNT=0x0601

android {
    INCLUDEPATH += $$NDK_ROOT/sources/crystax/include \
                   $$NDK_ROOT/sources/boost/1.59.0/include
    LIBS += -L"$$NDK_ROOT/sources/crystax/libs/$$ANDROID_TARGET_ARCH"
#    ANDROID_EXTRA_LIBS = $$NDK_ROOT/sources/crystax/libs/$$ANDROID_TARGET_ARCH/libcrystax.so \
#            $$NDK_ROOT/sources/boost/1.59.0/libs/$$ANDROID_TARGET_ARCH/gnu-4.9/libboost_system.so
#    ANDROID_PACKAGE_SOURCE_DIR = $$PWD/android
}

win32 {
    INCLUDEPATH += C:/boost/boost_1_71_0
    LIBS += LC:/boost/boost_1_71_0/stage/lib
}

SOURCES += \
    mtl/core/detail/impl/dllmain.cpp \
    mtl/network/udp/detail/impl/dgram_receive_group_task.cpp \
    mtl/network/udp/detail/impl/dgram_send_group_task.cpp \
    mtl/task/detail/impl/task_group.cpp \
    mtl/encrypt/detail/impl/rc4.cpp \
    mtl/network/detail/impl/byte_stream.cpp \
    mtl/network/detail/impl/in_stream.cpp \
    mtl/network/detail/impl/in_request.cpp \
    mtl/network/detail/impl/out_stream.cpp \
    mtl/network/p2p/detail/impl/p2p_client.cpp \
    mtl/network/p2p/detail/impl/p2p_group_packet_filter.cpp \
    mtl/network/p2p/detail/impl/p2p_server.cpp \
    mtl/network/udp/detail/impl/dgram.cpp \
    mtl/network/detail/impl/floating_buffer_pool.cpp \
    mtl/network/detail/impl/context_pool.cpp \
    mtl/network/detail/impl/out_request.cpp \
    mtl/network/detail/impl/singleton_buffer_pool.cpp \
    mtl/network/tcp/detail/impl/tcp_client.cpp \
    mtl/network/tcp/detail/impl/tcp_connection.cpp \
    mtl/network/tcp/detail/impl/tcp_server.cpp \
    mtl/datetime/detail/impl/date_time.cpp \
    mtl/framework/log/detail/impl/log.cpp \
    mtl/framework/core/detail/impl/connection_controller.cpp \
    mtl/framework/core/detail/impl/heart_beat_task.cpp \
    mtl/framework/core/detail/impl/connect_task.cpp \
    mtl/utility/detail/impl/util.cpp \
    mtl/task/detail/impl/task.cpp \
    mtl/network/p2p/detail/impl/p2p_broadcast_task.cpp \
    mtl/network/p2p/detail/impl/p2p_graph.cpp \
    mtl/network/udp/detail/impl/dgram_packet_record.cpp

HEADERS += \
    mtl/mtl.hpp \
    mtl/encrypt/rc4.hpp \
    mtl/network/udp/detail/dgram_receive_group_task.hpp \
    mtl/network/udp/detail/dgram_send_group_task.hpp \
    mtl/singleton/singleton.hpp \
    mtl/task/task.hpp \
    mtl/task/task_group.hpp \
    mtl/network/in_request.hpp \
    mtl/network/in_stream.hpp \
    mtl/network/out_stream.hpp \
    mtl/network/context_pool.hpp \
    mtl/network/singleton_buffer_pool.hpp \
    mtl/network/shared_buffer.hpp \
    mtl/network/detail/byte_order.hpp \
    mtl/network/detail/byte_stream.hpp \
    mtl/network/detail/floating_buffer_pool.hpp \
    mtl/network/p2p/client.hpp \
    mtl/network/p2p/server.hpp \
    mtl/network/p2p/detail/graph.hpp \
    mtl/network/p2p/detail/graph_relationships.hpp \
    mtl/network/p2p/detail/p2p_group_packet_filter.hpp \
    mtl/network/udp/dgram.hpp \
    mtl/network/tcp/client.hpp \
    mtl/network/tcp/connection.hpp \
    mtl/network/tcp/server.hpp \
    mtl/datetime/date_time.hpp \
    mtl/network/out_request.hpp \
    mtl/framework/log/log.h \
    mtl/framework/core/connection.h \
    mtl/framework/core/connection_controller.h \
    mtl/framework/core/connection_controller_factory.h \
    mtl/framework/core/framework.h \
    mtl/framework/core/heart_beat_task.h \
    mtl/framework/core/host_address.h \
    mtl/framework/core/logic_unit.h \
    mtl/framework/core/logic_unit_factory.h \
    mtl/framework/core/singleton_tcp_server.h \
    mtl/framework/core/tcp_client.h \
    mtl/framework/core/tcp_server.h \
    mtl/framework/core/tcp_server_work.h \
    mtl/framework/core/connect_task.h \
    mtl/framework/core/udp_server.h \
    mtl/network/p2p/detail/p2p_broadcast_task.hpp \
    mtl/network/p2p/protocol.hpp \
    mtl/network/udp/protocol.hpp \
    mtl/network/protocol.hpp \
    mtl/network/udp/detail/dgram_packet_record.hpp \
    mtl/utility/util.hpp

unix {
    target.path = /usr/lib
    INSTALLS += target
}
