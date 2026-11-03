#include <libgeneral/macros.h>
#include "notification.h"
#include "Client.hpp"
#include "MUXException.hpp"


#ifdef HAVE_DLFCN_H
#include <dlfcn.h>
#else
#define WITHOUT_DESKTOP_NOTIFICATION
#endif

#include <sys/socket.h>
#include <sys/un.h>
#include <string.h>
#include <unistd.h>

#pragma mark private
static int didTryInit = 0;

bool (*my_notify_init)(const char *app_name) = NULL;
void *(*my_notify_notification_new)(const char *title, const char *message, void *isNull) = NULL;
void *(*my_notify_notification_show)(void *notification, void *isNull) = NULL;
bool (*my_notify_notification_close)(void *notification, void *isNULL) = NULL;

extern const char *usbmuxd_socket_path;


extern "C" int notification_init(){
    didTryInit = 1;
#ifdef HAVE_DLFCN_H
    void *handle = dlopen("/usr/lib/x86_64-linux-gnu/libnotify.so.4", RTLD_NOW);
    if (!handle) return -1;

    my_notify_init = (bool(*)(const char *))dlsym(handle, "notify_init");
    if (!my_notify_init) return -2;

    if (!my_notify_init("usbmuxd2")) return -3;

    my_notify_notification_close = (bool(*)(void*, void*))dlsym(handle, "notify_notification_close");
    if (!my_notify_notification_close) return -4;

    my_notify_notification_new = (void*(*)(const char*, const char*, void*))dlsym(handle, "notify_notification_new");
    if (!my_notify_notification_new) return -5;

    my_notify_notification_show = (void*(*)(void*, void*))dlsym(handle, "notify_notification_show");
    if (!my_notify_notification_show) return -6;

    return 0;
#else
    return -99;
#endif
}

#ifndef WITHOUT_DESKTOP_NOTIFICATION
static int connect_usbmuxd(void){
    int sfd = -1;

    sfd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (sfd == -1) return -1;

    struct sockaddr_un spath = {
        AF_UNIX
    };
    size_t plen = strlen(usbmuxd_socket_path);
    memcpy(spath.sun_path, usbmuxd_socket_path, plen);

    if( connect(sfd, (struct sockaddr*)&spath, (socklen_t)(sizeof(spath.sun_family) + plen)) == -1 ){
        safeClose(sfd);
    }

    return sfd;
}
#endif

#pragma mark public
extern "C" void notification_send(const char *title, const char *message){
#ifndef WITHOUT_DESKTOP_NOTIFICATION
    if (!didTryInit) notification_init();
    if (my_notify_notification_new && my_notify_notification_show){
        static void *lastNotification = NULL;
        if (lastNotification){
            my_notify_notification_close(lastNotification, NULL);
        }
        my_notify_notification_show(lastNotification = my_notify_notification_new(title, message, NULL), NULL);
    }
#endif
}


extern "C" int desktop_notification_dameon(){
#ifndef WITHOUT_DESKTOP_NOTIFICATION
    if (notification_init()){
        info("Failed to init notifications. Nothing to do here");
        return 0;
    }

    info("Running notification daemon");
    while (1){
        int sfd = -1;
        sleep(1);
        sfd = connect_usbmuxd();
        if (sfd == -1) continue;
        Client cli(nullptr, nullptr, sfd, 0);
        try {
            cli.registerForDesktopNotifications();
        }catch (...){
            continue;
        }
        try {
            cli.runLoopBlocking();
        }catch (tihmstar::MUXException_unsupported &e){
            info("Notifications not suppored with the currently running daemon");
            break;
        }
    }

#endif    
    return 0;
}
