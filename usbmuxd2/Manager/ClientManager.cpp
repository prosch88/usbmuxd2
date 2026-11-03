//
//  ClientManager.cpp
//  usbmuxd2
//
//  Created by tihmstar on 20.07.23.
//
#include <libgeneral/macros.h>

#include "ClientManager.hpp"
#include "MUXException.hpp"
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/stat.h>
#include <unistd.h>
#include "Client.hpp"
#include <memory>
#include <poll.h>
#include <signal.h>

#ifdef __APPLE__
#include <libproc.h>
#endif

#ifdef SOCKET_PATH
const char *usbmuxd_socket_path = SOCKET_PATH;
#else
const char *usbmuxd_socket_path = "/var/run/usbmuxd";
#endif

#ifdef __APPLE__
static void silenceOtherInstances(){
    pid_t *pids = NULL;
    cleanup([&]{
        safeFree(pids);
    });
    int num_pids = 0;

    num_pids = proc_listallpids(NULL, 0);
    pids = (pid_t *)calloc(num_pids, sizeof(pid_t));
    num_pids = proc_listallpids(pids, num_pids * sizeof(pids));
    
    pid_t mypid = getpid();
    uid_t myuid = getuid();

    for (int i = 0; i < num_pids; i++) {
        struct proc_taskallinfo pidinfo = {};
        proc_pidinfo(pids[i], PROC_PIDTASKALLINFO, 0,  &pidinfo, sizeof(pidinfo));

        if (pids[i] == mypid) continue; //don't kill ourselves

        if (pidinfo.pbsd.pbi_uid == myuid) continue; //apple usbmuxd runs as '_usbmuxd' user, as which we will never run (under normal circumstances)
        
        if (strcmp("usbmuxd", pidinfo.pbsd.pbi_comm) == 0) {
            int err = kill(pids[i], SIGSTOP);
            warning("Found other usbmuxd process with pid=%d, stopping %s",pids[i], err == 0 ? "OK" : "FAIL");
            warning("\n"
                    "***************************\n"
                    "*         WARNING         *\n"
                    "* Running usbmuxd2 breaks *\n"
                    "*      Apple usbmuxd      *\n"
                    "*    until next reboot    *\n"
                    "***************************\n"
                    "");
        }
    }
}

std::string pocNameForPid(pid_t pid){
    int err = 0;
    char name[0x100] = {};
    retassure((err = proc_name(pid,name, sizeof(name))) > 0, "Failed to get name for pid %d with err=%d",pid,err);
    return name;
}

static void killOtherInstances(){
    pid_t *pids = NULL;
    cleanup([&]{
        safeFree(pids);
    });
    int num_pids = 0;

    num_pids = proc_listallpids(NULL, 0);
    pids = (pid_t *)calloc(num_pids, sizeof(pid_t));
    num_pids = proc_listallpids(pids, num_pids * sizeof(pids));
    
    pid_t mypid = getpid();
    uid_t myuid = getuid();

    for (int i = 0; i < num_pids; i++) {
        struct proc_taskallinfo pidinfo = {};
        proc_pidinfo(pids[i], PROC_PIDTASKALLINFO, 0,  &pidinfo, sizeof(pidinfo));

        if (pids[i] == mypid) continue; //don't kill ourselves

        if (pidinfo.pbsd.pbi_uid == myuid) continue; //apple usbmuxd runs as '_usbmuxd' user, as which we will never run (under normal circumstances)
        
        if (strcmp("usbmuxd", pidinfo.pbsd.pbi_comm) == 0) {
            int err = kill(pids[i], SIGKILL);
            warning("Found other usbmuxd process with pid=%d, killing %s",pids[i], err == 0 ? "OK" : "FAIL");
        }
    }
}
#else
std::string pocNameForPid(pid_t pid){
    reterror("Not implemented");
}
#endif

#pragma mark ClientManager
ClientManager::ClientManager(Muxer *mux, const char *exclusiveClientName)
: _mux(mux)
, _clientNumber(0), _listenfd(-1)
,_wakePipe{}
{
    struct sockaddr_un bind_addr = {};
    if (exclusiveClientName){
        debug("ClientManager only accepting process named '%s'",exclusiveClientName);
        _exclusiveClientName = exclusiveClientName;
    }

#ifdef __APPLE__
    silenceOtherInstances();
#endif

    retassure(unlink(usbmuxd_socket_path) != 1 || errno == ENOENT, "unlink(%s) failed: %s", usbmuxd_socket_path, strerror(errno));
    
    retassure((_listenfd = socket(AF_UNIX, SOCK_STREAM, 0))>=0, "socket() failed: %s", strerror(errno));
    
    bind_addr.sun_family = AF_UNIX;
    strcpy(bind_addr.sun_path, usbmuxd_socket_path);
    retassure(!bind(_listenfd, (struct sockaddr*)&bind_addr, sizeof(bind_addr)), "bind() failed: %s", strerror(errno));
    
    retassure(!listen(_listenfd, 5), "listen() failed: %s", strerror(errno));
    
    assure(!chmod(usbmuxd_socket_path, 0666));
    
    assure(!pipe(_wakePipe));
    
    _cliReaperThread = std::thread([this]{
        reaper_runloop();
    });
}

ClientManager::~ClientManager(){
    info("[destroying] ClientManager");
    stopLoop();
    
    safeClose(_wakePipe[0]);
    safeClose(_wakePipe[1]);

    if (_children.size()) {
        debug("waiting for client children to die...");
        std::unique_lock<std::mutex> ul(_childrenLck);
        while (size_t s = _children.size()) {
            for (auto c : _children) c->kill();
            uint64_t wevent = _childrenEvent.getNextEvent();
            ul.unlock();
            debug("Need to kill %zu more client children",s);
            _childrenEvent.waitForEvent(wevent);
            ul.lock();
        }
    }
    _reapClients.kill();
    _cliReaperThread.join();

    if (_listenfd > 0) {
        int cfd = _listenfd; _listenfd = -1;
        close(cfd);
    }
#ifdef __APPLE__
    killOtherInstances();
#endif
}

void ClientManager::stopAction() noexcept{
    if (_listenfd) shutdown(_listenfd, SHUT_RDWR);
    safeClose(_wakePipe[1]);
}

bool ClientManager::loopEvent(){
    int cfd = -1;
    cleanup([&]{
        safeClose(cfd);
    });
    
    cfd = accept_client();
    if (!shouldServeClient(cfd)) return true;

    if (cfd == -1) return true;
    try {
        int c = cfd; cfd = -1;
        handle_client(c); //always consumes cfd
    } catch (tihmstar::exception &e) {
        error("failed to handle client %d with error=%d",cfd,e.code());
    }
    return true;
}

void ClientManager::afterLoop() noexcept{
    if (_listenfd > 0) {
        int cfd = _listenfd; _listenfd = -1;
        close(cfd);
    }
}

void ClientManager::reaper_runloop(){
    while (true) {
        std::shared_ptr<Client> cli;
        try {
            cli = _reapClients.wait();
        } catch (...) {
            break;
        }
        //make device go out of scope so it can die in piece
        cli->deconstruct();
    }
}

int ClientManager::accept_client(){
    struct sockaddr_un addr = {};
    int err = 0;
    int cfd = 0;
    socklen_t len = 0;
    len = sizeof(struct sockaddr_un);
    struct pollfd pfd[2] = {
        {
            .fd = _listenfd,
            .events = POLLIN
        },
        {
            .fd = _wakePipe[0],
            .events = POLLIN
        }
    };
    if ((err = poll(pfd,2,-1)) == -1){
        retassure(errno == EINTR, "[CLIENTMANAGER] poll failed errno=%d (%s)",errno,strerror(errno));
        return -1;
    }
    retcustomassure(MUXException_graceful_kill,!(pfd[1].revents & POLLHUP), "graceful kill requested");
    retassure(pfd[0].revents & POLLIN, "poll returned, but there is no POLLIN event on client");
    retassure((cfd = accept(_listenfd, (struct sockaddr *)&addr, &len))>=0, "accept() failed (%s)", strerror(errno));
    
    return cfd;
}

bool ClientManager::shouldServeClient(int client_fd){
    if (!_exclusiveClientName.size()) return true;
    int err = 0;
    pid_t client_pid = 0;
    socklen_t len = sizeof(client_pid);
    retassure(!(err = getsockopt(client_fd, 0, 2, &client_pid, &len)), "Failed to get client pid");
    auto client_name = pocNameForPid(client_pid);
    if (_exclusiveClientName != client_name) {
        debug("ignoring client %d with name '%s' because it doesn't match exclusive procname '%s'",client_fd,client_name.c_str(),_exclusiveClientName.c_str());
        return false;
    }
    return true;
}

void ClientManager::handle_client(int client_fd){
    std::shared_ptr<Client> client = nullptr;
    cleanup([&]{
        if (client_fd > 0) {
            close(client_fd); client_fd = -1;
        }
    });
    
    try {
        client = std::make_shared<Client>(_mux,this, client_fd,_clientNumber++); client_fd = 0;
        client->_selfref = client;
    } catch (tihmstar::exception &e) {
        reterror("failed to handle client with error=%d",e.code());
    }

    {
        std::unique_lock<std::mutex> ul(_childrenLck);
        _children.insert(client.get());
    }
    
    //transfer ownership to muxer
    _mux->add_client(client); client = NULL;
}
