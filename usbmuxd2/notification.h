//
//  notification.h
//  usbmuxd2
//
//  Created by erd on 10.12.24.
//  Copyright © 2024 erd. All rights reserved.
//

#ifndef notification_h
#define notification_h


#ifdef __cplusplus
extern "C"{
#endif

int notification_init();
void notification_send(const char *title, const char *message);
    

int desktop_notification_dameon();

#ifdef __cplusplus
}
#endif
#endif /* notification_h */
