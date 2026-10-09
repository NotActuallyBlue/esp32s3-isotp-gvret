#pragma once

#include <stdbool.h>

// What the KEY button does, kept free of hardware so it can be tested on the desktop.
//   press while the Pairing Mode screen is up   hide it
//   hold 2 s while Bluetooth is locked          open the Bluetooth window again (and the Pairing Mode screen)
//   hold 5 s                                    About screen; any later press closes it (it also closes itself after 90 s)
#define KEY_REOPEN_MS       2000
#define KEY_ABOUT_MS        5000
#define KEY_ABOUT_SHOW_MS   90000

typedef struct {
    int     held_ms;
    int     about_open_ms;
    bool    was_down;
    bool    block;              // a press that just closed About must be let go before it can count again
    bool    reopened;           // this press already reopened Bluetooth
    bool    about_can_close;    // the press that opened About has been let go
} key_state_t;

typedef struct {
    bool reopen_bluetooth;
    bool show_about;
    bool hide_about;
    bool hide_pairing;
} key_actions_t;

key_actions_t key_step(key_state_t *s, bool key_down, int elapsed_ms, bool bluetooth_locked, bool pairing_shown, bool about_shown);
