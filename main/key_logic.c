#include "key_logic.h"

key_actions_t key_step(key_state_t *s, bool key_down, int elapsed_ms, bool bluetooth_locked, bool pairing_shown, bool about_shown)
{
    key_actions_t act = { 0 };
    bool pressed = key_down && !s->was_down;

    if (about_shown) {
        s->about_open_ms += elapsed_ms;
        if (!key_down) {
            s->about_can_close = true;
            s->held_ms = 0;
            s->reopened = false;
        }
        if ((pressed && s->about_can_close) || s->about_open_ms >= KEY_ABOUT_SHOW_MS) {
            act.hide_about = true;
            s->about_can_close = false;
            s->block = true;
        }
        s->was_down = key_down;
        return act;
    }

    if (pressed && pairing_shown) act.hide_pairing = true;

    if (!key_down) {
        s->held_ms = 0;
        s->block = false;
        s->reopened = false;
    } else if (!s->block) {
        s->held_ms += elapsed_ms;
        if (s->held_ms >= KEY_REOPEN_MS && bluetooth_locked && !s->reopened) {
            act.reopen_bluetooth = true;
            s->reopened = true;
        }
        if (s->held_ms >= KEY_ABOUT_MS) {
            act.show_about = true;
            s->about_open_ms = 0;
            s->about_can_close = false;
            s->held_ms = 0;
        }
    }
    s->was_down = key_down;
    return act;
}
