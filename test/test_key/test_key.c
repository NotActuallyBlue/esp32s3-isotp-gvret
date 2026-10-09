// Host-side tests for the KEY button logic. Run with: pio test -e native
#include <unity.h>
#include <string.h>
#include "../../main/key_logic.c"

static key_state_t st;
void setUp(void) { memset(&st, 0, sizeof(st)); }
void tearDown(void) {}

// Hold the key for ms and report whether each action fired at some point
typedef struct { bool reopen, about, hide_about, hide_pairing; int reopen_at, about_at; } seen_t;
static seen_t hold(int ms, bool locked, bool pairing, bool about_up)
{
    seen_t seen = { 0 };
    for (int t = 50; t <= ms; t += 50) {
        key_actions_t a = key_step(&st, true, 50, locked, pairing, about_up);
        if (a.reopen_bluetooth && !seen.reopen) { seen.reopen = true; seen.reopen_at = t; }
        if (a.show_about && !seen.about) { seen.about = true; seen.about_at = t; }
        seen.hide_about |= a.hide_about;
        seen.hide_pairing |= a.hide_pairing;
    }
    return seen;
}
static void release(void) { key_step(&st, false, 50, false, false, false); }

void test_short_tap_does_nothing(void)
{
    seen_t s = hold(1900, true, false, false);
    TEST_ASSERT_FALSE(s.reopen); TEST_ASSERT_FALSE(s.about);
}

void test_two_second_hold_reopens_when_locked(void)
{
    seen_t s = hold(2100, true, false, false);
    TEST_ASSERT_TRUE(s.reopen);
    TEST_ASSERT_EQUAL(2000, s.reopen_at);
    TEST_ASSERT_FALSE(s.about);
}

void test_two_second_hold_does_nothing_when_not_locked(void)
{
    seen_t s = hold(3000, false, false, false);
    TEST_ASSERT_FALSE(s.reopen); TEST_ASSERT_FALSE(s.about);
}

void test_five_second_hold_opens_about(void)
{
    seen_t s = hold(5100, false, false, false);
    TEST_ASSERT_TRUE(s.about);
    TEST_ASSERT_EQUAL(5000, s.about_at);
}

void test_locked_hold_to_five_seconds_reopens_then_about(void)
{
    seen_t s = hold(5100, true, false, false);
    TEST_ASSERT_TRUE(s.reopen); TEST_ASSERT_TRUE(s.about);
    TEST_ASSERT_TRUE(s.reopen_at < s.about_at);
}

void test_reopen_fires_once_per_press(void)
{
    int count = 0;
    for (int t = 50; t <= 4000; t += 50) if (key_step(&st, true, 50, true, false, false).reopen_bluetooth) count++;
    TEST_ASSERT_EQUAL(1, count);
    release();
    for (int t = 50; t <= 2100; t += 50) if (key_step(&st, true, 50, true, false, false).reopen_bluetooth) count++;
    TEST_ASSERT_EQUAL(2, count);                    // a new press can reopen again
}

void test_press_hides_the_pairing_screen(void)
{
    key_actions_t a = key_step(&st, true, 50, false, true, false);
    TEST_ASSERT_TRUE(a.hide_pairing);
    a = key_step(&st, true, 50, false, true, false);
    TEST_ASSERT_FALSE(a.hide_pairing);              // only on the press itself
}

void test_press_does_not_hide_pairing_when_not_shown(void)
{
    TEST_ASSERT_FALSE(key_step(&st, true, 50, false, false, false).hide_pairing);
}

void test_about_closes_on_a_new_press_not_the_opening_one(void)
{
    hold(5100, false, false, false);                // opens About
    // still holding the opening press: must not close
    for (int i = 0; i < 20; i++) TEST_ASSERT_FALSE(key_step(&st, true, 50, false, false, true).hide_about);
    release();
    key_step(&st, false, 50, false, false, true);   // let go while About is up
    TEST_ASSERT_TRUE(key_step(&st, true, 50, false, false, true).hide_about);
}

void test_long_closing_press_does_not_reopen_about(void)
{
    hold(5100, false, false, false);
    key_step(&st, false, 50, false, false, true);
    TEST_ASSERT_TRUE(key_step(&st, true, 50, false, false, true).hide_about);
    // About is closed now; the key is still held for another 6 s
    seen_t s = hold(6000, false, false, false);
    TEST_ASSERT_FALSE(s.about);
    release();
    s = hold(5100, false, false, false);
    TEST_ASSERT_TRUE(s.about);                      // a fresh press works again
}

void test_about_closes_by_itself(void)
{
    hold(5100, false, false, false);
    key_step(&st, false, 50, false, false, true);
    bool closed = false;
    for (int t = 0; t < 91000; t += 50) if (key_step(&st, false, 50, false, false, true).hide_about) { closed = true; break; }
    TEST_ASSERT_TRUE(closed);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_short_tap_does_nothing);
    RUN_TEST(test_two_second_hold_reopens_when_locked);
    RUN_TEST(test_two_second_hold_does_nothing_when_not_locked);
    RUN_TEST(test_five_second_hold_opens_about);
    RUN_TEST(test_locked_hold_to_five_seconds_reopens_then_about);
    RUN_TEST(test_reopen_fires_once_per_press);
    RUN_TEST(test_press_hides_the_pairing_screen);
    RUN_TEST(test_press_does_not_hide_pairing_when_not_shown);
    RUN_TEST(test_about_closes_on_a_new_press_not_the_opening_one);
    RUN_TEST(test_long_closing_press_does_not_reopen_about);
    RUN_TEST(test_about_closes_by_itself);
    return UNITY_END();
}
