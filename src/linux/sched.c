// Basic scheduling functions and startup/shutdown code.
//
// Copyright (C) 2016-2021  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <execinfo.h>
#include <stdio.h>
#include <unistd.h>
#include <setjmp.h> // setjmp
#include <time.h>
#include "autoconf.h" // CONFIG_*
#include "basecmd.h" // stats_update
#include "board/io.h" // readb
#include "board/irq.h" // irq_save
#include "board/misc.h" // timer_from_us
#include "board/pgm.h" // READP
#include "command.h" // shutdown
#include "sched.h" // sched_check_periodic
#include "stepper.h" // stepper_event
#include "internal.h"

static struct {
    struct timer *timer_list;
    uint32_t tasks_counter;
    uint8_t shutdown_status, shutdown_reason;
} SchedStatus = {.timer_list = NULL};

// Schedule a function call at a supplied time.
void
sched_add_timer(struct timer *add)
{
    uint32_t waketime = add->waketime;

    struct timer *tl = SchedStatus.timer_list;
    if (unlikely(tl == NULL)) {
         SchedStatus.timer_list = add;
         add->next = NULL;
    } else if (unlikely(timer_is_before(waketime, tl->waketime))) {
        // This timer is before all other scheduled timers
        if (timer_is_before(waketime, timer_read_time()))
            try_shutdown("Timer in past");
        add->next = tl;
        SchedStatus.timer_list = add;
    } else {
        struct timer *prev = tl;
        struct timer *pos = tl->next;
        for (; pos; pos=pos->next) {
            if (timer_is_before(waketime, pos->waketime))
                break;
            prev = pos;
        }
        add->next = pos;
        prev->next = add;
    }
}

// Remove a timer that may be live.
void
sched_del_timer(struct timer *del)
{
    if (SchedStatus.timer_list != NULL) {
        if (SchedStatus.timer_list == del) {
            SchedStatus.timer_list = del->next;
        } else {
            struct timer *pos;
            for (pos = SchedStatus.timer_list; pos->next; pos = pos->next) {
                if (pos->next == del) {
                    pos->next = del->next;
                    break;
                }
            }
        }
    }
}

// Invoke the next timer - called from board hardware irq code.
unsigned int
sched_timer_dispatch(void)
{
    uint32_t sleep_time = timer_from_us(1000);
    if (SchedStatus.timer_list != NULL)
    {
        uint32_t cur_time = timer_read_time();

        uint32_t max_latency = -1;
        
        for (;;)
        {
            // Invoke timer callback
            while (timer_is_before(SchedStatus.timer_list->waketime, cur_time))
            {
                struct timer *t = SchedStatus.timer_list;

                uint32_t latency = ((int32_t)(cur_time) - (int32_t)(t->waketime));
                if (latency > max_latency || max_latency == -1)
                    max_latency = latency;

                uint_fast8_t res;
                if (CONFIG_INLINE_STEPPER_HACK && likely(!t->func)) {
                    res = stepper_event(t);
                } else {
                    res = t->func(t);
                }
                
                // Update timer_list (rescheduling current timer if necessary)
                if (unlikely(res == SF_DONE)) {
                    SchedStatus.timer_list = t->next;
                } else if (t->next == NULL || !timer_is_before(t->waketime, t->next->waketime)) {
                    SchedStatus.timer_list = t->next;
                    sched_add_timer(t);
                }

                if (SchedStatus.timer_list == NULL)
                    break;
            }

            if (SchedStatus.timer_list == NULL)
                break;

            cur_time = timer_read_time();

            if (!timer_is_before(SchedStatus.timer_list->waketime, cur_time))
            {
                int32_t diff = SchedStatus.timer_list->waketime - cur_time;
                if (diff > timer_from_us(10))
                {
                    sleep_time = diff - timer_from_us(10);
                    if (sleep_time > timer_from_us(100))
                        sleep_time = timer_from_us(100);

                    break;
                }
            }
        }
    }

    return sleep_time;
}

// Remove all user timers
void
sched_timer_reset(void)
{
    SchedStatus.timer_list = NULL;
}


/****************************************************************
 * Tasks
 ****************************************************************/

// Note that at least one task is ready to run
void
sched_wake_tasks(void)
{
    SchedStatus.tasks_counter++;
}

// Check if tasks need to be run
uint8_t
sched_tasks_busy(void)
{
    return SchedStatus.tasks_counter > 0;
}

// Note that a task is ready to run
void
sched_wake_task(struct task_wake *w)
{
    sched_wake_tasks();
    writeb(&w->wake, 1);
    SchedStatus.tasks_counter++;
}

// Check if a task is ready to run (as indicated by sched_wake_task)
uint8_t
sched_check_wake(struct task_wake *w)
{
    if (!readb(&w->wake))
        return 0;
    writeb(&w->wake, 0);
    return 1;
}

// Main task dispatch loop
static void
run_tasks(void)
{
    uint32_t start = timer_read_time();
    SchedStatus.tasks_counter = 1;
    for (;;) {
        do {
            uint32_t next_time = sched_timer_dispatch();
            struct timespec sleep;
            clock_gettime(CLOCK_MONOTONIC, &sleep);
            sleep.tv_nsec += 1000 * next_time / timer_from_us(1);
            if (sleep.tv_nsec > NSECS) {
                sleep.tv_nsec -= NSECS;
                sleep.tv_sec += 1;
            }

            clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &sleep, NULL);
            //console_sleep(next_time);
        } while (SchedStatus.tasks_counter == 0);

        // Run all tasks
        extern void ctr_run_taskfuncs(void);
        ctr_run_taskfuncs();

        // Update statistics
        uint32_t cur = timer_read_time();
        stats_update(start, cur);
        start = cur;

        SchedStatus.tasks_counter--;
    }
}

/****************************************************************
 * Shutdown processing
 ****************************************************************/

#define SHUTDOWNSTATUS_NONE 0
#define SHUTDOWNSTATUS_INPROGRESS 1
#define SHUTDOWNSTATUS_SHUTDOWN 2

// Return true if the machine is in an emergency stop state
uint8_t
sched_is_shutdown(void)
{
    return SchedStatus.shutdown_status != SHUTDOWNSTATUS_NONE;
}

// Transition out of shutdown state
void
sched_clear_shutdown(void)
{
    if (SchedStatus.shutdown_status == SHUTDOWNSTATUS_NONE)
        shutdown("Shutdown cleared when not shutdown");
    if (SchedStatus.shutdown_status == SHUTDOWNSTATUS_INPROGRESS)
        // Ignore attempt to clear shutdown if still processing shutdown
        return;
    SchedStatus.shutdown_status = SHUTDOWNSTATUS_NONE;
}

// Report the last shutdown reason code
void
sched_report_shutdown(void)
{
    sendf("is_shutdown static_string_id=%hu", SchedStatus.shutdown_reason);
}

// Shutdown the machine if not already in the process of shutting down
void __always_inline
sched_try_shutdown(uint_fast8_t reason)
{
    if (SchedStatus.shutdown_status == SHUTDOWNSTATUS_NONE)
        sched_shutdown(reason);
}

// Force the machine to immediately run the shutdown handlers
static jmp_buf shutdown_jmp;

// Force the machine to immediately run the shutdown handlers
void
sched_shutdown(uint_fast8_t reason)
{
    longjmp(shutdown_jmp, reason);
}

void
run_shutdown(int reason)
{
    uint32_t cur = timer_read_time();
    if (SchedStatus.shutdown_status == SHUTDOWNSTATUS_NONE)
        SchedStatus.shutdown_reason = reason;
    SchedStatus.shutdown_status = SHUTDOWNSTATUS_INPROGRESS;
    sched_timer_reset();
    extern void ctr_run_shutdownfuncs(void);
    ctr_run_shutdownfuncs();
    SchedStatus.shutdown_status = SHUTDOWNSTATUS_SHUTDOWN;

    sendf("shutdown clock=%u static_string_id=%hu", cur
          , SchedStatus.shutdown_reason);
}

/****************************************************************
 * Startup
 ****************************************************************/


// Main loop of program
void
sched_main(void)
{
    SchedStatus.shutdown_status = SHUTDOWNSTATUS_NONE;

    extern void ctr_run_initfuncs(void);
    ctr_run_initfuncs();

    sendf("starting");

    int ret = setjmp(shutdown_jmp);
    if (ret)
        run_shutdown(ret);

    run_tasks();
}
