#include <zephyr/kernel.h>
#include <stdbool.h>
#include <stddef.h>  /* size_t */
#include <string.h>  /* memcpy */
#include <errno.h>   /* EINVAL */
/*
 * Three-thread C learning scaffold: YOU implement the TODOs.
 * Currently this builds as an empty app. Build after each small step.
 *
 * simulated IMU -> estimator -> state channel -> controller
 * supervisor -> context channel -> controller
 * controller -> command + control-status channels -> supervisor
 *
 * main() IS the supervisor thread. Create only two additional app threads.
 * Zephyr also has internal threads; those aren't part of this three-thread app.
 */

/* TODO 1: Define named constants for periods, age limits, stacks, priorities.
 * Starting periods: estimation 10 ms (100 Hz), control 20 ms (50 Hz),
 * supervisor 1000 ms (1 Hz). Freshness limits: state 50 ms, context 2500 ms,
 * control status 100 ms. Startup grace period: 3000 ms.
 * Start worker stacks at 1024 bytes and measure usage as the app grows.
 * Suggested preemptible priorities: control 4, estimator 5, supervisor 6.
 * Lower numbers mean higher priority. Main's priority is set in prj.conf.
 */
#define STACK_SIZE 1024
#define CONTROL_PRIO 4
#define ESTIMATION_PRIO 5
#define SUPERVISOR_PRIO 6

#define STATE_AGE_LIMIT_MS 50
#define CONTEXT_AGE_LIMIT_MS 2500
#define CONTROL_STATUS_AGE_LIMIT_MS 100

const int control_period_ms = 20;
const int estimation_period_ms = 10;
const int supervisor_period_ms = 1000;

/* TODO 2: Declare structs and enums. Include headers for the types you use.
 * Learn: enum, struct, bool, fixed-width integers, arrays, initialization.
 * IMU sample: acceleration[3], angular_velocity[3], sample timestamp.
 * State: placeholder position[3], velocity[3], attitude[3], valid flag,
 *        source-sample timestamp, sequence number.
 * Context: mode (INITIALIZING / READY / FAULT), allow_control, fault reason,
 *          publication timestamp, sequence number.
 * Command: placeholder output, enabled flag, timestamp, source-state sequence.
 * Control status: waiting / active / inhibited, reason, publication timestamp,
 *                 state age, consumed state sequence, cycle/overrun counters.
 * Use int64_t timestamps from k_uptime_get() (milliseconds since boot).
 * Zero is a valid timestamp: track whether a message was received separately.
 * Republishing old state must NOT refresh its source-sample timestamp.
 * Increment sequences on new publications. Keep simulated values simple.
 */

typedef struct {
	double acceleration[3];
	double angular_velocity[3];
	int64_t sample_timestamp_ms;
} IMU_sample;

typedef struct {
	double position[3];
	double velocity[3];
	double attitude[3];
} state_vector;
typedef struct {
	state_vector state;
	bool valid;
	int32_t sequence;
	int64_t source_sample_timestamp_ms;
} state_msg;

enum mode {
	INITIALIZING, READY, FAULT
};

enum fault_reason {
	FAULT_NONE = 0, 
	FAULT_STATE_STALE, 
	FAULT_STATE_MISSING, 
	FAULT_STATE_INVALID, 
	FAULT_CONTROL_STATUS_STALE
};

typedef struct {
	enum mode mode; 
	bool allow_control;
	enum fault_reason fault;
	int64_t pub_timestamp_ms;

} context;

enum control_status_type {
WAITING, ACTIVE, INHIBITED
};

typedef struct {
enum control_status_type status;
int64_t pub_timestamp_ms;
} control_status;

/* TODO 3: Define four latest-value channels, each protected by its own mutex.
 * Learn: static storage, pointers, struct copies, K_MUTEX_DEFINE.
 * Each channel holds one complete message and a "published" flag.
 * Write publish/snapshot functions for each type: lock, copy, unlock.
 * Snapshot into reader-owned local storage; never return mutable shared data.
 * Copy the published flag under the same lock. Check lock return values;
 * failed snapshots must inhibit control. "volatile" isn't synchronization.
 * Never calculate, sleep, print, or lock another channel while holding a lock.
 * State and context snapshots need not describe the same instant.
 * Each channel has one writer: estimator owns state; supervisor owns context;
 * controller owns command and status. Older messages are overwritten on purpose.
 * Reusing a still-fresh state on multiple control cycles is allowed.
 * Add a startup gate (K_SEM_DEFINE, capacity for two worker releases).
 * Workers wait once; main releases them after initialization/context publication.
 */


// ZBUS_CHAN_DEFINE(state_chan, struct state, NULL, NULL, ZBUS_OBSERVERS_EMPTY, ZBUS_MSG_INIT(.value = 0));

// // Thread A
// state latest_state = EKF.mean;
// zbus_chan_write(&state_chan, &latest_state, K_NO_WAIT);

// // Thread B
// state current_state = {0};
// zbus_chan_read(&state_chan, &current_state, K_NO_WAIT);

typedef struct {
	void *msg;
	size_t message_size;
	struct k_mutex *mutex;
	bool published;
} latest_val_channel;


static state_msg state_message;
K_MUTEX_DEFINE(state_mutex);
static latest_val_channel state_chan = {
	.msg = &state_message,
	.message_size = sizeof(state_message),
	.mutex = &state_mutex,
	.published = false
}; 


static context context_msg;
K_MUTEX_DEFINE(context_mutex);
//[[maybe_unused]] static latest_val_channel context_chan = {
	.msg = &context_msg,
	.message_size = sizeof(context_msg),
	.mutex = &context_mutex,
	.published = false
};

static control_status control_status_msg;
K_MUTEX_DEFINE(control_status_mutex);
//[[maybe_unused]] static latest_val_channel control_status_chan = {
	.msg = &control_status_msg,
	.message_size = sizeof(control_status_msg),
	.mutex = &control_status_mutex,
	.published = false
};

// static int channel_init(latest_val_channel *chan, void *msg, )


static int state_publish(latest_val_channel *chan, const void *message) {
	if (chan == NULL || message == NULL || sizeof(message) != chan->message_size) {
		return -EINVAL;
	}

	int ret = k_mutex_lock(chan->mutex, K_NO_WAIT);
	if (ret != 0) {
		printf("Cannot acquire/lock state\n");
		return ret;
	}

	memcpy(chan->msg, message, chan->message_size);
	chan->published = true;

	return k_mutex_unlock(chan->mutex);
}


static int state_snapshot(latest_val_channel *chan, void *out, bool *published) {
	if (chan == NULL || out == NULL || sizeof(out) != chan->message_size) {
		return -EINVAL;
	}
	int ret = k_mutex_lock(chan->mutex, K_NO_WAIT);
	if (ret != 0) {
		printf("Cannot acquire/lock state");
		return ret;
	}

	//*out = chan->msg;
	memcpy(out, chan->msg, chan->message_size);
	*published = chan->published; 

	return k_mutex_unlock(chan->mutex);
}



/* TODO 4: Write freshness and periodic timing helpers.
 * Learn: function arguments/returns, arithmetic, comparisons, signed types.
 * Fresh means received AND valid (where applicable) AND
 *             0 <= now - source_timestamp <= freshness_limit.
 * Distinguish missing, invalid, and stale data. Get now AFTER snapshots.
 * Each thread owns an absolute next-release timestamp. Run a cycle, advance
 * the deadline by its period, then k_msleep() only for the remaining time.
 * Sleeping a full period after work makes the frequency drift.
 * If late, count an overrun and skip expired releases until the next deadline
 * is in the future. Never busy-loop to catch up. Define the equality boundary.
 * Target frequency is not a hard guarantee; scheduling introduces jitter.
 */

/* TODO 5: Write the estimation entry function.
 * Signature: void function(void *p1, void *p2, void *p3).
 * Use ARG_UNUSED for unused arguments. Wait on the startup gate once.
 * Forever, at the estimation period:
 *   generate deterministic fake IMU from a counter/uptime (constant or ramp);
 *   calculate simple placeholder state; this is not real sensor fusion;
 *   stamp with SAMPLE time, set valid, increment sequence, publish;
 *   wait for the next absolute release.
 * Add a way to pause publication or mark samples invalid for fault experiments.
 * Do not print on every high-frequency iteration.
 */

/* TODO 6: Write the controller entry function. Wait on the startup gate once.
 * At EVERY control release:
 *   snapshot state/context, then get now;
 *   independently check missing/invalid/stale state and missing/stale context;
 *   enable only with fresh valid state AND fresh READY context allowing control;
 *   when enabled, calculate bounded proportional output from one state value,
 *     a target and a gain (a placeholder only; there is no physical actuator);
 *   otherwise publish DISABLED, ZERO output with a specific inhibit reason;
 *   publish command AND status even when inhibited, then wait until next release.
 * Record the source-state sequence in the command. A stale previously enabled
 * command must not survive a fault as if it were current.
 * Publishing status while inhibited prevents a supervisor startup deadlock.
 */

/* TODO 7: Use K_THREAD_DEFINE for estimation and control only.
 * Supply your entry functions, stacks and priorities.
 * The last macro argument is startup delay, NOT the thread's period.
 * Start at boot; the entry functions wait on your startup gate.
 */

int main(void)
{
	/* TODO 8: Implement the supervisor here.
	 * Initialize channels, flags and fault bookkeeping; publish INITIALIZING
	 * with allow_control false; release startup gate once for each worker.
	 * Then forever at the supervisor period:
	 *   snapshot state and controller status, then get now;
	 *   INITIALIZING -> READY on fresh valid state and fresh controller status;
	 *     an inhibited controller heartbeat is enough during initialization;
	 *   allow 3000 ms startup grace before missing data becomes a fault;
	 *   READY -> FAULT on missing/invalid/stale state or missing/stale status;
	 *   latch the first fault until reboot, keeping allow_control false;
	 *   publish context EVERY cycle so controller detects supervisor loss;
	 *   print time, mode/fault, state sequence/age, controller status/age,
	 *     optional command, and counters. Show missing messages explicitly;
	 *   wait for the next absolute release.
	 * Use correct printk format specifiers and end each status line with \n.
	 * A 1 Hz supervisor may take about 1 s to notice faults; controller's own
	 * age checks inhibit earlier, at its next 20 ms release.
	 */

	state_msg sample = {0};
	sample.state.position[0] = 12.0;
	sample.sequence = 1;
	sample.valid = true;
	sample.source_sample_timestamp_ms = k_uptime_get();

	int ret = state_publish(&state_chan, &sample);

	if (ret != 0) {
		printf("Failed");
		return -1;
	}

	state_msg latest = {0};
	bool received = false;	

	sample.sequence = 2;

	ret = state_publish(&state_chan, &sample);

	ret = state_snapshot(&state_chan, &latest, &received);

	printf("received: %d \n sequence #: %d", received, latest.sequence);





	return 0; /* Replace with your supervisor loop in TODO 8. */
}
