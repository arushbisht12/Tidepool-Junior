#include <stdint.h>
#include <zephyr/kernel.h>
#include <stdbool.h>
#include <stddef.h>  /* size_t */
#include <string.h>  /* memcpy */
#include <errno.h>   /* EINVAL */
#include <inttypes.h>
#include "ekf_bridge.h"
#include <math.h>

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
	double accel[3];
	double ang_vel[3];
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
	uint32_t sequence;
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

enum reason_type {
	NONE,
	SNAPSHOT_FAILURE,
	STATE_MISSING,
	STATE_INVALID,
	STATE_STALE,
	STATE_FUTURE, 
	CONTEXT_MISSING,
	CONTEXT_INVALID,
	CONTEXT_STALE,
	CONTEXT_FUTURE, 
	SUPERVISOR_NOT_READY,
	OUTPUT_OUTSIDE_BOUNDS,
	CONTROL_CMD_PUBLISH_FAILURE,
};

typedef struct {
	enum control_status_type status;
	enum reason_type reason;
	int64_t pub_timestamp_ms;
} control_status;

typedef struct {
	double output;
	bool enabled;
	int64_t pub_timestamp_ms;
	uint32_t source_state_sequence;
} control_cmd;

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
static latest_val_channel context_chan = {
	.msg = &context_msg,
	.message_size = sizeof(context_msg),
	.mutex = &context_mutex,
	.published = false
};

static control_status control_status_msg;
K_MUTEX_DEFINE(control_status_mutex);
static latest_val_channel control_status_chan = {
	.msg = &control_status_msg,
	.message_size = sizeof(control_status_msg),
	.mutex = &control_status_mutex,
	.published = false
};

static control_cmd control_cmd_msg;
K_MUTEX_DEFINE(control_cmd_mutex);
static latest_val_channel control_cmd_chan = {
	.msg = &control_cmd_msg,
	.message_size = sizeof(control_cmd_msg),
	.mutex = &control_cmd_mutex,
	.published = false
};

// static int channel_init(latest_val_channel *chan, void *msg, )


static int state_publish(latest_val_channel *chan, const void *message, size_t message_size) {
	if (chan == NULL || message == NULL || message_size != chan->message_size) {
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


static int state_snapshot(latest_val_channel *chan, void *out, size_t out_size, bool *published) {
	if (chan == NULL || out == NULL || out_size != chan->message_size || published == NULL) {
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

K_SEM_DEFINE(startup_gate, 0, 2);


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

enum freshness_result {
	DATA_FRESH,
	DATA_MISSING,
	DATA_INVALID,
	DATA_STALE,
	DATA_FUTURE
};

static enum freshness_result check_freshness(bool received, bool valid, int64_t timestamp_ms, int64_t now_ms, int64_t age_limit_ms) {
	if (received==false) {
		return DATA_MISSING;
	} else if (valid==false) {
		return DATA_INVALID;
	}

	int64_t age_ms = now_ms - timestamp_ms;
	if (age_ms < 0) {
		return DATA_FUTURE;
	} else if (age_ms > age_limit_ms) {
		return DATA_STALE;
	} 
	
	return DATA_FRESH;
}

typedef struct {
	int64_t period_ms;
	int64_t next_release_ms;
	uint32_t overrun_count;
	uint32_t skipped_release_count;
} periodic_schedule;

static int periodic_advance(periodic_schedule *schedule, int64_t now_ms) {
	if (schedule==NULL || schedule->period_ms<=0) {
		return -EINVAL;
	}
	schedule->next_release_ms += schedule->period_ms;
	if (schedule->next_release_ms <= now_ms) {
		schedule->overrun_count++;
		while (schedule->next_release_ms <= now_ms) {
			schedule->next_release_ms += schedule->period_ms;
			schedule->skipped_release_count++;
		}
	}
	return 0;
}

// while (true) {
// 	/* read inputs, calculate, publish */

// 	int ret = periodic_advance(&schedule, k_uptime_get());

// 	/* TODO: handle configuration errors */

// 	k_sleep(K_TIMEOUT_ABS_MS(schedule.next_release_ms));
// }


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


void estimation_entry(void *p1, void *p2, void *p3) {
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	// wait on startup gate
	int ret = k_sem_take(&startup_gate, K_FOREVER);
	if (ret != 0) {
		printf("Estimator startup wait failed: %d\n", ret);
		return;
	}

	ekf_init();

	periodic_schedule schedule = {
		.period_ms = estimation_period_ms,
		.next_release_ms = k_uptime_get(),
	};

	uint32_t state_pub_failures = 0;
	uint32_t sequence = 0; 

	bool have_prev_sample = false;
	uint32_t prev_sample_timestamp_ms = 0;

	while (true) {
		// generate deterministic fake IMU. 
		IMU_sample data = {
			.accel = {0.0, 0.0, -9.81},
			.ang_vel = {0.0, 0.0, 0.0},
			.sample_timestamp_ms = k_uptime_get(),
		};

		// calculate state
		state_msg next_state = {0};

		if (!have_prev_sample) {
			have_prev_sample = true;
		} else {
			double dt_s = (data.sample_timestamp_ms - prev_sample_timestamp_ms)/ 1000.0;
			ekf_step(data.accel, dt_s, next_state.state.position);
		}

		//next_state.state.position[0] = data.accel[0] * 0.5 * 0.2 * 0.2; 
		next_state.sequence = ++sequence; // increments sequence, then grabs value
		next_state.valid = true;
		next_state.source_sample_timestamp_ms = data.sample_timestamp_ms;

		prev_sample_timestamp_ms = data.sample_timestamp_ms;

		int ret = state_publish(&state_chan, &next_state, sizeof(next_state));
		if (ret != 0) {
			state_pub_failures++;// count failure
		}

		ret = periodic_advance(&schedule, k_uptime_get());
		if (ret != 0) {
			printk("Estimator schedule invalid: %d\n", ret);
			return;
		}

		k_sleep(K_TIMEOUT_ABS_MS(schedule.next_release_ms));
	}
}

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

void control_entry(void *p1, void *p2, void *p3) {
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	// wait on startup gate
	int ret = k_sem_take(&startup_gate, K_FOREVER);
	if (ret != 0) {
		printf("Estimator startup wait failed: %d\n", ret);
		return;
	}

	periodic_schedule schedule = {
		.period_ms = control_period_ms,
		.next_release_ms = k_uptime_get(),
	};

	uint32_t control_cmd_pub_failures = 0;	
	uint32_t status_pub_failures = 0;
	bool enable_control = false;

	while(true) {
		enable_control = false;
		enum reason_type reason = 0;

		// snapshot state / context
		state_msg current_state = {0};
		bool received_state = false;
		int state_ret = state_snapshot(&state_chan, &current_state, sizeof(current_state), &received_state);

		context current_context = {0};
		bool received_context = false;
		int context_ret = state_snapshot(&context_chan, &current_context, sizeof(current_context), &received_context);

		int64_t now_ms = k_uptime_get(); // why after, not before?

		if (state_ret != 0 || context_ret != 0) {
			reason = SNAPSHOT_FAILURE;
		} else {
			enum freshness_result state_health = check_freshness(
			received_state,
			current_state.valid,
			current_state.source_sample_timestamp_ms,
			now_ms,
			STATE_AGE_LIMIT_MS);

			enum freshness_result context_health = check_freshness(
				received_context,
				true,
				current_context.pub_timestamp_ms,
				now_ms,
				CONTEXT_AGE_LIMIT_MS);
			
			if (state_health == DATA_MISSING) {
				reason = STATE_MISSING;
			} else if (state_health == DATA_INVALID) {
				reason = STATE_INVALID;
			} else if (state_health == DATA_STALE) {
				reason = STATE_STALE;
			} else if (state_health == DATA_FUTURE) {
				reason = STATE_FUTURE;
			} else if (context_health == DATA_MISSING) {
				reason = CONTEXT_MISSING;
			} else if (context_health == DATA_INVALID) {
				reason = CONTEXT_INVALID;
			} else if (context_health == DATA_STALE) {
				reason = CONTEXT_STALE;
			} else if (context_health == DATA_FUTURE) {
				reason = CONTEXT_FUTURE;
			} else if (current_context.mode != READY) {
				reason = SUPERVISOR_NOT_READY;
			} else if (!current_context.allow_control) {
				reason = 0;
			} else {
				enable_control = true;
			}
		}
		

		control_cmd cmd = {0};
		double error = 1.0 - current_state.state.position[0];
		double output = 0.01 * error;
		if (output <= 1.0 && output >= -1.0 && enable_control) {
			cmd.output = output;
		} else {
			if (reason == 0) {reason = OUTPUT_OUTSIDE_BOUNDS;}
			enable_control = false;
		}
		cmd.enabled = enable_control;
		cmd.source_state_sequence = current_state.sequence;
		cmd.pub_timestamp_ms = k_uptime_get();

		int ret = state_publish(&control_cmd_chan, &cmd, sizeof(cmd));
		if (ret != 0) {
			if (reason==0) {reason = CONTROL_CMD_PUBLISH_FAILURE;}
			control_cmd_pub_failures++;
		}

		enum control_status_type status = 0;
		if (enable_control) {status = ACTIVE;} else {status = INHIBITED;}

		control_status control_status_msg = {
			.status = status,
			.reason = reason,
			.pub_timestamp_ms = k_uptime_get(),
		};

		ret = state_publish(&control_status_chan, &control_status_msg, sizeof(control_status_msg));
		if (ret != 0) {
			printf("Control status failed to publish. \n");
			status_pub_failures++;
		}

		ret = periodic_advance(&schedule, k_uptime_get());
		if (ret != 0) {
			printk("Estimator schedule invalid: %d\n", ret);
			return;
		}

		k_sleep(K_TIMEOUT_ABS_MS(schedule.next_release_ms));
	}
}

/* TODO 7: Use K_THREAD_DEFINE for estimation and control only.
 * Supply your entry functions, stacks and priorities.
 * The last macro argument is startup delay, NOT the thread's period.
 * Start at boot; the entry functions wait on your startup gate.
 */

K_THREAD_DEFINE(estimation_thread, STACK_SIZE, estimation_entry, NULL, NULL, NULL, ESTIMATION_PRIO, 0, 0);
K_THREAD_DEFINE(control_thread, STACK_SIZE, control_entry, NULL, NULL, NULL, CONTROL_PRIO, 0, 0);



static void test_freshness(void) {
	enum freshness_result result = check_freshness(true, true, 950, 1000, 50);
	__ASSERT(result == DATA_FRESH, "A sample exactly at the age limit should be fresh");

	result = check_freshness(false, true, 950, 1000, 50);
	__ASSERT(result == DATA_MISSING, "A sample that wasn't received should be missing");

	result = check_freshness(true, false, 950, 1000, 50);
	__ASSERT(result == DATA_INVALID, "A sample that wasn't valid should be invalid");

	result = check_freshness(true, true, 949, 1000, 50);
	__ASSERT(result == DATA_STALE, "A sample below the age limit should be stale");

	result = check_freshness(true, true, 1001, 1000, 50);
	__ASSERT(result == DATA_FUTURE, "A sample time greater than the current time is from the future");
}

static void test_periodic_advance(void) {
	periodic_schedule schedule = {20, 1000, 0, 0};
	// int *original_ptr = &schedule;
	// int *copy_ptr = malloc(sizeof(int));
	// *copy_ptr = *original_ptr;
	
	int ret = periodic_advance(&schedule, 1003);
	__ASSERT(ret == 0, "Advance should succeed");
	__ASSERT(schedule.next_release_ms == 1020, "Next release should remain on schedule");

	ret = periodic_advance(&schedule, 1040);
	__ASSERT(ret == 0, "Advance should succeed");
	__ASSERT(schedule.overrun_count == 1 && schedule.skipped_release_count == 1, "Overran once, skipped one release.");
	__ASSERT(schedule.next_release_ms == 1060, "Next release should skip one");


	//printf("next release: %"PRIi64", overrun count: %d, skipped release count: %d\n", schedule.next_release_ms, schedule.overrun_count, schedule.skipped_release_count);
	ret = periodic_advance(&schedule, 1087);
	__ASSERT(ret == 0, "Advance should succeed");
	__ASSERT(schedule.overrun_count == 2 && schedule.skipped_release_count == 2, "Overran once more, skipped one more release.");
	__ASSERT(schedule.next_release_ms == 1100, "Next release should skip one");

	ret = periodic_advance(&schedule, 1145);
	__ASSERT(ret == 0, "Advance should succeed");
	__ASSERT(schedule.overrun_count == 3 && schedule.skipped_release_count == 4, "Overran once more, skipped two more releases.");
	__ASSERT(schedule.next_release_ms == 1160, "Next release should skip two");
}

static void test_ekf_bridge(void) {
	double accel[3] = {2.0, 0.0, 0.0};
	double position[3] = {0};
	double dt_seconds = 0.1;

	int ret = ekf_step(accel, dt_seconds, position);
	__ASSERT(ret==-EACCES, "Needs to be initialized before");

	ekf_init();
	
	ret = ekf_step(accel, -0.1, position);
	__ASSERT(ret=-EINVAL, "Negative dt");
	ret = ekf_step(NULL, dt_seconds, NULL);
	__ASSERT(ret=-EINVAL, "Null argument pointers");
	ret = ekf_step(accel, dt_seconds, position);
	__ASSERT(fabs(position[0] - 0.01) < 1e-9, "X should be 0.01");
	ekf_step(accel, dt_seconds, position);
	__ASSERT(fabs(position[0] - 0.02) < 1e-9, "X should be 0.02");

	ekf_init();
	ekf_get(position);
	__ASSERT(position[0]==0, "Reset to 0 failed");
}

static void run_tests(void) {
	test_freshness();
	test_periodic_advance();
	test_ekf_bridge();

	printk("All tests passed successfully.\n");
}

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


	run_tests();

	// initialization

	context supervisor_context = {
		.mode = INITIALIZING,
		.allow_control = false,
		.fault = FAULT_NONE,
		.pub_timestamp_ms = k_uptime_get(),
	};

	int ret = state_publish(&context_chan, &supervisor_context, sizeof(supervisor_context));
	if (ret != 0) {
		printk("Initial context publication failed: %d\n", ret);
		/* Handle startup failure; do not release workers yet. */
	} else {
		k_sem_give(&startup_gate);
		k_sem_give(&startup_gate);
	}

	periodic_schedule schedule = {
		.period_ms = supervisor_period_ms,
		.next_release_ms = k_uptime_get(),
	};

	// loop

	int supervisor_context_pub_failures = 0;

	while (true) {


		// state listener

		state_msg observed_state = {0};
		bool received_state = false;

		int state_ret = state_snapshot(
			&state_chan,
			&observed_state,
			sizeof(observed_state),
			&received_state);

		int64_t now_ms = k_uptime_get();

		if (state_ret != 0) {
			printk("State snapshot failed: %d\n", state_ret);
		} else if (!received_state) {
			printk("State: not published yet\n");
		} else {
			int64_t age_ms = now_ms - observed_state.source_sample_timestamp_ms;

			printk("State seq=%" PRIu32 " age=%" PRId64 " ms valid=%d\n",
				observed_state.sequence,
				age_ms,
				(int)observed_state.valid);
		}

		// control cmd listener

		control_cmd cmd = {0};
		bool received_cmd = false;

		int cmd_ret = state_snapshot(
			&control_cmd_chan,
			&cmd,
			sizeof(cmd),
			&received_cmd);

		now_ms = k_uptime_get();

		if (cmd_ret != 0) {
			printk("Control command snapshot failed: %d\n", cmd_ret);
		} else if (!received_cmd) {
			printk("Control command: not published yet\n");
		} else {
			int64_t age_ms = now_ms - cmd.pub_timestamp_ms;
			printk("Control output= %lf, age=%" PRId64 " ms,  enabled=%d\n",
				cmd.output,
				age_ms,
				(int)cmd.enabled);
		}


		// control status listener

		control_status status = {0};
		bool received_status = false;

		int status_ret = state_snapshot(
			&control_status_chan,
			&status,
			sizeof(status),
			&received_status);

		now_ms = k_uptime_get();

		if (status_ret != 0) {
			printk("Control status snapshot failed: %d\n", status_ret);
		} else if (!received_status) {
			printk("Control status: not published yet\n");
		} else {
			int64_t age_ms = now_ms - status.pub_timestamp_ms;

			printk("Control status type= %d, Reaon= %d, age=%" PRId64 " ms\n",
				status.status,
				status.reason, 
				age_ms);
		}

		supervisor_context.pub_timestamp_ms = k_uptime_get();
		supervisor_context.mode = READY;
		supervisor_context.allow_control = true;

		int context_ret = state_publish(
			&context_chan,
			&supervisor_context,
			sizeof(supervisor_context));
		if (context_ret != 0) {
			supervisor_context_pub_failures++;
		}


		int ret = periodic_advance(&schedule, k_uptime_get());
		if (ret != 0) {
			printf("Scheduler not initialized: %d\n", ret);
		}

		k_sleep(K_TIMEOUT_ABS_MS(schedule.next_release_ms));
	}




	// state_msg sample = {0};
	// sample.state.position[0] = 12.0;
	// sample.sequence = 1;
	// sample.valid = true;
	// sample.source_sample_timestamp_ms = k_uptime_get();

	// int ret = state_publish(&state_chan, &sample, sizeof(sample));

	// if (ret != 0) {
	// 	printf("Failed");
	// 	return -1;
	// }

	// state_msg latest = {0};
	// bool received = false;	

	// sample.sequence = 2;

	// ret = state_publish(&state_chan, &sample, sizeof(sample));

	// ret = state_snapshot(&state_chan, &latest, sizeof(latest), &received);

	// int64_t now_ms = k_uptime_get();
	// enum freshness_result result = check_freshness(received, latest.valid, latest.source_sample_timestamp_ms, now_ms, STATE_AGE_LIMIT_MS);

	// printf("received: %d \n sequence #: %d", received, latest.sequence);
}
