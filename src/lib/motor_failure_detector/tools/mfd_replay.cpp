/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file mfd_replay.cpp
 *
 * Replays the real MotorFailureDetector over a recorded .ulg and prints a per-motor trip verdict.
 * Built on ulog_cpp, so it reads ESC current / motor command / arming state by field name from the
 * log's own format -- a single binary replays logs of any PX4 message version with no rebuild.
 * Mirrors escCheck.cpp: detector runs only when armed; config from the log's MOTFAIL_* params
 * (per-field fallback to a built-in calibration), with --set NAME=value overrides for what-if retuning.
 *
 * The runtime's exact 10 Hz sample phase can't be reconstructed offline, so in a single pass over the
 * log it runs kNumPhases detector instances on different grid offsets and reports a per-motor peak
 * BAND + trip fraction -- a single-phase verdict is only a lower bound.
 */

#include "MotorFailureDetector.hpp"

#include <ulog_cpp/data_container.hpp>
#include <ulog_cpp/exception.hpp>
#include <ulog_cpp/reader.hpp>

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

namespace
{
constexpr int kMM = MotorFailureDetector::kMaxMotors;
constexpr int kMotor1 = 101;   // esc_report::ACTUATOR_FUNCTION_MOTOR1
constexpr int kArmed = 2;      // vehicle_status::ARMING_STATE_ARMED
constexpr uint64_t kRuntimeTickUs = 100000;   // runtime runs the check at 10 Hz (Commander.cpp) -- decimate to match
constexpr int kNumPhases = 5;   // phase-sweep: run N detectors on staggered grid offsets to expose sampling-phase sensitivity

// Default model fit; the FALLBACK used when the log carries no MOTFAIL_* params.
// Threshold band calibrated at the runtime's 10 Hz cadence (kMaxGap = 0.3 s, phase-swept):
// clean across the reference multirotor corpus. Illustrative for this airframe, not a shipped default.
MotorFailureDetector::Config makeConfig()
{
	MotorFailureDetector::Config cfg{};
	cfg.model_a = 71.97f;
	cfg.model_b = 0.f;
	cfg.model_c = 2.88f;
	cfg.residual_lpf_tau_s = 0.2f;
	cfg.threshold_a = 5.0f;
	cfg.threshold_rel = 0.2f;
	cfg.persistence_s = 0.4f;
	return cfg;
}

// MOTFAIL_* params -> Config fields, mirroring escCheck.cpp. tau is the compile-time constant there too.
struct ParamBind {
	const char *name;
	float MotorFailureDetector::Config::*field;
};

const ParamBind kParamBinds[] = {
	{"MOTFAIL_QUAD", &MotorFailureDetector::Config::model_a},
	{"MOTFAIL_C2T",  &MotorFailureDetector::Config::model_b},
	{"MOTFAIL_IDLE", &MotorFailureDetector::Config::model_c},
	{"MOTFAIL_OFF",  &MotorFailureDetector::Config::threshold_a},
	{"MOTFAIL_REL",  &MotorFailureDetector::Config::threshold_rel},
	{"MOTFAIL_TIME", &MotorFailureDetector::Config::persistence_s},
};
constexpr int kNumParams = sizeof(kParamBinds) / sizeof(kParamBinds[0]);

// Set the Config field bound to param `name` (length `name_len`) to `value`. Returns its index, or -1.
int setParam(MotorFailureDetector::Config &cfg, const char *name, int name_len, float value)
{
	for (int i = 0; i < kNumParams; ++i) {
		if ((int)std::strlen(kParamBinds[i].name) == name_len && std::strncmp(kParamBinds[i].name, name, name_len) == 0) {
			cfg.*(kParamBinds[i].field) = value;
			return i;
		}
	}

	return -1;
}
}

class Replay : public ulog_cpp::DataContainer
{
public:
	Replay() : ulog_cpp::DataContainer(ulog_cpp::DataContainer::StorageConfig::Header)
	{
		for (int p = 0; p < kNumPhases; ++p) {
			for (int m = 0; m < kMM; ++m) { trip_time[p][m] = -1.0; }
		}
	}

	// set by main() from the command line before parsing
	MotorFailureDetector::Config cli_cfg{};
	bool cli_set[kNumParams] = {false};
	int cli_overrides = 0;

	// One detector instance per sampling phase; a single parse feeds all of them.
	MotorFailureDetector det[kNumPhases];
	MotorFailureDetector::Config cfg = makeConfig();
	bool log_set[kNumParams] = {false};
	int params_from_log = 0;
	bool configured = false;

	bool armed = false, have_status = false;
	float command_latest[kMM] = {0};
	bool reversible_latest[kMM] = {false};
	float peak[kNumPhases][kMM] = {};
	double trip_time[kNumPhases][kMM];
	bool seen[kMM] = {false};
	double start_us = -1.0;
	long evaluated = 0;   // phase-0 evaluation count (== per-phase sample count)

	// Cached ULog "legend" (filled in addLoggedMessage): each message's id + layout, and handles to the
	// fields we read -- resolved once so the per-record loop reads by handle, not by name.
	uint16_t esc_id = 0xffff, actuator_id = 0xffff, status_id = 0xffff;   // 0xffff = message type not seen yet
	std::shared_ptr<ulog_cpp::MessageFormat> esc_fmt, actuator_fmt, status_fmt;
	int esc_slots = 0, control_channels = 0;   // array sizes from the layouts: esc[] slots, control[] channels

	std::vector<char> esc_current_seen;   // per-ESC-slot "has reported current" latch (mirrors escCheck), never reset
	uint64_t next_eval_us[kNumPhases] = {0};   // per-phase next 10 Hz evaluation time (runtime-cadence match)

	// field handles (suffix _f), grouped by the message they come from:
	std::shared_ptr<ulog_cpp::Field> esc_timestamp_f, esc_array_f, esc_function_f, esc_current_f, esc_count_f;  // esc_status
	std::shared_ptr<ulog_cpp::Field> actuator_control_f, actuator_reversible_f;                                 // actuator_motors
	std::shared_ptr<ulog_cpp::Field> status_arming_f;                                                           // vehicle_status

	void addLoggedMessage(const ulog_cpp::AddLoggedMessage &msg) override
	{
		ulog_cpp::DataContainer::addLoggedMessage(msg);

		if (msg.multiId() != 0) { return; }

		auto it = messageFormats().find(msg.messageName());

		if (it == messageFormats().end()) { return; }

		if (msg.messageName() == "esc_status") {
			esc_id = msg.msgId(); esc_fmt = it->second;
			esc_array_f = esc_fmt->field("esc");
			esc_slots = esc_array_f->arrayLength();
			esc_timestamp_f = esc_fmt->field("timestamp");
			esc_count_f = esc_fmt->field("esc_count");
			esc_function_f = esc_array_f->nestedField("actuator_function");
			esc_current_f = esc_array_f->nestedField("esc_current");
			esc_current_seen.assign(esc_slots, 0);

		} else if (msg.messageName() == "actuator_motors") {
			actuator_id = msg.msgId(); actuator_fmt = it->second;
			actuator_control_f = actuator_fmt->field("control");
			control_channels = actuator_control_f->arrayLength();
			actuator_reversible_f = actuator_fmt->field("reversible_flags");

		} else if (msg.messageName() == "vehicle_status") {
			status_id = msg.msgId(); status_fmt = it->second;
			status_arming_f = status_fmt->field("arming_state");
			have_status = true;
		}
	}

	void parameter(const ulog_cpp::Parameter &p) override
	{
		ulog_cpp::DataContainer::parameter(p);

		if (configured || p.field().type().type != ulog_cpp::Field::BasicType::FLOAT) { return; }

		const std::string &name = p.field().name();
		const int idx = setParam(cfg, name.c_str(), (int)name.size(), p.value().as<float>());

		if (idx >= 0 && !log_set[idx]) { log_set[idx] = true; ++params_from_log; }
	}

	void data(const ulog_cpp::Data &record) override
	{
		const uint16_t id = record.msgId();

		if (id == status_id) {
			const bool now_armed = (ulog_cpp::TypedDataView(record, *status_fmt)[status_arming_f].as<int>() == kArmed);

			if (armed && !now_armed && configured) {   // disarm resets the detector (mirrors configure-on-disarm)
				for (int p = 0; p < kNumPhases; ++p) { det[p].configure(cfg); next_eval_us[p] = 0; }
			}

			armed = now_armed;
			return;
		}

		if (id == actuator_id) {
			ulog_cpp::TypedDataView view(record, *actuator_fmt);
			const uint32_t reversible_mask = view[actuator_reversible_f].as<uint32_t>();
			const auto control = view[actuator_control_f];

			for (int m = 0; m < kMM && m < control_channels; ++m) {
				command_latest[m] = control[m].as<float>();
				reversible_latest[m] = (reversible_mask >> m) & 1;
			}

			return;
		}

		if (id != esc_id) { return; }

		ulog_cpp::TypedDataView view(record, *esc_fmt);

		if (view[esc_count_f].as<int>() <= 0) { return; }       // runtime skips esc_status with esc_count <= 0

		if (have_status && !armed) { return; }                  // armed gate, like escCheck.cpp

		const uint64_t sample_us = view[esc_timestamp_f].as<uint64_t>();

		// Shape the per-motor inputs once -- they don't depend on the sampling phase, only on the
		// record. Which phase grids consume this sample is decided in the phase loop below.
		float command[kMM], current[kMM] {};
		bool reversible[kMM] {}, enabled[kMM] {};
		std::fill(std::begin(command), std::end(command), NAN);

		const auto esc_arr = view[esc_array_f];

		for (int i = 0; i < esc_slots; ++i) {
			const auto esc_i = esc_arr[i];
			const int function = esc_i[esc_function_f].as<int>();

			if (function < kMotor1 || function - kMotor1 >= kMM) { continue; }

			const float slot_current = esc_i[esc_current_f].as<float>();

			if (slot_current > FLT_EPSILON) { esc_current_seen[i] = 1; }   // latch first current report (per ESC slot)

			if (!esc_current_seen[i]) { continue; }                        // don't judge until the ESC reports current

			const int m = function - kMotor1;
			enabled[m] = true;
			seen[m] = true;
			current[m] = slot_current;
			command[m] = command_latest[m];                                // 0 until an actuator arrives (runtime zero-init)
			reversible[m] = reversible_latest[m];
		}

		if (!configured) {
			for (int i = 0; i < kNumParams; ++i) {
				if (cli_set[i]) { cfg.*(kParamBinds[i].field) = cli_cfg.*(kParamBinds[i].field); }
			}

			for (int p = 0; p < kNumPhases; ++p) { det[p].configure(cfg); }

			configured = true;
		}

		// One parse, kNumPhases detectors: feed each phase whose 10 Hz grid this sample crosses. Each
		// phase's grid is offset by p/kNumPhases of a tick, so a single pass over the log covers the
		// sampling-phase spread the runtime's unknown tick alignment could land on (no file re-read).
		for (int p = 0; p < kNumPhases; ++p) {
			if (next_eval_us[p] == 0) {   // first tick: anchor this phase's grid at its offset
				next_eval_us[p] = sample_us + (uint64_t)p * kRuntimeTickUs / kNumPhases;
			}

			if (sample_us < next_eval_us[p]) { continue; }

			next_eval_us[p] += kRuntimeTickUs;

			if (start_us < 0) { start_us = (double)sample_us; }

			const double elapsed_s = ((double)sample_us - start_us) / 1e6;

			det[p].update(kMM, (hrt_abstime)sample_us, command, current, reversible, enabled);

			for (int m = 0; m < kMM; ++m) {
				if (!seen[m]) { continue; }

				const float abs_residual_lpf = std::fabs(det[p].status(m).residual_lpf);

				if (!std::isnan(abs_residual_lpf) && abs_residual_lpf > peak[p][m]) { peak[p][m] = abs_residual_lpf; }

				if (det[p].status(m).failed && trip_time[p][m] < 0) { trip_time[p][m] = elapsed_s; }
			}

			if (p == 0) { ++evaluated; }
		}
	}
};

// Parse the whole log once through a fresh Replay (which runs all kNumPhases detectors internally).
// Returns the populated Replay, or nullptr on a fatal error (already reported to stderr).
std::shared_ptr<Replay> parseLog(const char *ulog_path, const MotorFailureDetector::Config &cli_cfg,
				 const bool cli_set[], int cli_overrides)
{
	auto replay = std::make_shared<Replay>();
	replay->cli_cfg = cli_cfg;
	std::copy(cli_set, cli_set + kNumParams, replay->cli_set);
	replay->cli_overrides = cli_overrides;

	FILE *file = fopen(ulog_path, "rb");

	if (!file) { std::fprintf(stderr, "cannot open %s\n", ulog_path); return nullptr; }

	uint8_t buf[4096];
	int bytes_read;
	ulog_cpp::Reader reader{replay};

	while ((bytes_read = fread(buf, 1, sizeof(buf), file)) > 0) {
		try {
			reader.readChunk(buf, bytes_read);

		} catch (const ulog_cpp::ExceptionBase &e) {
			std::fprintf(stderr, "%s: %s\n  (a needed field is missing -- the log's message layout differs from what this tool expects)\n",
				     ulog_path, e.what());
			fclose(file);
			return nullptr;
		}

		if (replay->hadFatalError()) { break; }
	}

	fclose(file);

	if (replay->hadFatalError()) { std::fprintf(stderr, "fatal parse error in %s\n", ulog_path); return nullptr; }

	return replay;
}

int main(int argc, char **argv)
{
	const char *usage = "usage: %s <log.ulg> [--set MOTFAIL_NAME=value ...]\n";
	const char *ulog_path = nullptr;
	MotorFailureDetector::Config cli_cfg{};
	bool cli_set[kNumParams] = {false};
	int cli_overrides = 0;

	for (int i = 1; i < argc; ++i) {
		if (!std::strcmp(argv[i], "--set") && i + 1 < argc) {
			const char *kv = argv[++i];
			const char *eq = std::strchr(kv, '=');
			char *end = nullptr;
			const float val = eq ? std::strtof(eq + 1, &end) : 0.f;
			const int idx = (eq && end != eq + 1) ? setParam(cli_cfg, kv, (int)(eq - kv), val) : -1;

			if (idx < 0) {
				std::fprintf(stderr, "bad --set '%s' (expected MOTFAIL_NAME=value)\n", kv);
				return 2;
			}

			if (!cli_set[idx]) { ++cli_overrides; }

			cli_set[idx] = true;

		} else if (!ulog_path && argv[i][0] != '-') {
			ulog_path = argv[i];

		} else {
			std::fprintf(stderr, usage, argv[0]);
			return 2;
		}
	}

	if (!ulog_path) { std::fprintf(stderr, usage, argv[0]); return 2; }

	auto replay = parseLog(ulog_path, cli_cfg, cli_set, cli_overrides);

	if (!replay) { return 2; }

	// Aggregate the per-phase results into a peak band + trip fraction per motor.
	bool seen_any[kMM] = {false};
	float peak_min[kMM], peak_max[kMM];
	int trip_count[kMM] = {0};

	for (int m = 0; m < kMM; ++m) { peak_min[m] = FLT_MAX; peak_max[m] = 0.f; }

	for (int m = 0; m < kMM; ++m) {
		if (!replay->seen[m]) { continue; }

		seen_any[m] = true;

		for (int p = 0; p < kNumPhases; ++p) {
			peak_min[m] = std::min(peak_min[m], replay->peak[p][m]);
			peak_max[m] = std::max(peak_max[m], replay->peak[p][m]);

			if (replay->trip_time[p][m] >= 0) { ++trip_count[m]; }
		}
	}

	for (const std::string &err : replay->parsingErrors()) { std::fprintf(stderr, "parse: %s\n", err.c_str()); }

	bool any_motor = false;

	for (int m = 0; m < kMM; ++m) { any_motor |= seen_any[m]; }

	if (!any_motor) {
		std::fprintf(stderr, "no armed esc_status samples in %s\n", ulog_path);
		return 2;
	}

	if (!replay->have_status) {
		std::fprintf(stderr, "warning: no vehicle_status in %s -- arming gate disabled, evaluating all esc samples\n", ulog_path);
	}

	char config_source[160];
	int n = replay->params_from_log > 0
		? std::snprintf(config_source, sizeof(config_source), "from log")
		: std::snprintf(config_source, sizeof(config_source), "built-in calibration");
	n = std::min(n, (int)sizeof(config_source) - 1);

	if (cli_overrides > 0) {
		n += std::snprintf(config_source + n, sizeof(config_source) - n, ", override%s:", cli_overrides > 1 ? "s" : "");

		for (int i = 0; i < kNumParams && n < (int)sizeof(config_source) - 1; ++i) {
			if (cli_set[i]) { n += std::snprintf(config_source + n, sizeof(config_source) - n, " %s", kParamBinds[i].name); }
		}
	}

	const MotorFailureDetector::Config &cfg = replay->cfg;
	const char *count_label = replay->have_status ? "armed samples each" : "samples each, arming gate OFF";
	std::printf("# %s  (%d phases, %ld %s)\n"
		    "# config (%s):  I_exp = %.2f*u^2 + %.2f*u + %.2f A,  trip if |LPF(I-I_exp)| > %.2f + %.2f*I_exp for %.2f s\n",
		    ulog_path, kNumPhases, replay->evaluated, count_label, config_source,
		    (double)cfg.model_a, (double)cfg.model_b, (double)cfg.model_c,
		    (double)cfg.threshold_a, (double)cfg.threshold_rel, (double)cfg.persistence_s);
	std::printf("# %-5s  %-15s  %-5s  %s\n", "motor", "peak|LPF(r)|[A]", "trips", "verdict");

	bool any_failed = false, phase_dependent = false;

	for (int m = 0; m < kMM; ++m) {
		if (!seen_any[m]) { continue; }

		const char *verdict;

		if (trip_count[m] == kNumPhases) {
			verdict = "FAILED (all phases)"; any_failed = true;

		} else if (trip_count[m] > 0) {
			verdict = "FAILED (sampling-dependent!)"; any_failed = true; phase_dependent = true;

		} else {
			verdict = "ok";
		}

		if (peak_max[m] > 1.3f * peak_min[m]) { phase_dependent = true; }   // wide band => phase-sensitive

		char trips_str[16];
		std::snprintf(trips_str, sizeof(trips_str), "%d/%d", trip_count[m], kNumPhases);
		std::printf("  %5d  %6.2f - %-6.2f  %-5s  %s\n",
			    m, (double)peak_min[m], (double)peak_max[m], trips_str, verdict);
	}

	if (phase_dependent) {
		std::printf("# NOTE: the verdict depends on which samples are used -- treat any trip as a\n"
			    "#       possible failure on the vehicle, not a pass.\n");
	}

	return any_failed ? 1 : 0;
}
