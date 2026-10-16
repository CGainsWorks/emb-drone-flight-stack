// fc_sil: run the standard software-in-the-loop step responses.
//
//   fc_sil [--out DIR] [--json] [--check]
//          [--rate-roll KP,KI,KD] [--rate-pitch KP,KI,KD] [--rate-yaw KP,KI,KD]
//          [--angle-p KP] [--estimator-kp KP] [--motor-tau S] [--inertia-scale K]
//
// --motor-tau and --inertia-scale perturb the assumed plant, so a gain set can
// be checked for margin rather than fitted to one model.
//
// Without gain options the firmware's own FlightControllerConfig defaults are
// used, which is what CI gates on. --out writes one CSV per scenario plus
// metrics.json; --json prints the metrics to stdout (used by
// tools/pid_tuner.py); --check exits non-zero if any scenario misses its
// limits. All results are simulated.

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include "sil_harness.hpp"

namespace {

bool parse_gains(const char* text, fc::PidGains& g) {
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;
    if (std::sscanf(text, "%f,%f,%f", &kp, &ki, &kd) != 3) {
        return false;
    }
    g = fc::PidGains{kp, ki, kd};
    return true;
}

int usage() {
    std::fprintf(stderr,
                 "usage: fc_sil [--out DIR] [--json] [--check] [--rate-roll KP,KI,KD]\n"
                 "              [--rate-pitch KP,KI,KD] [--rate-yaw KP,KI,KD] [--angle-p KP]\n");
    return 2;
}

}  // namespace

int main(int argc, char** argv) {
    fc::FlightControllerConfig cfg;
    fc::sil::QuadParams plant;
    std::string out_dir;
    bool print_json = false;
    bool check = false;

    for (int i = 1; i < argc; ++i) {
        const char* a = argv[i];
        const bool has_value = i + 1 < argc;
        if (std::strcmp(a, "--out") == 0 && has_value) {
            out_dir = argv[++i];
        } else if (std::strcmp(a, "--json") == 0) {
            print_json = true;
        } else if (std::strcmp(a, "--check") == 0) {
            check = true;
        } else if (std::strcmp(a, "--rate-roll") == 0 && has_value) {
            if (!parse_gains(argv[++i], cfg.rate_roll)) {
                return usage();
            }
        } else if (std::strcmp(a, "--rate-pitch") == 0 && has_value) {
            if (!parse_gains(argv[++i], cfg.rate_pitch)) {
                return usage();
            }
        } else if (std::strcmp(a, "--rate-yaw") == 0 && has_value) {
            if (!parse_gains(argv[++i], cfg.rate_yaw)) {
                return usage();
            }
        } else if (std::strcmp(a, "--angle-p") == 0 && has_value) {
            cfg.angle_p = std::strtof(argv[++i], nullptr);
        } else if (std::strcmp(a, "--estimator-kp") == 0 && has_value) {
            cfg.estimator.kp = std::strtof(argv[++i], nullptr);
        } else if (std::strcmp(a, "--motor-tau") == 0 && has_value) {
            plant.motor_tau_s = std::strtod(argv[++i], nullptr);
        } else if (std::strcmp(a, "--inertia-scale") == 0 && has_value) {
            const double k = std::strtod(argv[++i], nullptr);
            plant.ixx *= k;
            plant.iyy *= k;
            plant.izz *= k;
        } else {
            return usage();
        }
    }

    std::vector<fc::sil::RunResult> runs;
    bool all_pass = true;
    for (const auto& s : fc::sil::standard_scenarios()) {
        runs.push_back(fc::sil::run_scenario(s, cfg, plant));
        const auto& r = runs.back();
        const bool pass = r.armed && fc::sil::within_limits(r.metrics, fc::sil::limits_for(s));
        all_pass = all_pass && pass;
        if (!print_json) {
            std::printf("%-18s %s  overshoot %7.2f %%  settling %7.3f s  ss error %6.2f %%\n",
                        s.name.c_str(), pass ? "PASS" : "FAIL", r.metrics.overshoot_pct,
                        r.metrics.settling_time_s, r.metrics.steady_state_error_pct);
        }
        if (!out_dir.empty()) {
            const std::string path = out_dir + "/" + s.name + ".csv";
            if (!fc::sil::write_csv(r, path)) {
                std::fprintf(stderr, "cannot write %s\n", path.c_str());
                return 1;
            }
        }
    }

    const std::string json = fc::sil::metrics_json(runs, cfg, plant);
    if (print_json) {
        std::fputs(json.c_str(), stdout);
    }
    if (!out_dir.empty()) {
        const std::string path = out_dir + "/metrics.json";
        std::FILE* f = std::fopen(path.c_str(), "wb");
        if (f == nullptr) {
            std::fprintf(stderr, "cannot write %s\n", path.c_str());
            return 1;
        }
        std::fputs(json.c_str(), f);
        std::fclose(f);
    }

    if (!print_json) {
        std::printf("(simulated: software-in-the-loop against an assumed plant)\n");
    }
    return (check && !all_pass) ? 1 : 0;
}
