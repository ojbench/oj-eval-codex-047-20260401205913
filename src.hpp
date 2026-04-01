// Implementation for Problem 047 (ACMOJ 2284)
// You may submit this single header. It is included by the judge framework.
#ifndef PPCA_SRC_HPP
#define PPCA_SRC_HPP

#include "math.h"

class Controller {

public:
    Controller(const Vec &_pos_tar, double _v_max, double _r, int _id, Monitor *_monitor) {
        pos_tar = _pos_tar;
        v_max = _v_max;
        r = _r;
        id = _id;
        monitor = _monitor;
    }

    void set_pos_cur(const Vec &_pos_cur) {
        pos_cur = _pos_cur;
    }

    void set_v_cur(const Vec &_v_cur) {
        v_cur = _v_cur;
    }

private:
    int id;
    Vec pos_tar;
    Vec pos_cur;
    Vec v_cur;
    double v_max, r;
    Monitor *monitor;

    // Check if a candidate velocity is safe for the next interval
    bool candidate_safe(const Vec &v_candidate) const {
        int n = monitor->get_robot_number();
        for (int j = 0; j < n; ++j) {
            if (j == id) continue;
            Vec pj = monitor->get_pos_cur(j);
            Vec vj = monitor->get_v_cur(j); // last-step velocity (best estimate)
            double rj = monitor->get_r(j);

            Vec delta_pos = pos_cur - pj;
            Vec delta_v = v_candidate - vj;

            double project = delta_pos.dot(delta_v);
            if (project >= 0) {
                // Moving apart in relative motion
                continue;
            }
            double delta_v_norm = delta_v.norm();
            // time to closest approach along relative velocity direction (>=0)
            double t_closest = -project / delta_v_norm;
            double min_dis_sqr;
            double delta_r = r + rj;
            if (t_closest < delta_v_norm * TIME_INTERVAL) {
                // Closest approach occurs within the interval
                min_dis_sqr = delta_pos.norm_sqr() - t_closest * t_closest;
            } else {
                // Closest approach after interval end; check end-of-interval distance
                Vec end_delta = delta_pos + delta_v * TIME_INTERVAL;
                min_dis_sqr = end_delta.norm_sqr();
            }
            if (min_dis_sqr <= delta_r * delta_r - EPSILON) {
                return false; // collision risk
            }
        }
        return true;
    }

public:

    Vec get_v_next() {
        // If already at target, stay still
        Vec to_tar = pos_tar - pos_cur;
        double dist = to_tar.norm();
        if (dist <= EPSILON) {
            return Vec();
        }

        // Aim to reach target exactly if within one interval reach
        double max_safe_speed = v_max * 0.999; // small margin for numerical safety
        double desired_speed = dist / TIME_INTERVAL;
        double base_speed = desired_speed < max_safe_speed ? desired_speed : max_safe_speed;

        Vec dir = to_tar.normalize();
        Vec best = dir * base_speed;

        // If straight-line motion risks collision (based on last known velocities),
        // try reduced speeds down to full stop.
        if (!candidate_safe(best)) {
            static const double factors[] = {0.8, 0.6, 0.4, 0.2, 0.0};
            bool found = false;
            for (double f : factors) {
                Vec cand = dir * (base_speed * f);
                if (candidate_safe(cand)) {
                    best = cand;
                    found = true;
                    break;
                }
            }
            if (!found) {
                // As a fallback, attempt a slight perpendicular sidestep at low speed
                // to break symmetry in dense situations.
                double side_speed = std::min(max_safe_speed * 0.2, base_speed * 0.2);
                Vec lateral = Vec(-dir.y, dir.x) * side_speed; // rotate 90deg
                if (candidate_safe(lateral)) {
                    best = lateral;
                } else if (candidate_safe(-lateral)) {
                    best = -lateral;
                } else {
                    best = Vec();
                }
            }
        }

        return best;
    }
};


#endif // PPCA_SRC_HPP

