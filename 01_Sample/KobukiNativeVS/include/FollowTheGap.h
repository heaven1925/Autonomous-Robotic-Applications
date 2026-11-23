// FollowTheGap.h - the Follow-the-Gap obstacle-avoidance algorithm core.
//
// Pure function: RangeScan + goal bearing in -> (v, w) out. No Kobuki, no
// Windows, no I/O - fully platform independent, as required by the
// architecture note (Kobuki_Follow_the_Gap_Sensor_Mimarisi.md, section 4).
//
// Steps (matching that document's section 4.1, with multi-gap scoring):
//   1. group the raw samples that actually hit something into per-object
//      clusters (consecutive hits that stay close to each other in
//      range belong to the same obstacle). This separates "an obstacle's
//      own surface" from "open background behind/between obstacles",
//      which a plain distance threshold cannot do when two obstacles are
//      both comfortably far away but only narrowly spaced apart.
//   2. the space before the first cluster, between every pair of
//      neighbouring clusters, and after the last cluster are the
//      candidate gaps - EVERY one of them, not just the widest
//   3. measure each inter-obstacle gap's physical opening width (straight-
//      line distance between the two real obstacle points that bound it)
//      and confirm it is >= the robot's diameter plus a safety margin on
//      both sides - this is the explicit "kendi boyutunu hesaba katma /
//      sığıp sığmayacağını ölçme" the user asked for. A gap open on one
//      side (nothing between it and the edge of the field of view) is
//      unobstructed on that side and always wide enough.
//   4. score every navigable gap by how much it exceeds the required
//      width ("yeterli boşluk") and how close its heading is to the goal
//      bearing ("hedefe en yakın mesafe"); drive through the highest-
//      scoring gap
//   5. convert the chosen bearing into (v, w), slowing down for sharp
//      turns and for close obstacles
//   6. if no gap is wide enough anywhere, report gapFound = false so the
//      caller can stop the robot safely (section 4.1, step 7)
#pragma once

#include "RangeScan.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace ftg {

struct FollowTheGapParams {
    double robot_radius  = 0.20; // m   - half of the robot's 40 cm diameter
    double safety_margin = 0.10; // m   - extra clearance kept on EACH side of a gap
    double max_speed     = 0.20; // m/s
    double max_turn       = 0.80; // rad/s
    double turn_gain     = 1.2;  // P gain: bearing error -> angular speed
    double slow_turn_rad = 0.60; // rad - v is already fully reduced past this bearing error
    double slow_dist     = 0.80; // m   - start slowing v when the closest obstacle is nearer than this

    // Gap scoring weights (must sum to a positive number; they are
    // normalised internally). Raise width_weight to prefer the roomiest
    // opening, raise proximity_weight to prefer the one pointing most
    // directly at the goal.
    double width_weight     = 0.4; // how much extra width beyond "just fits" is worth
    double proximity_weight = 0.6; // how much heading-to-goal closeness is worth

    // A gap is scored as fully "yeterli" (sufficient) once it is this much
    // wider than the bare minimum fit, not only once it is enormous - a
    // modest, comfortable gap straight toward the goal should not lose to
    // a technically-unbounded detour around the outside of everything.
    double width_adequate_ratio = 1.2; // 20% of headroom above the minimum counts as "plenty"
};

struct FollowTheGapResult {
    double v = 0.0;
    double w = 0.0;
    bool   gapFound = false;
    double chosenBearing = 0.0;    // rad, robot-relative (for logging)
    double chosenGapWidth = 0.0;   // m, measured physical opening width of the chosen gap
    double closestObstacle = 0.0;  // m, nearest raw obstacle reading - for logging / metrics
    int    navigableGapCount = 0;  // how many gaps actually measured wide enough to fit the robot
};

namespace detail {

struct ObstacleCluster { std::size_t start, end; }; // inclusive raw-sample indices hitting the same object

// Groups samples that struck something (range meaningfully below
// max_range) into per-object clusters, splitting whenever the scan
// returns to open background or jumps to a clearly different object
// (a range disparity as big as the robot's own clearance). This is what
// lets the width check below compare genuine obstacle edges against each
// other, instead of mistaking an open background reading for one.
inline std::vector<ObstacleCluster> clusterObstacles(const RangeScan& scan, double clearance) {
    std::vector<ObstacleCluster> clusters;
    const std::size_t n = scan.ranges.size();
    const double backgroundThreshold = scan.max_range * 0.98;
    std::size_t i = 0;
    while (i < n) {
        if (scan.ranges[i] < backgroundThreshold) {
            const std::size_t start = i;
            ++i;
            while (i < n && scan.ranges[i] < backgroundThreshold &&
                   std::abs(scan.ranges[i] - scan.ranges[i - 1]) < clearance) {
                ++i;
            }
            clusters.push_back({start, i - 1});
        } else {
            ++i;
        }
    }
    return clusters;
}

inline double chordDistance(const RangeScan& scan, std::size_t a, std::size_t b) {
    const double aa = scan.angleAt(a), ab = scan.angleAt(b);
    const double ax = scan.ranges[a] * std::cos(aa), ay = scan.ranges[a] * std::sin(aa);
    const double bx = scan.ranges[b] * std::cos(ab), by = scan.ranges[b] * std::sin(ab);
    return std::hypot(bx - ax, by - ay);
}

} // namespace detail

inline FollowTheGapResult followTheGap(const RangeScan& scan, double goalBearingRad,
                                        const FollowTheGapParams& p = {}) {
    FollowTheGapResult result;
    const std::size_t n = scan.ranges.size();
    if (n == 0) return result;

    const double clearance     = p.robot_radius + p.safety_margin; // needed on EACH side of the robot
    const double requiredWidth = 2.0 * clearance;                  // robot diameter + margin both sides
    result.closestObstacle = *std::min_element(scan.ranges.begin(), scan.ranges.end());

    // 1-2. Cluster real obstacle hits, then every gap between/around them
    // (including the open edges of the field of view) is a candidate.
    auto clusters = detail::clusterObstacles(scan, clearance);

    auto clusterMinRangeOf = [&](const detail::ObstacleCluster& c) {
        double m = scan.max_range;
        for (std::size_t k = c.start; k <= c.end; ++k) m = (std::min)(m, scan.ranges[k]);
        return m;
    };

    // A ray can slip through a razor-thin sliver of angle right beside a
    // very close obstacle and still see all the way to a genuinely far,
    // separate object in the background - clusterObstacles would then
    // report that sliver as its own tiny "open" span. But the robot's
    // own body is far wider than that sliver at such close range, so
    // driving into it would still clip the near obstacle. Merge two
    // neighbouring clusters when BOTH of these hold: the angular span
    // between them is narrower than the clearance the nearer one
    // requires, AND the two are at clearly different distances (a real,
    // symmetric corridor/gap has both sides at comparable range - only a
    // "peek past a close obstacle to a distant one" has one side much
    // nearer than the other).
    for (bool merged = true; merged;) {
        merged = false;
        for (std::size_t c = 0; c + 1 < clusters.size(); ++c) {
            const double rangeA = clusterMinRangeOf(clusters[c]);
            const double rangeB = clusterMinRangeOf(clusters[c + 1]);
            const double nearRange = (std::min)(rangeA, rangeB);
            const double farRange  = (std::max)(rangeA, rangeB);
            const double marginAngle = std::atan2(clearance, (std::max)(nearRange, 1e-3));
            const double spanAngle = scan.angleAt(clusters[c + 1].start) - scan.angleAt(clusters[c].end);
            const bool tooThin = spanAngle < 2.0 * marginAngle;
            const bool lopsided = nearRange < 0.6 * farRange;
            if (tooThin && lopsided) {
                clusters[c].end = clusters[c + 1].end;
                clusters.erase(clusters.begin() + static_cast<long>(c) + 1);
                merged = true;
                break;
            }
        }
    }

    struct Candidate { double width, bearing; };
    std::vector<Candidate> candidates;

    auto addCandidate = [&](std::size_t spanStart, std::size_t spanEnd,
                             bool boundedLeft, bool boundedRight,
                             std::size_t leftEdge, std::size_t rightEdge) {
        if (spanStart > spanEnd) return; // the two obstacles touch directly - no gap here
        const double width = (boundedLeft && boundedRight)
            ? detail::chordDistance(scan, leftEdge, rightEdge)
            : std::numeric_limits<double>::max(); // open on at least one side - unobstructed there

        // The selectable heading range is shrunk inward from each bounded
        // side by the angle the robot's own clearance subtends at that
        // object's nearest boundary sample, so the chosen heading keeps
        // at least `clearance` away from the ray that first grazes the
        // obstacle - a lighter touch than using the object's single
        // closest point everywhere, which the cluster-merge step above
        // already protects against being exploited as a false gap.
        double lo = scan.angleAt(spanStart), hi = scan.angleAt(spanEnd);
        if (boundedLeft)
            lo += std::atan2(clearance, (std::max)(scan.ranges[leftEdge], 1e-3));
        if (boundedRight)
            hi -= std::atan2(clearance, (std::max)(scan.ranges[rightEdge], 1e-3));
        if (lo > hi) return; // the clearance margin consumed the whole span - not actually usable

        candidates.push_back({width, std::clamp(goalBearingRad, lo, hi)});
    };

    if (clusters.empty()) {
        addCandidate(0, n - 1, false, false, 0, 0); // nothing detected anywhere - fully open
    } else {
        if (clusters.front().start > 0)
            addCandidate(0, clusters.front().start - 1, false, true, 0, clusters.front().start);
        for (std::size_t c = 0; c + 1 < clusters.size(); ++c) {
            addCandidate(clusters[c].end + 1, clusters[c + 1].start - 1, true, true,
                         clusters[c].end, clusters[c + 1].start);
        }
        if (clusters.back().end + 1 < n)
            addCandidate(clusters.back().end + 1, n - 1, true, false, clusters.back().end, 0);
    }

    // 3-4. Keep only the gaps the robot physically fits through, score
    // the rest by (width adequacy, closeness to the goal bearing) and
    // pick the best one.
    bool   haveBest = false;
    double bestScore = -std::numeric_limits<double>::max();
    double bestWidth = 0.0, bestBearing = 0.0;

    for (const auto& c : candidates) {
        if (c.width < requiredWidth) continue; // measured too narrow for the robot - not navigable
        ++result.navigableGapCount;

        // "yeterli boşluk": how far past the bare minimum this gap measures,
        // saturating at width_adequate_ratio so a comfortably-wide gap scores
        // as fully sufficient rather than needing to be enormous.
        const double adequateWidth = requiredWidth * p.width_adequate_ratio;
        const double widthScore = c.width >= adequateWidth
            ? 1.0
            : std::clamp((c.width - requiredWidth) / (adequateWidth - requiredWidth), 0.0, 1.0);
        // "hedefe en yakın mesafe": 1.0 when the goal bearing is reachable inside
        // the gap untouched, decaying as the gap forces a larger detour.
        const double proximityScore = 1.0 - std::abs(normalize(goalBearingRad - c.bearing)) / Pi;

        const double score = p.width_weight * widthScore + p.proximity_weight * proximityScore;
        if (score > bestScore) {
            bestScore = score;
            haveBest = true;
            bestWidth = c.width;
            bestBearing = c.bearing;
        }
    }

    if (!haveBest) {
        result.gapFound = false; // no opening measures wide enough for the robot - caller must stop
        return result;
    }
    result.gapFound = true;
    result.chosenBearing = bestBearing;
    result.chosenGapWidth = bestWidth;

    // 5. Bearing error -> angular speed; slow down for sharp turns and
    //    for close obstacles.
    result.w = std::clamp(p.turn_gain * result.chosenBearing, -p.max_turn, p.max_turn);
    const double turnFactor = std::clamp(1.0 - std::abs(result.chosenBearing) / p.slow_turn_rad, 0.15, 1.0);
    const double distFactor = std::clamp(result.closestObstacle / p.slow_dist, 0.15, 1.0);
    result.v = p.max_speed * turnFactor * distFactor;

    return result;
}

} // namespace ftg

