// Results.h - reads results/results.csv (one line per run, written by every
// Explore_*.exe, see common/Metrics.h) and builds the comparison of the four
// methods on the three criteria:
//
//   1. Speed          mean time_s of the runs that finished the map by themselves
//   2. Success ratio  mean success_ratio (correct / observable cells of the obstacle map)
//   3. Obstacle maps  number of different maps "passed": a map is passed when at
//                     least half of the method's runs on it finished by themselves
//                     with success_ratio >= threshold
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <ctime>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

namespace results {

struct Row {
    std::string timestamp, algorithm, map, mode, finish, output;
    unsigned seed = 0;
    bool noise = true, verify = true, completed = false, scored = false;
    double timeS = 0.0, drivenM = 0.0, success = 0.0, accuracy = 0.0;
    int observable = 0, correctObs = 0, wrongObs = 0, undecidedObs = 0, correct = 0, wrong = 0, undecided = 0;
    int bumps = 0, scans = 0, plans = 0, goals = 0, faceOn = 0;
    long readings = 0;
};

inline std::vector<std::string> splitCsv(const std::string& line) {
    std::vector<std::string> out;
    std::string cur;
    for (char ch : line) {
        if (ch == ',') { out.push_back(cur); cur.clear(); }
        else if (ch != '\r') cur += ch;
    }
    out.push_back(cur);
    return out;
}

// Columns are looked up by their header name, so older / newer files still load.
inline std::vector<Row> load(const std::filesystem::path& path, std::string* error = nullptr) {
    std::vector<Row> rows;
    std::ifstream in(path);
    if (!in) { if (error) *error = "no results yet (" + path.string() + ")"; return rows; }
    std::string line;
    if (!std::getline(in, line)) return rows;
    const std::vector<std::string> head = splitCsv(line);
    std::map<std::string, std::size_t> col;
    for (std::size_t i = 0; i < head.size(); ++i) col[head[i]] = i;
    while (std::getline(in, line)) {
        if (line.empty()) continue;
        const std::vector<std::string> f = splitCsv(line);
        const auto get = [&](const char* name) -> std::string {
            const auto it = col.find(name);
            return it != col.end() && it->second < f.size() ? f[it->second] : std::string();
        };
        const auto num = [&](const char* name) { const std::string s = get(name); return s.empty() ? 0.0 : std::atof(s.c_str()); };
        const auto inum = [&](const char* name) { return static_cast<int>(num(name)); };
        Row r;
        r.timestamp = get("timestamp");
        r.algorithm = get("algorithm");
        r.map = get("map");
        r.mode = get("mode");
        r.finish = get("finish");
        r.output = get("output");
        r.seed = static_cast<unsigned>(num("seed"));
        r.noise = get("noise") != "0";
        r.verify = get("verify") != "0";
        r.completed = get("completed") == "1";
        r.scored = !get("success_ratio").empty();
        r.timeS = num("time_s");
        r.drivenM = num("driven_m");
        r.success = num("success_ratio");
        r.accuracy = num("accuracy_all");
        r.observable = inum("observable");
        r.correctObs = inum("correct_obs");
        r.wrongObs = inum("wrong_obs");
        r.undecidedObs = inum("undecided_obs");
        r.correct = inum("correct");
        r.wrong = inum("wrong");
        r.undecided = inum("undecided");
        r.bumps = inum("bumps");
        r.scans = inum("scans");
        r.plans = inum("plans");
        r.goals = inum("goals");
        r.faceOn = inum("face_on");
        r.readings = static_cast<long>(num("readings"));
        if (!r.algorithm.empty()) rows.push_back(r);
    }
    return rows;
}

// ------------------------------------------------------------ comparison --
struct Cell {          // one method on one map
    int runs = 0, completed = 0, passed = 0, scored = 0;
    double timeSum = 0.0, successSum = 0.0, drivenSum = 0.0;
    double meanTime() const { return completed ? timeSum / completed : 0.0; }
    double meanSuccess() const { return scored ? successSum / scored : 0.0; }
    bool mapPassed() const { return runs > 0 && passed * 2 >= runs; }
};

struct Summary {       // one method over all maps
    std::string algorithm;
    int runs = 0, completed = 0, scored = 0, mapsTried = 0, mapsPassed = 0, rank = 0;
    double timeSum = 0.0, successSum = 0.0, drivenSum = 0.0;
    std::map<std::string, Cell> perMap;
    double meanTime() const { return completed ? timeSum / completed : 0.0; }
    double meanSuccess() const { return scored ? successSum / scored : 0.0; }
    double meanDriven() const { return completed ? drivenSum / completed : 0.0; }
};

struct Comparison {
    std::vector<Summary> methods;     // ranked: maps passed, success, speed
    std::vector<std::string> maps;    // every map seen
};

// mode: "sim", "real" or "" (all). threshold: success ratio (0..1) for a passed run.
inline Comparison compare(const std::vector<Row>& rows, const std::string& mode, double threshold,
                          const std::vector<std::string>& methodOrder) {
    Comparison c;
    std::map<std::string, Summary> by;
    for (const Row& r : rows) {
        if (!mode.empty() && r.mode != mode) continue;
        Summary& s = by[r.algorithm];
        s.algorithm = r.algorithm;
        Cell& cell = s.perMap[r.map];
        ++s.runs;
        ++cell.runs;
        if (r.completed) {
            ++s.completed; ++cell.completed;
            s.timeSum += r.timeS; cell.timeSum += r.timeS;
            s.drivenSum += r.drivenM; cell.drivenSum += r.drivenM;
        }
        if (r.scored) {
            ++s.scored; ++cell.scored;
            s.successSum += r.success; cell.successSum += r.success;
        }
        if (r.completed && r.scored && r.success + 1e-9 >= threshold) ++cell.passed;
        if (std::find(c.maps.begin(), c.maps.end(), r.map) == c.maps.end()) c.maps.push_back(r.map);
    }
    std::sort(c.maps.begin(), c.maps.end());
    for (auto& [id, s] : by) {
        s.mapsTried = static_cast<int>(s.perMap.size());
        for (const auto& [m, cell] : s.perMap) s.mapsPassed += cell.mapPassed() ? 1 : 0;
        c.methods.push_back(s);
    }
    // Rank: more maps passed, then higher success, then faster.
    std::vector<Summary> ranked = c.methods;
    std::sort(ranked.begin(), ranked.end(), [](const Summary& a, const Summary& b) {
        if (a.mapsPassed != b.mapsPassed) return a.mapsPassed > b.mapsPassed;
        if (std::abs(a.meanSuccess() - b.meanSuccess()) > 1e-6) return a.meanSuccess() > b.meanSuccess();
        const double ta = a.completed ? a.meanTime() : 1e18, tb = b.completed ? b.meanTime() : 1e18;
        return ta < tb;
    });
    for (std::size_t i = 0; i < ranked.size(); ++i)
        for (Summary& s : c.methods)
            if (s.algorithm == ranked[i].algorithm) s.rank = static_cast<int>(i) + 1;
    // Display order: the methods' own order (11_01 .. 11_04), unknown ids last.
    const auto pos = [&](const std::string& id) {
        const auto it = std::find(methodOrder.begin(), methodOrder.end(), id);
        return it == methodOrder.end() ? 1000 : static_cast<int>(it - methodOrder.begin());
    };
    std::sort(c.methods.begin(), c.methods.end(), [&](const Summary& a, const Summary& b) { return pos(a.algorithm) < pos(b.algorithm); });
    return c;
}

inline std::string fmt(double v, int prec) {
    char b[48];
    std::snprintf(b, sizeof(b), "%.*f", prec, v);
    return b;
}

// Markdown report: criteria, ranking, method x map matrix, every run.
inline bool writeReport(const std::filesystem::path& path, const std::vector<Row>& rows, const Comparison& c, const std::string& mode,
                        double threshold, const std::map<std::string, std::string>& names) {
    std::ofstream out(path);
    if (!out) return false;
    const auto nameOf = [&](const std::string& id) {
        const auto it = names.find(id);
        return it == names.end() ? id : it->second;
    };
    const std::time_t t = std::time(nullptr);
    std::tm tm{};
    localtime_s(&tm, &t);
    char date[32];
    std::strftime(date, sizeof(date), "%Y-%m-%d %H:%M", &tm);
    out << "# 11 Exploration Mapping - yontem karsilastirmasi\n\n"
        << "Tarih: " << date << "  \nKosular: " << (mode.empty() ? "tumu" : (mode == "sim" ? "simulasyon" : "gercek robot"))
        << "  \nBasari esigi: %" << fmt(threshold * 100.0, 0) << "\n\n"
        << "## Kriterler\n\n"
        << "1. **Hiz** - haritayi kendiliginden tamamlayan kosularin ortalama suresi (s). Simulasyonda simulasyon saati, robotta gercek sure.\n"
        << "2. **Basari orani** - dogru karar verilen hucre / gorulebilir hucre (engel haritasina gore). Gorulebilir hucre: bos hucre ya da bos komsusu olan engel hucresi; hicbir sonarin goremeyecegi dis koseler sayilmaz.\n"
        << "3. **Engel haritasi** - kac farkli haritada basarili: kosularin en az yarisi kendiliginden bitti ve basari orani >= esik.\n\n"
        << "## Ozet (sira: basarili harita, basari orani, hiz)\n\n"
        << "| Sira | Yontem | Kosu | Tamamlanan | Ort. sure (s) | Ort. basari | Ort. yol (m) | Basarili harita |\n"
        << "|---:|---|---:|---:|---:|---:|---:|---:|\n";
    std::vector<const Summary*> ranked;
    for (const Summary& s : c.methods) ranked.push_back(&s);
    std::sort(ranked.begin(), ranked.end(), [](const Summary* a, const Summary* b) { return a->rank < b->rank; });
    for (const Summary* s : ranked)
        out << "| " << s->rank << " | " << nameOf(s->algorithm) << " | " << s->runs << " | " << s->completed << " | "
            << (s->completed ? fmt(s->meanTime(), 1) : "-") << " | " << (s->scored ? fmt(s->meanSuccess() * 100.0, 1) + " %" : "-")
            << " | " << (s->completed ? fmt(s->meanDriven(), 1) : "-") << " | " << s->mapsPassed << " / " << s->mapsTried << " |\n";

    out << "\n## Yontem x harita (ort. sure s / ort. basari / gecen kosu)\n\n| Yontem |";
    for (const auto& m : c.maps) out << " " << m << " |";
    out << "\n|---|";
    for (std::size_t i = 0; i < c.maps.size(); ++i) out << "---|";
    out << "\n";
    for (const Summary& s : c.methods) {
        out << "| " << nameOf(s.algorithm) << " |";
        for (const auto& m : c.maps) {
            const auto it = s.perMap.find(m);
            if (it == s.perMap.end()) { out << " - |"; continue; }
            const Cell& k = it->second;
            out << " " << (k.completed ? fmt(k.meanTime(), 0) + " s" : "bitmedi") << " / "
                << (k.scored ? fmt(k.meanSuccess() * 100.0, 1) + " %" : "-") << " / " << k.passed << "/" << k.runs
                << (k.mapPassed() ? " OK" : "") << " |";
        }
        out << "\n";
    }

    out << "\n## Kosular\n\n| Zaman | Yontem | Harita | Mod | Seed | Sonuc | Sure (s) | Yol (m) | Basari | Yanlis | Kararsiz | Tampon |\n"
        << "|---|---|---|---|---:|---|---:|---:|---:|---:|---:|---:|\n";
    for (const Row& r : rows) {
        if (!mode.empty() && r.mode != mode) continue;
        out << "| " << r.timestamp << " | " << nameOf(r.algorithm) << " | " << r.map << " | " << r.mode << " | " << r.seed << " | "
            << r.finish << " | " << fmt(r.timeS, 1) << " | " << fmt(r.drivenM, 1) << " | "
            << (r.scored ? fmt(r.success * 100.0, 1) + " %" : "-") << " | " << (r.scored ? std::to_string(r.wrong) : "-") << " | "
            << r.undecided << " | " << r.bumps << " |\n";
    }
    return static_cast<bool>(out);
}

} // namespace results
