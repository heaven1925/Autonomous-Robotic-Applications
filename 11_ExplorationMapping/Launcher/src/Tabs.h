// Tabs.h - the "Geçmiş" (run history + replay) and "Haritalar" (obstacle map
// editor) tabs. Included by main.cpp inside its namespace, after drawLive /
// drawSide and the widgets.

// ================================================================= history ==
// Rows of results.csv, newest first, filtered by mode.
std::vector<int> histList() {
    std::vector<int> v;
    for (int i = static_cast<int>(g.rows.size()) - 1; i >= 0; --i) {
        const std::string& mode = g.rows[static_cast<std::size_t>(i)].mode;
        if (g.histFilter == 1 && mode != "sim") continue;
        if (g.histFilter == 2 && mode != "real") continue;
        v.push_back(i);
    }
    return v;
}

fs::path histFile(const results::Row& r, const wchar_t* suffix) {   // suffix: L".replay", L"_prob.txt", L".png"
    fs::path p = fs::path(widen(r.output)).make_preferred();
    const std::wstring stem = p.stem().wstring();
    return p.parent_path() / (stem + suffix);
}

int mapIndex(const std::string& name) {
    for (std::size_t i = 0; i < g.maps.size(); ++i)
        if (g.maps[i].name == widen(name)) return static_cast<int>(i);
    return -1;
}
int methodIndex(const std::string& id) {
    for (std::size_t i = 0; i < Methods.size(); ++i)
        if (Methods[i].id == id) return static_cast<int>(i);
    return 0;
}

void histShow(std::size_t index) {
    g.recIndex = g.rec.frames.empty() ? 0 : std::min(index, g.rec.frames.size() - 1);
    g.recView = g.rec.view(g.recIndex);
}

void histPlay(bool on) {
    if (!g.rec.animated) on = false;
    if (on && g.recT >= g.rec.duration - 1e-6) g.recT = 0.0;   // at the end: start again
    g.recPlaying = on;
    g.recTick = GetTickCount64();
    if (on) SetTimer(g.hwnd, TimerReplay, 33, nullptr);
    else KillTimer(g.hwnd, TimerReplay);
}

void histSelect(int row) {
    histPlay(false);
    g.histRow = row;
    g.rec = replay::Recording{};
    g.recView = live::State{};
    if (row < 0 || row >= static_cast<int>(g.rows.size())) return;
    const results::Row& r = g.rows[static_cast<std::size_t>(row)];
    g.rec = replay::load(histFile(r, L".replay"));
    if (!g.rec.loaded) {   // no recording: the final grid from <map>_prob.txt
        const int m = mapIndex(r.map);
        if (m >= 0) {
            const astar::GridMap& grid = g.maps[static_cast<std::size_t>(m)].grid;
            g.rec = replay::fromProbFile(histFile(r, L"_prob.txt"), grid.cols(), grid.rows());
        }
        if (g.rec.loaded) {
            live::State& s = g.rec.frames.back().state;
            s.t = s.endTime = r.timeS;
            s.driven = s.endDriven = r.drivenM;
            s.goals = r.goals;
            s.scans = r.scans;
            s.bumps = r.bumps;
            s.completed = r.completed;
            s.finish = r.finish;
            s.saved = r.output;
            g.rec.duration = r.timeS;
        }
    }
    g.recT = g.rec.duration;
    histShow(g.rec.frames.empty() ? 0 : g.rec.frames.size() - 1);
}

// Entering the tab: keep the selection if it still exists, else the newest run.
void openHistory() {
    const std::vector<int> list = histList();
    if (g.histRow >= 0 && g.histRow < static_cast<int>(g.rows.size()) && g.rec.loaded) return;
    histSelect(list.empty() ? -1 : list.front());
}

void replayTick() {
    if (!g.recPlaying) return;
    static const double speeds[] = { 1.0, 4.0, 16.0, 64.0 };
    const ULONGLONG now = GetTickCount64();
    g.recT += static_cast<double>(now - g.recTick) / 1000.0 * speeds[std::clamp(g.recSpeed, 0, 3)];
    g.recTick = now;
    if (g.recT >= g.rec.duration) {
        g.recT = g.rec.duration;
        histShow(g.rec.frames.size() - 1);
        histPlay(false);
    } else {
        histShow(g.rec.indexAt(g.recT));
    }
    repaint();
}

// Click / drag on the time bar.
void histSeek() {
    if (!g.rec.animated) return;
    const double frac = std::clamp((g.clickX - g.histBarX) / g.histBarW, 0.0, 1.0);
    g.recT = frac * g.rec.duration;
    histShow(g.rec.indexAt(g.recT));
}

void histOpen(bool picture) {
    if (g.histRow < 0 || g.histRow >= static_cast<int>(g.rows.size())) return;
    const results::Row& r = g.rows[static_cast<std::size_t>(g.histRow)];
    const fs::path target = picture ? histFile(r, L".png") : fs::path(widen(r.output)).make_preferred().parent_path();
    if (fs::exists(target)) ShellExecuteW(g.hwnd, L"open", target.c_str(), nullptr, nullptr, SW_SHOWNORMAL);
}

std::wstring clock(double t) { return fmt(L"%.0f:", std::floor(t / 60.0)) + fmt(L"%02.0f", std::floor(std::fmod(t, 60.0))); }

void drawHistoryTab(Painter& P) {
    const double x0 = 20, w = 380, y = 80, h = BH - 20 - y;
    P.card(x0, y, w, h);
    P.text(L"Koşu geçmişi", x0 + 18, y + 14, 200, 24, 15.0f, col::Text, true);
    const std::vector<int> list = histList();
    P.text(std::to_wstring(list.size()) + L" koşu", x0 + w - 18 - 120, y + 14, 120, 24, 12.0f, col::Muted, false, Align::Right);
    segmented(P, x0 + 16, y + 46, w - 32, 34, { L"Tümü", L"Simülasyon", L"Gerçek robot" }, g.histFilter, Act::HistFilter);

    // List.
    const double ly0 = y + 92, rowH = 48;
    const int visible = 9;
    g.histScroll = std::clamp(g.histScroll, 0, std::max(0, static_cast<int>(list.size()) - visible));
    if (list.empty()) P.text(L"Henüz koşu yok.", x0 + 18, ly0 + 10, w - 36, 24, 13.0f, col::Faint);
    for (int i = 0; i < visible && g.histScroll + i < static_cast<int>(list.size()); ++i) {
        const int row = list[static_cast<std::size_t>(g.histScroll + i)];
        const results::Row& r = g.rows[static_cast<std::size_t>(row)];
        const double ry = ly0 + i * rowH;
        const bool on = row == g.histRow, hv = hovered(Act::HistRow, row);
        if (on || hv) P.round(x0 + 10, ry, w - 26, rowH - 4, 9, on ? col::AccentFaint : col::Hover, on ? col::Accent : 0, 1.4);
        P.text(methodName(r.algorithm) + L"  ·  " + widen(r.map), x0 + 22, ry + 3, 220, 20, 13.0f, on ? col::AccentDark : col::Text, true);
        // "2026-11-24 08:22:10" -> "24.11 08:22"
        std::wstring when = widen(r.timestamp);
        if (when.size() >= 16) when = when.substr(8, 2) + L"." + when.substr(5, 2) + L" " + when.substr(11, 5);
        P.text(when, x0 + w - 28 - 110, ry + 3, 110, 20, 11.5f, col::Muted, false, Align::Right);
        const bool real = r.mode == "real";
        const std::wstring sub = std::wstring(real ? L"gerçek" : L"sim") + L" · " + (r.completed ? L"✓ " : L"■ ") + fmt(L"%.0f s", r.timeS) +
                                 (r.scored ? fmt(L" · %%%.0f", r.success * 100.0) : std::wstring()) + (real ? L"" : L" · seed " + std::to_wstring(r.seed));
        P.text(sub, x0 + 22, ry + 23, 250, 18, 11.5f, r.completed ? col::Muted : col::Amber);
        std::error_code ec;
        if (fs::exists(histFile(r, L".replay"), ec)) P.text(L"▶ kayıt", x0 + w - 28 - 70, ry + 23, 70, 18, 11.0f, col::Accent, true, Align::Right);
        hit(x0 + 10, ry, w - 26, rowH - 4, Act::HistRow, row);
    }
    if (static_cast<int>(list.size()) > visible) {   // scroll position
        const double th = visible * rowH - 4, frac = static_cast<double>(visible) / static_cast<double>(list.size());
        const double pos = static_cast<double>(g.histScroll) / static_cast<double>(list.size());
        P.round(x0 + w - 12, ly0, 4, th, 2, col::Track);
        P.round(x0 + w - 12, ly0 + th * pos, 4, std::max(18.0, th * frac), 2, 0xFF94A3B8);
    }

    // Player.
    const double py = ly0 + visible * rowH + 8;
    P.rect(x0 + 16, py, w - 32, 1, col::Border);
    const bool anim = g.rec.animated;
    button(P, x0 + 16, py + 12, 118, 36, g.recPlaying ? L"⏸  Duraklat" : L"▶  Oynat", anim ? Btn::Primary : Btn::Secondary, Act::HistPlay, 0, anim, 13.0f);
    segmented(P, x0 + 142, py + 12, w - 158, 36, { L"1×", L"4×", L"16×", L"64×" }, g.recSpeed, Act::HistSpeed, anim);
    g.histBarX = x0 + 16;
    g.histBarW = w - 32;
    const double frac = g.rec.duration > 0 ? g.recT / g.rec.duration : 0.0;
    P.progress(g.histBarX, py + 66, g.histBarW, 8, anim ? frac : 0.0, col::Accent);
    if (anim) {
        P.circle(g.histBarX + g.histBarW * frac, py + 70, 7, col::Accent, 0xFFFFFFFF, 2);
        hit(g.histBarX - 6, py + 56, g.histBarW + 12, 28, Act::HistSeek);
    }
    P.text(anim ? clock(g.recT) + L" / " + clock(g.rec.duration) + L"   ·   kare " + std::to_wstring(g.recIndex + 1) + L" / " + std::to_wstring(g.rec.frames.size())
                : (g.rec.loaded ? L"Bu koşunun kare kaydı yok: yalnız son harita." : (g.histRow >= 0 ? L"Bu koşunun dosyaları bulunamadı." : L"")),
           x0 + 16, py + 84, w - 32, 20, 11.5f, col::Muted);
    P.text(L"Kare kaydı launcher'dan başlatılan koşularda tutulur (benchmark hariç). Boşluk: oynat / duraklat.", x0 + 16, py + 106, w - 32, 32, 11.0f,
           col::Faint, false, Align::Left, true, false);
    const double bw = (w - 32 - 8) / 2.0;
    button(P, x0 + 16, y + h - 50, bw, 34, L"Resmi aç", Btn::Secondary, Act::HistPicture, 0, g.histRow >= 0, 12.5f);
    button(P, x0 + 24 + bw, y + h - 50, bw, 34, L"Klasörü aç", Btn::Secondary, Act::HistFolder, 0, g.histRow >= 0, 12.5f);

    // The run itself.
    LiveView v;
    v.s = &g.recView;
    v.replay = true;
    if (g.histRow >= 0 && g.histRow < static_cast<int>(g.rows.size())) {
        const results::Row& r = g.rows[static_cast<std::size_t>(g.histRow)];
        v.show = g.rec.loaded;
        v.map = mapIndex(r.map);
        v.method = methodIndex(r.algorithm);
        v.real = r.mode == "real";
    } else {
        v.map = g.maps.empty() ? -1 : g.map;
        v.method = g.method;
    }
    drawLive(P, 416, 80, v);
    drawSide(P, 416 + 612, 188, v);
}

// ============================================================ map editor ==
bool builtinMap(const std::wstring& name) { return name == L"map1" || name == L"map2" || name == L"map3" || name == L"map4"; }
fs::path mapsDir() { return g.dir / "obstacleMapReferences"; }

void editSelect(int index) {
    if (g.maps.empty()) return;
    index = std::clamp(index, 0, static_cast<int>(g.maps.size()) - 1);
    const MapRef& m = g.maps[static_cast<std::size_t>(index)];
    g.draft = {};
    g.draft.active = true;
    g.draft.name = m.name;
    g.draft.grid = m.grid;
    g.draft.editable = !builtinMap(m.name);
    g.mapMsg.clear();
}

// A copy of the shown map under the next free name (map5..map8).
void editNew() {
    if (!g.draft.active || static_cast<int>(g.maps.size()) >= MaxMaps) return;
    std::wstring name;
    for (int n = 1; n <= 99 && name.empty(); ++n) {
        const std::wstring cand = L"map" + std::to_wstring(n);
        bool used = false;
        for (const MapRef& m : g.maps) used |= m.name == cand;
        std::error_code ec;
        if (!used && !fs::exists(mapsDir() / cand, ec)) name = cand;
    }
    if (name.empty()) return;
    g.draft.name = name;
    g.draft.grid.name = utf8(name);
    g.draft.isNew = g.draft.dirty = g.draft.editable = true;
    g.mapMsg = L"Yeni harita " + name + L": hücrelere tıklayarak engel ekleyin / kaldırın, sonra Kaydet.";
}

bool editFixed(const astar::GridMap& m, astar::Cell c) {
    const bool border = c.x == 0 || c.y == 0 || c.x == m.cols() - 1 || c.y == m.rows() - 1;
    return border || (m.start && m.start->x == c.x && m.start->y == c.y);
}

// first = the click (decides add / remove); then the drag paints the same value.
void editCell(int index, bool first) {
    if (!g.draft.active) return;
    astar::GridMap& m = g.draft.grid;
    const astar::Cell c = m.cellOf(index);
    if (!g.draft.editable) {
        if (first) g.mapMsg = g.draft.name + L" dersin referans haritasıdır (salt okunur). Değiştirmek için önce \"Yeni harita\" ile kopyalayın.";
        return;
    }
    if (editFixed(m, c)) {
        if (first) g.mapMsg = L"Kenar duvarı ve başlangıç hücresi değiştirilemez.";
        return;
    }
    if (first) g.paintValue = m.occupied(c) ? 0 : 1;
    if (g.paintValue < 0 || m.occupied(c) == (g.paintValue == 1)) return;
    m.setOccupied(c, g.paintValue == 1);
    g.draft.dirty = true;
    g.mapMsg.clear();
}

void reloadMapsKeeping(const std::wstring& runMap) {
    loadMaps();
    g.benchMap.resize(g.maps.size(), true);
    g.map = 0;
    for (std::size_t i = 0; i < g.maps.size(); ++i)
        if (g.maps[i].name == runMap) g.map = static_cast<int>(i);
    g.mapPage = g.map / 4;
}

void editSave() {
    if (!g.draft.active || !g.draft.editable || !g.draft.dirty) return;
    const std::string name = utf8(g.draft.name);
    int runs = 0;
    for (const results::Row& r : g.rows) runs += r.map == name;
    if (!g.draft.isNew && runs > 0 &&
        MessageBoxW(g.hwnd, (g.draft.name + L" ile " + std::to_wstring(runs) + L" kayıtlı koşu var. Harita değişirse o koşuların başarı oranı "
                             L"eski haritaya göre kalır ve karşılaştırma tutarsız olur.\n\nYine de üzerine yazılsın mı?").c_str(),
                    L"Haritayı kaydet", MB_ICONWARNING | MB_OKCANCEL | MB_DEFBUTTON2) != IDOK)
        return;
    std::string error;
    if (!mapfile::save(mapsDir(), name, g.draft.grid, error)) {
        MessageBoxW(g.hwnd, widen(error).c_str(), L"Haritayı kaydet", MB_ICONERROR);
        return;
    }
    const std::wstring saved = g.draft.name;
    const std::wstring runMap = g.maps.empty() ? saved : g.maps[static_cast<std::size_t>(g.map)].name;
    reloadMapsKeeping(runMap);
    editSelect(std::max(0, mapIndex(utf8(saved))));
    g.mapMsg = L"✓ Kaydedildi: obstacleMapReferences\\" + saved + L"\\" + saved + L".txt";
    saveSettings();
}

void editRevert() {
    if (!g.draft.active) return;
    const int i = mapIndex(utf8(g.draft.name));
    editSelect(i >= 0 ? i : g.map);
}

void editDelete() {
    if (!g.draft.active || g.draft.isNew || builtinMap(g.draft.name)) return;
    const std::wstring name = g.draft.name;
    if (name.size() < 4 || name.rfind(L"map", 0) != 0 || name.find_first_not_of(L"0123456789", 3) != std::wstring::npos) return;
    const fs::path dir = mapsDir() / name;
    if (MessageBoxW(g.hwnd, (name + L" silinsin mi?\n\n" + dir.wstring() + L" klasörü kaldırılır. Bu haritayla yapılmış koşular tabloda kalır.").c_str(),
                    L"Haritayı sil", MB_ICONWARNING | MB_OKCANCEL | MB_DEFBUTTON2) != IDOK)
        return;
    std::error_code ec;
    fs::remove_all(dir, ec);
    const std::wstring runMap = g.maps.empty() ? L"" : g.maps[static_cast<std::size_t>(g.map)].name;
    reloadMapsKeeping(runMap);
    editSelect(g.map);
    g.mapMsg = ec ? L"Silinemedi: " + dir.wstring() : L"Silindi: " + name;
    saveSettings();
}

// "Bu haritayla çalıştır": choose it on the run tab.
void editUse() {
    const int i = mapIndex(utf8(g.draft.name));
    if (i < 0 || g.draft.isNew) return;
    g.map = i;
    g.mapPage = i / 4;
    g.tab = 0;
    g.haveLive = false;
    saveSettings();
}

void drawMapsTab(Painter& P) {
    const double x0 = 20, w = 380, y = 80, h = BH - 20 - y;
    if (!g.draft.active && !g.maps.empty()) editSelect(g.map);
    P.card(x0, y, w, h);
    P.text(L"Engel haritaları", x0 + 18, y + 14, 200, 24, 15.0f, col::Text, true);
    P.text(std::to_wstring(g.maps.size()) + L" / " + std::to_wstring(MaxMaps), x0 + w - 18 - 80, y + 14, 80, 24, 12.0f, col::Muted, false, Align::Right);

    // Tiles: saved maps, then the unsaved new one.
    const double tw = (w - 32 - 3 * 8) / 4.0, th = 84;
    const std::size_t count = g.maps.size() + (g.draft.isNew ? 1 : 0);
    for (std::size_t i = 0; i < count && i < static_cast<std::size_t>(MaxMaps); ++i) {
        const bool isDraft = i >= g.maps.size();
        const std::wstring& name = isDraft ? g.draft.name : g.maps[i].name;
        const astar::GridMap& grid = isDraft || name == g.draft.name ? g.draft.grid : g.maps[i].grid;
        const double tx = x0 + 16 + static_cast<double>(i % 4) * (tw + 8), ty = y + 50 + static_cast<double>(i / 4) * (th + 8);
        const bool on = name == g.draft.name;
        const bool hv = !isDraft && hovered(Act::EditSelect, static_cast<int>(i));
        P.round(tx, ty, tw, th, 10, on ? col::AccentFaint : (hv ? col::Hover : col::Card), on ? col::Accent : col::Border, on ? 1.6 : 1.0);
        miniMap(P, grid, tx + 6, ty + 6, tw - 12, 54);
        P.text(name + (on && g.draft.dirty ? L" ●" : L""), tx, ty + 61, tw, 20, 12.0f, on ? col::AccentDark : col::Text, true, Align::Center);
        if (!isDraft) hit(tx, ty, tw, th, Act::EditSelect, static_cast<int>(i));
    }
    double by = y + 50 + 2 * (th + 8) + 8;
    P.text(L"map1 – map4 dersin referans haritalarıdır (görselleriyle birlikte, salt okunur). Kendi düzeniniz için birini kopyalayıp değiştirin.",
           x0 + 18, by, w - 36, 50, 11.5f, col::Muted, false, Align::Left, true, false);
    by += 60;
    const bool full = static_cast<int>(g.maps.size()) >= MaxMaps;
    button(P, x0 + 16, by, w - 32, 40, full ? L"En çok 8 harita" : L"+  Yeni harita (" + g.draft.name + L" kopyası)", Btn::Secondary, Act::EditNew, 0,
           g.draft.active && !g.draft.isNew && !full, 13.0f);
    by += 48;
    const double bw = (w - 32 - 8) / 2.0;
    button(P, x0 + 16, by, bw, 40, L"Kaydet", g.draft.dirty ? Btn::Primary : Btn::Secondary, Act::EditSave, 0, g.draft.editable && g.draft.dirty, 13.0f);
    button(P, x0 + 24 + bw, by, bw, 40, g.draft.isNew ? L"Vazgeç" : L"Geri al", Btn::Secondary, Act::EditRevert, 0, g.draft.dirty, 13.0f);
    by += 48;
    button(P, x0 + 16, by, bw, 40, L"Bu haritayla çalıştır", Btn::Secondary, Act::EditUse, 0, g.draft.active && !g.draft.isNew && !g.draft.dirty, 12.5f);
    button(P, x0 + 24 + bw, by, bw, 40, L"Sil", Btn::Secondary, Act::EditDelete, 0, g.draft.active && !g.draft.isNew && !builtinMap(g.draft.name), 13.0f);
    by += 52;
    if (!g.mapMsg.empty())
        P.text(g.mapMsg, x0 + 18, by, w - 36, 60, 12.0f, g.mapMsg[0] == L'✓' ? col::Green : col::Amber, false, Align::Left, true, false);
    P.text(L"Kaydedilen harita Çalıştır ve Karşılaştırma sekmelerinde seçilebilir. Laboratuvarda blokları aynı düzende kurmayı unutmayın.", x0 + 18,
           y + h - 56, w - 36, 44, 11.0f, col::Faint, false, Align::Left, true, false);

    // Editor.
    const double ex = 416, ew = BW - 20 - ex;
    P.card(ex, y, ew, h);
    if (!g.draft.active) return;
    const astar::GridMap& m = g.draft.grid;
    P.text(g.draft.name, ex + 20, y + 14, 200, 24, 17.0f, col::Text, true);
    const std::wstring state = !g.draft.editable ? L"salt okunur (referans)" : (g.draft.dirty ? L"● kaydedilmedi" : L"kaydedildi");
    P.pill(state, ex + 20 + P.measure(g.draft.name, 17.0f, true) + 14, y + 14, 24, !g.draft.editable ? col::Muted : (g.draft.dirty ? col::Amber : col::Green),
           !g.draft.editable ? col::Hover : (g.draft.dirty ? col::AmberSoft : col::GreenSoft), 11.5f);
    P.text(g.draft.editable ? L"Hücreye tıklayın: engel ekle / kaldır · basılı tutup sürükleyin: boyayın" : L"Düzenlemek için soldan \"Yeni harita\" ile kopyalayın",
           ex + 20, y + 42, 560, 18, 12.0f, col::Muted);

    const int cols = m.cols(), rows = m.rows();
    const double areaX = ex + 50, areaY = y + 76, areaW = 560, areaH = h - 76 - 64;
    const double c = std::floor(std::min(areaW / cols, areaH / rows));
    const double gx = areaX + (areaW - c * cols) / 2, gy = areaY;
    const std::vector<char> lost = mapfile::unreachableMask(m);
    for (int yy = 0; yy < rows; ++yy)
        for (int xx = 0; xx < cols; ++xx) {
            const astar::Cell k{ xx, yy };
            const double cx = gx + xx * c, cy = gy + yy * c;
            const bool occ = m.occupied(k), fixed = editFixed(m, k);
            const bool hv = g.draft.editable && !fixed && hovered(Act::EditCell, m.index(k));
            const bool un = lost[static_cast<std::size_t>(m.index(k))] != 0;
            ARGB fill = occ ? (fixed ? 0xFF9F1818 : col::Obstacle) : (un ? col::AmberSoft : 0xFFFFFFFF);
            if (hv) fill = occ ? 0xFFE04848 : col::AccentSoft;
            P.rect(cx, cy, c, c, fill);
            P.text(std::to_wstring(m.coords.colLabel(xx)) + L"," + std::to_wstring(m.coords.rowLabel(yy)), cx, cy, c, c, 11.5f,
                   occ ? 0xFFFFFFFF : (un ? col::Amber : col::Faint), false, Align::Center);
            hit(cx, cy, c, c, Act::EditCell, m.index(k));
        }
    for (int xx = 0; xx <= cols; ++xx) P.line(gx + xx * c, gy, gx + xx * c, gy + rows * c, 0x40334155, 1);
    for (int yy = 0; yy <= rows; ++yy) P.line(gx, gy + yy * c, gx + cols * c, gy + yy * c, 0x40334155, 1);
    P.round(gx - 1, gy - 1, cols * c + 2, rows * c + 2, 3, 0, 0xFF334155, 1.5);
    for (int xx = 0; xx < cols; ++xx)
        P.text(std::to_wstring(m.coords.colLabel(xx)), gx + xx * c, gy + rows * c + 4, c, 18, 12.0f, col::Muted, true, Align::Center);
    for (int yy = 0; yy < rows; ++yy)
        P.text(std::to_wstring(m.coords.rowLabel(yy)), gx - 30, gy + yy * c, 24, c, 12.0f, col::Muted, true, Align::Right);
    if (m.start) P.text(L"★", gx + m.start->x * c + 3, gy + m.start->y * c + 2, 18, 18, 12.0f, col::Green, true);

    // Summary.
    const mapfile::Check k = mapfile::check(m);
    const double sx = ex + 50 + 560 + 30, sw = ex + ew - 20 - sx;
    double sy = y + 76;
    P.text(L"Özet", sx, sy, sw, 22, 14.0f, col::Text, true);
    sy += 30;
    const auto row = [&](const std::wstring& label, const std::wstring& value, ARGB colr) {
        P.text(label, sx, sy, sw - 70, 24, 12.5f, col::Muted);
        P.text(value, sx + sw - 70, sy, 70, 24, 14.0f, colr, true, Align::Right);
        sy += 28;
        P.rect(sx, sy - 3, sw, 1, 0xFFF1F5F9);
    };
    row(L"Alan", std::to_wstring(cols) + L" × " + std::to_wstring(rows), col::Text);
    row(L"Hücre", fmt(L"%.0f cm", m.cellSize() * 100.0), col::Text);
    row(L"İç engel", std::to_wstring(k.inner), col::Text);
    row(L"Boş hücre", std::to_wstring(k.freeCells), col::Text);
    row(L"Görülebilir hücre", std::to_wstring(k.observable), col::Text);
    row(L"Ulaşılamayan boş hücre", std::to_wstring(k.unreachable), k.unreachable ? col::Amber : col::Green);
    sy += 8;
    if (k.unreachable)
        P.text(L"! Sarı hücrelere robot başlangıçtan gidemez. Yöntemler onları kararsız bırakabilir; başarı oranı düşer.", sx, sy, sw, 66, 12.0f,
               col::Amber, false, Align::Left, true, false);
    else
        P.text(L"✓ Bütün boş hücrelere başlangıçtan ulaşılabiliyor.", sx, sy, sw, 40, 12.0f, col::Green, false, Align::Left, true, false);
    sy += 76;
    P.text(L"Görülebilir hücre: boş, ya da boş bir komşusu olan engel. Başarı oranı bu hücreler üzerinden hesaplanır.", sx, sy, sw, 66, 11.5f, col::Faint,
           false, Align::Left, true, false);
    // Legend.
    double lx = ex + 50;
    const double lgy = y + h - 32;
    const auto item = [&](ARGB colr, const std::wstring& label) {
        P.round(lx, lgy + 3, 14, 14, 3, colr, col::Border);
        P.text(label, lx + 20, lgy, 160, 20, 12.0f, col::Muted);
        lx += 32 + P.measure(label, 12.0f);
    };
    item(0xFF9F1818, L"kenar duvarı (sabit)");
    item(col::Obstacle, L"engel");
    item(0xFFFFFFFF, L"boş");
    item(col::AmberSoft, L"ulaşılamaz");
    P.text(L"★ başlangıç (sabit)", lx, lgy, 200, 20, 12.0f, col::Muted);
}
