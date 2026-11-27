// Panels.h - the panels the run tab shows on its right side for the real robot
// (instead of the map) : sensor calibration (step 3), manual drive + sensor
// test, pre-flight check. Included by main.cpp inside its namespace, after the
// widgets (button, dropdown, tile, ...).

// ====================================================== sensor calibration ==
// The last readings (calibrated cm if there is a calibration), 40 cm dashed.
void readingChart(Painter& P, double x, double y, double w, double h) {
    P.round(x, y, w, h, 10, 0xFFF8FAFC, col::Border);
    const sensor::Snapshot& s = g.sens;
    constexpr std::uint64_t Span = 120;
    const std::uint64_t first = s.seq > Span ? s.seq - Span + 1 : 1;
    std::vector<std::pair<std::uint64_t, double>> pts;
    for (const auto& smp : s.recent)
        if (smp.seq >= first && smp.echo()) pts.push_back({ smp.seq, g.cal.valid ? g.cal.cm(rawCm(smp)) : rawCm(smp) });
    if (pts.empty()) {
        P.text(streaming() ? L"Yankı yok (sensörün önü boş ya da > 4 m)" : L"Okuma yok", x, y, w, h, 12.5f, col::Faint, false, Align::Center);
        return;
    }
    double lo = rangecal::ReferenceCm, hi = rangecal::ReferenceCm;
    for (const auto& pt : pts) { lo = std::min(lo, pt.second); hi = std::max(hi, pt.second); }
    const double span = std::max(20.0, hi - lo);
    const double step = span > 120 ? 50.0 : (span > 50 ? 20.0 : 10.0);
    lo = std::floor((lo - span * 0.1) / step) * step;
    hi = std::ceil((hi + span * 0.1) / step) * step;
    lo = std::max(0.0, lo);
    const double px = x + 50, pw = w - 62, py = y + 12, ph = h - 24;
    const auto X = [&](std::uint64_t q) { return px + pw * static_cast<double>(q - first) / static_cast<double>(Span - 1); };
    const auto Y = [&](double v) { return py + ph * (1.0 - (v - lo) / (hi - lo)); };
    for (double v = lo; v <= hi + 1e-9; v += step) {
        P.line(px, Y(v), px + pw, Y(v), 0x22334155, 1);
        P.text(fmt(L"%.0f cm", v), x + 2, Y(v) - 9, 44, 18, 10.5f, col::Faint, false, Align::Right);
    }
    {
        Gdiplus::Pen pen(Gdiplus::Color(col::Amber), P.S(1.6));
        pen.SetDashStyle(Gdiplus::DashStyleDash);
        P.g.DrawLine(&pen, P.S(px), P.S(Y(rangecal::ReferenceCm)), P.S(px + pw), P.S(Y(rangecal::ReferenceCm)));
    }
    Gdiplus::Pen pen(Gdiplus::Color(col::Accent), P.S(2.2));
    pen.SetLineJoin(Gdiplus::LineJoinRound);
    for (std::size_t i = 1; i < pts.size(); ++i)
        if (pts[i].first == pts[i - 1].first + 1)
            P.g.DrawLine(&pen, P.S(X(pts[i - 1].first)), P.S(Y(pts[i - 1].second)), P.S(X(pts[i].first)), P.S(Y(pts[i].second)));
    P.circle(X(pts.back().first), Y(pts.back().second), 4.5, col::Accent, 0xFFFFFFFF, 1.5);
}

// Top view: robot, sensor, 40 cm, block (to scale, 2.6 px per cm).
void calDiagram(Painter& P, double x, double y, double w, double h) {
    P.round(x, y, w, h, 10, 0xFFF8FAFC, col::Border);
    const double k = 2.6, cy = y + h / 2 + 4;
    const double robotR = 17.5 * k, rcx = x + 22 + robotR;
    P.circle(rcx, cy, robotR, 0xFFE2E8F0, 0xFF64748B, 1.5);
    P.text(L"Kobuki", rcx - robotR, cy - 10, robotR * 1.3, 20, 11.0f, col::Muted, true, Align::Center);
    const double sx = rcx + 11.0 * k;   // sensor face, 11 cm ahead of the centre
    P.round(sx - 7, cy - 14, 7, 28, 2, col::Accent);
    P.text(L"HC-SR04", sx - 40, cy - 40, 80, 18, 10.5f, col::Accent, true, Align::Center);
    const double bx = sx + rangecal::ReferenceCm * k, bs = 40.0 * k;
    P.round(bx, cy - bs / 2, bs, bs, 4, col::Obstacle);
    P.text(L"blok", bx, cy - 18, bs, 18, 12.0f, 0xFFFFFFFF, true, Align::Center);
    P.text(L"40 × 40 cm", bx, cy, bs, 18, 11.0f, 0xFFFFFFFF, false, Align::Center);
    P.line(sx + 3, cy, bx - 3, cy, col::Text, 1.6);
    P.line(sx + 3, cy, sx + 11, cy - 5, col::Text, 1.6);
    P.line(sx + 3, cy, sx + 11, cy + 5, col::Text, 1.6);
    P.line(bx - 3, cy, bx - 11, cy - 5, col::Text, 1.6);
    P.line(bx - 3, cy, bx - 11, cy + 5, col::Text, 1.6);
    P.text(L"40 cm", sx, cy - 26, bx - sx, 20, 13.0f, col::Text, true, Align::Center);
    P.text(L"üstten görünüş, ölçekli", x + w - 160, y + 6, 150, 16, 10.5f, col::Faint, false, Align::Right);
}

// Output of arduino-cli / the setup script, in place of the live readings.
void uploadLog(Painter& P, double x, double y, double w, double h) {
    P.card(x, y, w, h);
    const bool setup = g.up.setup;
    P.text(setup ? L"arduino-cli kurulumu" : L"Sketch yükleme", x + 20, y + 14, 300, 22, 15.0f, col::Text, true);
    std::wstring st = g.up.running ? L"Çalışıyor..." : (g.up.ok ? L"✓ Tamamlandı" : L"✗ Başarısız");
    const ARGB fg = g.up.running ? col::Accent : (g.up.ok ? col::Green : col::Red);
    const ARGB bg = g.up.running ? col::AccentSoft : (g.up.ok ? col::GreenSoft : col::RedSoft);
    const double pw = P.measure(st, 12.5f, true) + 18;
    P.pill(st, x + w - 20 - pw, y + 14, 24, fg, bg);
    const double ly0 = y + 48, lh = 17;
    const double area = h - 48 - 60;
    P.round(x + 20, ly0, w - 40, area, 8, 0xFF0F172A);
    const auto fit = static_cast<std::size_t>((area - 12) / lh);
    const std::size_t from = g.up.log.size() > fit ? g.up.log.size() - fit : 0;
    double ly = ly0 + 6;
    for (std::size_t i = from; i < g.up.log.size(); ++i) {
        const std::wstring& l = g.up.log[i];
        const bool cmd = l.rfind(L"> ", 0) == 0;
        P.text(l, x + 30, ly, w - 60, lh, 10.5f, cmd ? 0xFF93C5FD : 0xFFE2E8F0);
        ly += lh;
    }
    if (g.up.running) {
        P.text(setup ? L"İndiriliyor; birkaç dakika sürebilir." : L"Derleniyor ve karta yazılıyor; kartı ve kabloyu oynatmayın.", x + 20, y + h - 48, w - 160,
               36, 11.5f, col::Muted, false, Align::Left, true);
        button(P, x + w - 20 - 110, y + h - 48, 110, 34, L"İptal", Btn::Secondary, Act::UploadCancel, 0, true, 12.5f);
    } else {
        P.text(g.up.ok ? (setup ? L"arduino-cli hazır. Şimdi Sketch'i yükle'ye basabilirsiniz." : L"Kart yeniden başladı; port yeniden test ediliyor.")
                       : (setup ? L"Kurulum tamamlanamadı (internet bağlantısı?)." : L"Kart türü ve port doğru mu? Port başka programda açık olabilir."),
               x + 20, y + h - 48, w - 160, 36, 11.5f, g.up.ok ? col::Muted : col::Red, false, Align::Left, true);
        button(P, x + w - 20 - 110, y + h - 48, 110, 34, L"Kapat", Btn::Secondary, Act::UploadClose, 0, true, 12.5f);
    }
}

void drawCalibration(Painter& P, double x0, double y0) {
    const double W = BW - 20 - x0;
    const sensor::Snapshot& s = g.sens;
    const sensor::Sample* last = lastSample();

    // Header.
    P.card(x0, y0, W, 76);
    P.text(L"Sensör kalibrasyonu", x0 + 20, y0 + 10, 400, 28, 17.0f, col::Text, true);
    P.text(L"HC-SR04 (ön) · Arduino " + g.arduino.port + L" · bloklar 40 × 40 cm = 1 hücre", x0 + 20, y0 + 42, 520, 20, 12.0f, col::Muted);
    std::wstring lt;
    ARGB fg = col::Accent, bg = col::AccentSoft;
    bool retry = false;
    if (g.up.running) lt = L"Port yüklemeye bırakıldı";
    else switch (s.state) {
        case sensor::State::Opening: lt = L"Port açılıyor..."; break;
        case sensor::State::Waiting:
            lt = s.error.empty() ? L"Veri bekleniyor..." : L"✗ " + s.error;
            if (!s.error.empty()) { fg = col::Red; bg = col::RedSoft; retry = true; }
            break;
        case sensor::State::Streaming: lt = L"● Canlı · " + fmt(L"%.0f Hz", s.hz); fg = col::Green; bg = col::GreenSoft; break;
        case sensor::State::Error: lt = L"✗ " + s.error; fg = col::Red; bg = col::RedSoft; retry = true; break;
        default: lt = g.probing ? L"Port test ediliyor..." : L"Kapalı"; fg = col::Muted; bg = col::Hover; break;
    }
    double hx = x0 + W - 20 - 150;
    button(P, hx, y0 + 20, 150, 36, L"←  Haritaya dön", Btn::Secondary, Act::CalClose, 0, !g.up.running, 13.0f);
    if (retry) { hx -= 140; button(P, hx, y0 + 20, 130, 36, L"Yeniden bağlan", Btn::Secondary, Act::LinkRetry, 0, true, 12.5f); }
    const double pw = P.measure(lt, 12.5f, true) + 18;
    P.pill(lt, hx - 12 - pw, y0 + 25, 26, fg, bg);

    const double cy = y0 + 76 + 12, lw = 452, rx = x0 + lw + 12, rw = W - lw - 12;

    // A. Pins + sketch.
    const double ah = 252;
    P.card(x0, cy, lw, ah);
    P.text(L"Pin bağlantısı", x0 + 20, cy + 14, 300, 22, 15.0f, col::Text, true);
    P.text(L"HC-SR04'ün bağlı olduğu Arduino dijital pinleri", x0 + 20, cy + 38, lw - 40, 18, 12.0f, col::Muted);
    if (g.trig != DefaultTrig || g.echo != DefaultEcho)
        button(P, x0 + lw - 20 - 116, cy + 12, 116, 28, L"Varsayılana al", Btn::Secondary, Act::DefaultPins, 0, !g.pinWait, 12.0f);
    else
        P.text(L"varsayılan D11 / D12", x0 + lw - 20 - 160, cy + 14, 160, 22, 11.5f, col::Faint, false, Align::Right);
    P.text(L"Trig", x0 + 20, cy + 66, 40, 36, 13.5f, col::Text, true);
    dropdown(P, x0 + 60, cy + 66, 92, 36, L"D" + std::to_wstring(g.trig), false, Act::TrigPin, 0, !g.pinWait && !g.up.running);
    P.text(L"Echo", x0 + 168, cy + 66, 44, 36, 13.5f, col::Text, true);
    dropdown(P, x0 + 212, cy + 66, 92, 36, L"D" + std::to_wstring(g.echo), false, Act::EchoPin, 0, !g.pinWait && !g.up.running);
    button(P, x0 + 320, cy + 66, lw - 340, 36, g.pinWait ? L"Yazılıyor..." : L"Karta yaz", g.pinsDirty ? Btn::Primary : Btn::Secondary,
           Act::WritePins, 0, s.cfgKnown && g.pinsDirty && !g.pinWait, 13.0f);
    std::wstring st;
    ARGB sc = col::Muted;
    bool wantsSketch = g.arduino.r.kind == probe::Kind::WrongSketch;
    if (!g.pinMsg.empty()) {
        st = g.pinMsg;
        sc = g.pinWait ? col::Accent : (g.pinOk ? col::Green : col::Red);
    } else if (s.cfgKnown) {
        if (g.pinsDirty) { st = L"Seçim karttakinden farklı - Karta yaz'a basın."; sc = col::Amber; }
        else {
            st = L"✓ Kartta kayıtlı: Trig D" + std::to_wstring(s.trig) + L" · Echo D" + std::to_wstring(s.echo) + L"  (sensor.ino v" +
                 std::to_wstring(s.ver) + L")";
            sc = col::Green;
        }
    } else if (s.cfgUnsupported) {
        st = L"! Kart pin komutlarını tanımıyor (eski sensor.ino). Aşağıdan yeni sketch'i yükleyin.";
        sc = col::Amber;
        wantsSketch = true;
    } else if (streaming()) {
        st = L"Karttaki pin ayarı soruluyor...";
    } else if (wantsSketch) {
        st = L"! Kartta başka bir program var. Aşağıdan sensor.ino'yu yükleyin.";
        sc = col::Amber;
    } else {
        st = L"Kart bağlı değil.";
        sc = col::Faint;
    }
    P.text(st, x0 + 20, cy + 108, lw - 40, 36, 12.0f, sc, false, Align::Left, true, false);
    // Sketch upload.
    P.rect(x0 + 20, cy + 150, lw - 40, 1, col::Border);
    const bool haveCli = !cliPath().empty();
    P.text(L"Kart", x0 + 20, cy + 162, 40, 36, 13.5f, col::Text, true);
    dropdown(P, x0 + 60, cy + 162, 196, 36, arduino::boards()[static_cast<std::size_t>(g.board)].name, false, Act::Board, 0, !g.up.running);
    button(P, x0 + 268, cy + 162, lw - 288, 36, g.up.running ? L"Yükleniyor..." : (haveCli ? L"Sketch'i yükle" : L"arduino-cli'yi kur"),
           wantsSketch && haveCli ? Btn::Primary : Btn::Secondary, Act::Upload, 0, !g.up.running && !g.pinWait && !g.calRun.active, 13.0f);
    P.text(haveCli ? L"sensor.ino (v2) arduino-cli ile derlenir ve karta yazılır; pin ayarı EEPROM'da kalır."
                   : L"Sketch yüklemek için arduino-cli gerekir; düğme onu indirir (~180 MB). Arduino IDE kuruluysa onunki kullanılır.",
           x0 + 20, cy + 204, lw - 40, 36, 11.5f, col::Faint, false, Align::Left, true, false);

    // B. Live reading (or the upload log).
    const double by = cy + ah + 12, bh = BH - 20 - by;
    if (g.up.show) {
        uploadLog(P, x0, by, lw, bh);
    } else {
        P.card(x0, by, lw, bh);
        P.text(L"Anlık okuma", x0 + 20, by + 14, 300, 22, 15.0f, col::Text, true);
        if (s.seq) {
            int noEcho = 0, n = 0;
            for (auto it = s.recent.rbegin(); it != s.recent.rend() && n < 100; ++it, ++n) noEcho += !it->echo();
            P.text(fmt(L"%.0f Hz · ", s.hz) + std::to_wstring(s.seq) + L" ölçüm · yankısız %" + std::to_wstring(n ? noEcho * 100 / n : 0),
                   x0 + 200, by + 14, lw - 220, 22, 11.5f, col::Muted, false, Align::Right);
        }
        const double tw = (lw - 40 - 16) / 3.0, ty = by + 44;
        const bool echo = last && last->echo();
        std::wstring usText = L"—", usSub = L"yankı süresi";
        if (last && last->us >= 0) usText = fmt(L"%.0f µs", static_cast<double>(last->us));
        else if (last && last->us == -1) usText = L"yankı yok";
        else if (last) usSub = L"eski sketch: yok";
        tile(P, x0 + 20, ty, tw, 84, L"HAM DEĞER", usText, usSub, col::Text);
        tile(P, x0 + 20 + tw + 8, ty, tw, 84, L"HAM MESAFE", echo ? fmt(L"%.1f cm", rawCm(*last)) : L"—", L"343 m/s ile", col::Text);
        tile(P, x0 + 20 + 2 * (tw + 8), ty, tw, 84, L"KALİBRELİ", echo && g.cal.valid ? fmt(L"%.1f cm", g.cal.cm(rawCm(*last))) : L"—",
             g.cal.valid ? fmt(L"ham × %.4f", g.cal.scale) : L"kalibre edilmedi", g.cal.valid ? col::Green : col::Faint);
        readingChart(P, x0 + 20, ty + 94, lw - 40, bh - (ty + 94 - by) - 36);
        P.text(g.cal.valid ? L"Çizgi: kalibreli mesafe (son 12 s) · kesikli: 40 cm" : L"Çizgi: ham mesafe (son 12 s) · kesikli: 40 cm",
               x0 + 20, by + bh - 30, lw - 40, 20, 11.0f, col::Faint);
    }

    // C. 40 cm calibration.
    const double ch = BH - 20 - cy;
    P.card(rx, cy, rw, ch);
    P.text(L"40 cm kalibrasyonu", rx + 20, cy + 14, 300, 22, 15.0f, col::Text, true);
    P.text(L"Bilinen bir mesafeyle (1 blok = 1 hücre = 40 cm) ham okumayı cm'ye bağlar.", rx + 20, cy + 38, rw - 40, 18, 12.0f, col::Muted);
    calDiagram(P, rx + 20, cy + 64, rw - 40, 136);
    const wchar_t* steps[] = {
        L"Robotu düz zemine koyun; sensör yere paralel ve tam ileri baksın.",
        L"40 × 40 cm bloğu, düz yüzü sensöre dik gelecek şekilde sensörün ön yüzünden tam 40 cm uzağa koyun.",
        L"Okuma sabitlenince başlatın: 50 ölçüm alınır (~5 s), ortalaması 40 cm'ye eşlenir.",
    };
    double sy = cy + 212;
    for (int i = 0; i < 3; ++i) {
        P.circle(rx + 30, sy + 10, 10, col::AccentSoft);
        P.text(std::to_wstring(i + 1), rx + 20, sy, 20, 20, 11.0f, col::AccentDark, true, Align::Center);
        P.text(steps[i], rx + 48, sy + 1, rw - 68, 36, 12.0f, col::Text, false, Align::Left, true, false);
        sy += 40;
    }
    sy += 4;
    const bool echoNow = last && last->echo();
    const bool measuring = g.calRun.active && !g.calRun.verify;
    if (measuring) {
        const double frac = static_cast<double>(g.calRun.raw.size()) / CalSamples;
        P.text(L"Ölçülüyor: " + std::to_wstring(g.calRun.raw.size()) + L" / " + std::to_wstring(CalSamples) +
                   (g.calRun.noEcho ? L"   (yankısız " + std::to_wstring(g.calRun.noEcho) + L")" : L""),
               rx + 20, sy, rw - 160, 22, 13.0f, col::Accent, true);
        P.progress(rx + 20, sy + 28, rw - 160, 10, frac, col::Accent);
        button(P, rx + rw - 20 - 110, sy + 4, 110, 38, L"İptal", Btn::Secondary, Act::CalCancel, 0, true, 13.0f);
    } else {
        const bool can = streaming() && echoNow && !g.calRun.active && !g.up.running;
        button(P, rx + 20, sy, rw - 40, 46, g.cal.valid ? L"▶   Yeniden kalibre et (40 cm)" : L"▶   40 cm kalibrasyonunu başlat",
               g.cal.valid && !calStale() && !calPinsChanged() ? Btn::Secondary : Btn::Primary, Act::CalStart, 0, can, 14.0f);
        if (!can && !g.calRun.active)
            P.text(g.up.running ? L"Yükleme sürüyor." : (!streaming() ? L"Arduino verisi bekleniyor (2. adımda Arduino ✓ olmalı)." : L"Sensör yankı almıyor: blok önünde mi?"),
                   rx + 20, sy + 50, rw - 40, 18, 11.5f, col::Amber);
    }

    // Result.
    const double ry = sy + 72, rh = cy + ch - 16 - ry;
    if (!g.calError.empty()) {
        P.round(rx + 20, ry, rw - 40, rh, 10, col::RedSoft);
        P.text(L"✗ Kalibrasyon başarısız", rx + 36, ry + 12, rw - 72, 22, 14.0f, col::Red, true);
        P.text(g.calError, rx + 36, ry + 40, rw - 72, 60, 12.0f, col::Text, false, Align::Left, true, false);
        if (g.cal.valid)
            P.text(fmt(L"Önceki kalibrasyon geçerli: ham × %.4f  (", g.cal.scale) + widen(g.cal.date) + L")", rx + 36, ry + rh - 34, rw - 72, 20,
                   11.5f, col::Muted);
    } else if (g.cal.valid) {
        const rangecal::Calibration& c = g.cal;
        const bool pins = calPinsChanged(), stale = calStale();
        const bool warn = pins || stale;
        P.round(rx + 20, ry, rw - 40, rh, 10, warn ? col::AmberSoft : col::GreenSoft);
        P.text(pins ? L"! Pinler değişti: yeniden kalibre edin" : (stale ? L"! Kalibrasyon eski: yenileyin" : L"✓ Kalibre edildi"), rx + 36, ry + 10, 280,
               22, 14.0f, warn ? col::Amber : col::Green, true);
        P.text(ageText(calAgeHours()) + L" · " + widen(c.date), rx + rw - 36 - 220, ry + 10, 220, 22, 11.5f, col::Muted, false, Align::Right);
        P.text(L"40 cm  =  " + (c.meanUs > 0 ? fmt(L"%.0f µs", c.meanUs) : fmt(L"%.1f ham cm", c.rawMeanCm)), rx + 36, ry + 34, rw - 72, 30, 20.0f,
               col::Text, true);
        const std::wstring lines[] = {
            fmt(L"Ham ortalama %.2f cm", c.rawMeanCm) + fmt(L"  →  ölçek × %.4f", c.scale) + fmt(L"  ·  ses hızı ≈ %.0f m/s", c.speedOfSound()),
            fmt(L"Yayılım σ %.2f cm · ", c.sigmaCm) + std::to_wstring(c.samples) + L" ölçüm · " + std::to_wstring(c.rejected) + L" aykırı atıldı · Trig D" +
                std::to_wstring(c.trig) + L" / Echo D" + std::to_wstring(c.echo),
        };
        double ly = ry + 68;
        for (const auto& l : lines) {
            P.text(l, rx + 36, ly, rw - 72, 20, 11.5f, col::Text);
            ly += 20;
        }
        // "Kontrol et": is it still right? Block at 40 or 80 cm.
        const double vy = ry + rh - 76;
        P.rect(rx + 36, vy, rw - 72, 1, warn ? 0x40D97706 : 0x4016A34A);
        P.text(L"Kontrol", rx + 36, vy + 8, 60, 32, 12.5f, col::Text, true);
        segmented(P, rx + 96, vy + 8, 150, 32, { L"40 cm", L"80 cm" }, g.verifyRef == 40 ? 0 : 1, Act::VerifyRef, !g.calRun.active);
        const bool verifying = g.calRun.active && g.calRun.verify;
        button(P, rx + 256, vy + 8, rw - 292, 32, verifying ? L"Ölçülüyor " + std::to_wstring(g.calRun.raw.size()) + L"/" + std::to_wstring(VerifySamples)
                                                              : L"Bloğu koyup kontrol et",
               Btn::Secondary, Act::Verify, 0, streaming() && echoNow && !g.calRun.active && !g.up.running, 12.5f);
        if (g.verify.have)
            P.text((g.verify.ok ? L"✓ " : L"✗ ") + fmt(L"%.0f cm'de ", g.verify.ref) + fmt(L"%.1f cm · sapma ", g.verify.mean) +
                       fmt(L"%+.1f cm", g.verify.err) + (g.verify.ok ? L" · geçerli" : L" · yeniden kalibre edin"),
                   rx + 36, vy + 46, rw - 72, 20, 12.0f, g.verify.ok ? col::Green : col::Red, true);
        else
            P.text(L"Bloğu 40 ya da 80 cm'ye koyun: 20 ölçümle sapma gösterilir, kayıt değişmez.", rx + 36, vy + 46, rw - 72, 20, 11.0f, col::Muted);
    } else {
        P.round(rx + 20, ry, rw - 40, rh, 10, col::Hover);
        P.text(L"Henüz kalibre edilmedi", rx + 36, ry + 12, rw - 72, 22, 14.0f, col::Muted, true);
        P.text(L"Programlar ham HC-SR04 mesafelerini kullanır (ölçek × 1). Kalibrasyon " + g.calFile.filename().wstring() +
                   L" dosyasına yazılır; dört yöntem de gerçek robotta onu okur.",
               rx + 36, ry + 40, rw - 72, 60, 12.0f, col::Muted, false, Align::Left, true, false);
    }
}

// ============================================================ manual drive ==
// Called every 50 ms while the panel is open: refresh the hold-to-drive
// command (Teleop.h stops the robot 300 ms after the last one) and the status.
void driveTick() {
    if (!g.bot.isOpen()) return;
    const bool focus = GetForegroundWindow() == g.hwnd;
    if (!focus) g.driveKeys = g.driveBtn = 0;
    const int k = g.driveKeys | g.driveBtn;
    const double sv = g.driveSpeed == 0 ? 0.08 : 0.15, sw = g.driveSpeed == 0 ? 0.5 : 0.9;
    const double v = ((k & KeyFwd) ? 1.0 : 0.0) - ((k & KeyBack) ? 1.0 : 0.0);
    const double w = ((k & KeyLeft) ? 1.0 : 0.0) - ((k & KeyRight) ? 1.0 : 0.0);
    if (k) g.bot.command(v * sv, w * sw);
    else if (g.driveLast) g.bot.command(0.0, 0.0);   // released: stop now, not after the watchdog
    g.driveLast = k;
    g.drive = g.bot.status();
    repaint();
}

void indicator(Painter& P, double x, double y, double w, const std::wstring& label, bool on, bool seen, ARGB onColour = col::Red) {
    P.round(x, y, w, 40, 9, on ? onColour : (seen ? col::GreenSoft : 0xFFF8FAFC), on ? 0 : col::Border);
    P.text(label, x, y + 3, w, 18, 12.0f, on ? 0xFFFFFFFF : col::Text, true, Align::Center);
    P.text(on ? L"BASILI" : (seen ? L"✓ denendi" : L"—"), x, y + 20, w, 16, 10.5f, on ? 0xFFFFFFFF : (seen ? col::Green : col::Faint), false, Align::Center);
}

void drawDrive(Painter& P, double x0, double y0) {
    const double W = BW - 20 - x0;
    const teleop::Status& d = g.drive;
    const bool open = d.state == teleop::State::Open;

    P.card(x0, y0, W, 76);
    P.text(L"Elle sürüş ve sensör testi", x0 + 20, y0 + 10, 400, 28, 17.0f, col::Text, true);
    P.text(L"Kobuki " + g.kobuki.port + L" · tampon basılınca ileri hareket kesilir · tuş bırakılınca robot durur", x0 + 20, y0 + 42, 600, 20, 12.0f,
           col::Muted);
    std::wstring lt;
    ARGB fg = col::Accent, bg = col::AccentSoft;
    bool retry = false;
    switch (d.state) {
        case teleop::State::Opening: lt = L"Bağlanıyor..."; break;
        case teleop::State::Open:
            if (d.alive) { lt = L"● Bağlı"; fg = col::Green; bg = col::GreenSoft; }
            else { lt = L"✗ Veri kesildi"; fg = col::Red; bg = col::RedSoft; retry = true; }
            break;
        case teleop::State::Error: lt = L"✗ " + d.error; fg = col::Red; bg = col::RedSoft; retry = true; break;
        default: lt = g.probing ? L"Port test ediliyor..." : L"Kapalı"; fg = col::Muted; bg = col::Hover; break;
    }
    double hx = x0 + W - 20 - 150;
    button(P, hx, y0 + 20, 150, 36, L"←  Haritaya dön", Btn::Secondary, Act::DriveOpen, 0, true, 13.0f);
    if (retry) { hx -= 140; button(P, hx, y0 + 20, 130, 36, L"Yeniden bağlan", Btn::Secondary, Act::DriveRetry, 0, true, 12.5f); }
    const double pw = P.measure(lt, 12.5f, true) + 18;
    P.pill(lt, hx - 12 - pw, y0 + 25, 26, fg, bg);

    const double cy = y0 + 88, lw = 452, rx = x0 + lw + 12, rw = W - lw - 12, ch = BH - 20 - cy;

    // Driving.
    P.card(x0, cy, lw, ch);
    P.text(L"Sürüş", x0 + 20, cy + 14, 200, 22, 15.0f, col::Text, true);
    P.text(L"Ok tuşları ya da W A S D: basılı tutun · Boşluk: dur", x0 + 20, cy + 38, lw - 40, 18, 12.0f, col::Muted);
    const int k = g.driveKeys | g.driveBtn;
    const double bs = 72, gap = 8, cx = x0 + lw / 2 - bs / 2, py = cy + 70;
    const auto pad = [&](double x, double y, const wchar_t* label, int bit) {
        const bool on = (k & bit) != 0;
        const bool hv = open && hovered(Act::DriveBtn, bit);
        P.round(x, y, bs, bs, 14, !open ? col::Track : (on ? col::Accent : (hv ? col::Hover : col::Card)), on || !open ? 0 : col::Border, 1.2);
        P.text(label, x, y, bs, bs, 24.0f, !open ? col::Faint : (on ? 0xFFFFFFFF : col::Text), true, Align::Center);
        if (open) hit(x, y, bs, bs, Act::DriveBtn, bit);
    };
    pad(cx, py, L"▲", KeyFwd);
    pad(cx - bs - gap, py + bs + gap, L"◀", KeyLeft);
    pad(cx + bs + gap, py + bs + gap, L"▶", KeyRight);
    pad(cx, py + 2 * (bs + gap), L"▼", KeyBack);
    {
        const double sx = cx, sy = py + bs + gap;
        const bool hv = open && hovered(Act::DriveStop);
        P.round(sx, sy, bs, bs, 14, !open ? col::Track : (hv ? 0xFFB91C1C : col::Red));
        P.text(L"DUR", sx, sy, bs, bs, 14.0f, !open ? col::Faint : 0xFFFFFFFF, true, Align::Center);
        if (open) hit(sx, sy, bs, bs, Act::DriveStop);
    }
    double y = py + 3 * (bs + gap) + 10;
    P.text(L"Hız", x0 + 20, y, 40, 36, 13.0f, col::Text, true);
    segmented(P, x0 + 64, y, lw - 84, 36, { L"Yavaş · 0.08 m/s", L"Normal · 0.15 m/s" }, g.driveSpeed, Act::DriveSpeed, open);
    y += 52;
    P.text(L"Sabit hareketler", x0 + 20, y, 200, 20, 12.5f, col::Muted, true);
    P.text(L"1 hücre = 40 cm", x0 + lw - 20 - 160, y, 160, 20, 11.5f, col::Faint, false, Align::Right);
    y += 26;
    const double bw = (lw - 40 - 8) / 2.0;
    const bool can = open && d.alive;
    button(P, x0 + 20, y, bw, 38, L"İleri 40 cm (1 hücre)", Btn::Secondary, Act::DriveMove, 40, can, 12.5f);
    button(P, x0 + 28 + bw, y, bw, 38, L"Geri 40 cm", Btn::Secondary, Act::DriveMove, -40, can, 12.5f);
    y += 46;
    button(P, x0 + 20, y, bw, 38, L"↺  Sola 90°", Btn::Secondary, Act::DriveTurn, 90, can, 12.5f);
    button(P, x0 + 28 + bw, y, bw, 38, L"↻  Sağa 90°", Btn::Secondary, Act::DriveTurn, -90, can, 12.5f);
    y += 50;
    std::wstring note;
    ARGB nc = col::Muted;
    if (d.goal == teleop::Goal::Move) { note = fmt(L"İlerliyor: kalan %.0f cm", std::abs(d.goalLeft) * 100.0); nc = col::Accent; }
    else if (d.goal == teleop::Goal::Turn) { note = fmt(L"Dönüyor: kalan %.0f°", std::abs(d.goalLeft)); nc = col::Accent; }
    else if (!d.note.empty()) { note = d.note; nc = d.note[0] == L'✓' ? col::Green : col::Amber; }
    else note = L"Sabit hareketler tampon, tekerlek ya da zaman aşımında kendiliğinden durur.";
    P.text(note, x0 + 20, y, lw - 40, 20, 12.0f, nc, d.goal != teleop::Goal::None);
    P.text(fmt(L"Gönderilen komut: %.2f m/s · ", d.v) + fmt(L"%.2f rad/s", d.w), x0 + 20, y + 22, lw - 40, 18, 11.5f, col::Faint);

    // Sensors.
    P.card(rx, cy, rw, ch);
    P.text(L"Sensörler", rx + 20, cy + 14, 200, 22, 15.0f, col::Text, true);
    const auto& c = d.core;
    double sy = cy + 48;
    // Battery: 13.2 V empty .. 16.7 V full.
    const double volts = c.batteryVoltage();
    const ARGB bc = volts >= 14.5 ? col::Green : (volts >= 13.5 ? col::Amber : col::Red);
    P.text(L"Batarya", rx + 20, sy, 90, 24, 12.5f, col::Muted, true);
    P.text(open ? fmt(L"%.1f V", volts) : L"—", rx + 110, sy, 70, 24, 15.0f, open ? bc : col::Faint, true);
    P.progress(rx + 184, sy + 8, rw - 204 - 110, 8, open ? (volts - 13.2) / 3.5 : 0.0, bc);
    P.text(!open ? L"" : (c.charger ? L"şarj kaynağı takılı" : (volts < 13.5 ? L"şarj edin" : L"")), rx + rw - 20 - 104, sy, 104, 24, 11.0f,
           c.charger ? col::Amber : bc, false, Align::Right);
    sy += 38;
    P.text(L"Tampon", rx + 20, sy, 200, 20, 12.5f, col::Muted, true);
    button(P, rx + rw - 20 - 84, sy - 4, 84, 26, L"Sıfırla", Btn::Secondary, Act::BumperReset, 0, open, 11.5f);
    sy += 26;
    const double iw = (rw - 40 - 16) / 3.0;
    indicator(P, rx + 20, sy, iw, L"Sol", open && c.bumperLeft(), d.seenLeft);
    indicator(P, rx + 28 + iw, sy, iw, L"Orta", open && c.bumperCenter(), d.seenCenter);
    indicator(P, rx + 36 + 2 * iw, sy, iw, L"Sağ", open && c.bumperRight(), d.seenRight);
    sy += 46;
    const int tested = d.seenLeft + d.seenCenter + d.seenRight;
    P.text(tested == 3 ? L"✓ Tampon testi tamam: üçü de algılandı." : L"Tampon testi: her birine elle bir kez basın (" + std::to_wstring(tested) + L" / 3).",
           rx + 20, sy, rw - 40, 18, 11.5f, tested == 3 ? col::Green : col::Muted);
    sy += 30;
    const auto flag = [&](double x, double w, const std::wstring& label, bool on, const wchar_t* onText) {
        P.round(x, sy, w, 34, 9, on ? col::AmberSoft : 0xFFF8FAFC, col::Border);
        P.text(label, x + 10, sy, w - 20, 34, 12.0f, col::Text, true);
        P.text(on ? onText : L"normal", x + 10, sy, w - 20, 34, 11.5f, on ? col::Amber : col::Faint, on, Align::Right);
    };
    const double fw = (rw - 40 - 8) / 2.0;
    flag(rx + 20, fw, L"Tekerlekler", open && c.wheel_drop != 0, L"havada");
    flag(rx + 28 + fw, fw, L"Zemin (cliff)", open && c.cliff != 0, L"boşluk");
    sy += 48;
    // Pose.
    P.text(L"Konum (odometri + jiroskop)", rx + 20, sy, 260, 20, 12.5f, col::Muted, true);
    button(P, rx + rw - 20 - 84, sy - 4, 84, 26, L"Sıfırla", Btn::Secondary, Act::DriveReset, 0, open, 11.5f);
    sy += 28;
    const double tw = (rw - 40 - 16) / 3.0;
    tile(P, rx + 20, sy, tw, 78, L"YÖN", open ? fmt(L"%.1f°", d.headingDeg) : L"—", L"sola +", col::Text);
    tile(P, rx + 28 + tw, sy, tw, 78, L"YOL", open ? fmt(L"%.0f cm", d.distance * 100.0) : L"—", L"ileri +", col::Text);
    tile(P, rx + 36 + 2 * tw, sy, tw, 78, L"X , Y", open ? fmt(L"%.0f , ", d.x * 100.0) + fmt(L"%.0f", d.y * 100.0) : L"—", L"cm", col::Text);
    sy += 90;
    P.text(L"\"İleri 40 cm\" sonrası YOL ≈ 40 cm, \"Sola 90°\" sonrası YÖN ≈ 90° olmalı; cetvelle de ölçün.", rx + 20, sy, rw - 40, 34, 11.5f, col::Faint,
           false, Align::Left, true, false);
    sy += 44;
    // Front sensor (Arduino), if its port is known.
    P.rect(rx + 20, sy, rw - 40, 1, col::Border);
    sy += 12;
    P.text(L"Ön sensör (Arduino)", rx + 20, sy, 200, 20, 12.5f, col::Muted, true);
    const sensor::Sample* last = lastSample();
    std::wstring st;
    ARGB sc = col::Faint;
    if (g.arduino.port.empty()) st = L"Arduino portu seçilmedi";
    else if (!streaming()) st = g.sens.error.empty() ? L"bağlanıyor..." : g.sens.error;
    else if (!last || !last->echo()) { st = L"yankı yok"; sc = col::Amber; }
    else {
        st = fmt(L"%.1f cm", g.cal.valid ? g.cal.cm(rawCm(*last)) : rawCm(*last)) + (g.cal.valid ? L" (kalibreli)" : L" (ham)");
        if (last->us >= 0) st += fmt(L" · %.0f µs", static_cast<double>(last->us));
        sc = col::Text;
    }
    P.text(st, rx + 20, sy + 22, rw - 40, 28, 16.0f, sc, true);
}

// ====================================================== pre-flight check ==
void drawPreflight(Painter& P, double x0, double y0) {
    const double W = BW - 20 - x0;
    const std::vector<PreItem> items = preflightItems();
    int fails = 0, warns = 0, busyN = 0;
    for (const auto& it : items) { fails += it.level == 2; warns += it.level == 1; busyN += it.level == 3; }
    const bool checking = g.preChecking || busyN > 0;

    P.card(x0, y0, W, 76);
    P.text(L"Hazırlık kontrolü", x0 + 20, y0 + 10, 400, 28, 17.0f, col::Text, true);
    const MapRef* map = g.maps.empty() ? nullptr : &g.maps[static_cast<std::size_t>(g.map)];
    P.text(Methods[static_cast<std::size_t>(g.method)].name + L"  ·  " + (map ? map->name : L"-") + L"  ·  gerçek robot  ·  portlar yeniden dinlenir, robota komut gitmez",
           x0 + 20, y0 + 42, 640, 20, 12.0f, col::Muted);
    std::wstring lt;
    ARGB fg, bg;
    if (checking) { lt = L"Kontrol ediliyor..."; fg = col::Accent; bg = col::AccentSoft; }
    else if (fails) { lt = L"✗ " + std::to_wstring(fails) + L" sorun"; fg = col::Red; bg = col::RedSoft; }
    else if (warns) { lt = L"! " + std::to_wstring(warns) + L" uyarı"; fg = col::Amber; bg = col::AmberSoft; }
    else { lt = L"✓ Hazır"; fg = col::Green; bg = col::GreenSoft; }
    const double pw = P.measure(lt, 12.5f, true) + 18;
    P.pill(lt, x0 + W - 20 - pw, y0 + 25, 26, fg, bg);

    const double cy = y0 + 88, ch = BH - 20 - cy;
    P.card(x0, cy, W, ch);
    double y = cy + 16;
    for (const auto& it : items) {
        const ARGB c = it.level == 0 ? col::Green : (it.level == 1 ? col::Amber : (it.level == 2 ? col::Red : col::Accent));
        const ARGB soft = it.level == 0 ? col::GreenSoft : (it.level == 1 ? col::AmberSoft : (it.level == 2 ? col::RedSoft : col::AccentSoft));
        P.circle(x0 + 38, y + 20, 13, soft);
        P.text(it.level == 0 ? L"✓" : (it.level == 1 ? L"!" : (it.level == 2 ? L"✗" : L"…")), x0 + 25, y + 7, 26, 26, 13.5f, c, true, Align::Center);
        P.text(it.title, x0 + 64, y, 220, 40, 13.5f, col::Text, true);
        P.text(it.detail, x0 + 290, y, W - 310, 40, 12.5f, it.level == 0 ? col::Muted : c);
        y += 42;
        P.rect(x0 + 20, y - 1, W - 40, 1, 0xFFF1F5F9);
    }

    // Placement + buttons.
    const double fy = cy + ch - 150;
    P.round(x0 + 20, fy, W - 40, 62, 10, col::AccentFaint, col::AccentSoft);
    P.text(L"Yerleştirme", x0 + 36, fy + 8, 200, 20, 12.5f, col::AccentDark, true);
    P.text(L"Robotun merkezi başlangıç hücresinde (1,1), +y yönüne (heading 90°) bakıyor. Çevresi boş; yanında durun. "
           L"Başlat'tan sonra robot bağlanır ve 3 s sonra hareket eder.",
           x0 + 36, fy + 28, W - 72, 30, 12.0f, col::Text, false, Align::Left, true, false);
    const double by = cy + ch - 68;
    button(P, x0 + 20, by, 130, 48, L"Vazgeç", Btn::Secondary, Act::PreCancel, 0, true, 13.5f);
    button(P, x0 + 160, by, 190, 48, L"Yeniden kontrol et", Btn::Secondary, Act::PreRetry, 0, !checking, 13.5f);
    const bool ports = !g.kobuki.port.empty() && !g.arduino.port.empty() && g.kobuki.port != g.arduino.port;
    const bool can = !checking && ports && exeExists(g.method);
    if (fails && !checking)
        P.text(L"Sorunlar giderilmeden başlatmak koşunun yarıda kalmasına yol açabilir.", x0 + 370, by, W - 390 - 270, 48, 12.0f, col::Red, false,
               Align::Left, true);
    button(P, x0 + W - 20 - 260, by, 260, 48, fails ? L"Yine de başlat" : L"▶   Robotta başlat", fails ? Btn::Danger : Btn::Primary, Act::PreGo, 0, can,
           15.0f);
}
