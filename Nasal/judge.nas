# AeroJudge - maneuver scoring system
# Written and designed by Weekendr58 aka. 7558
# One of my first "complex" nasal scripts - sorry for bad formatting... inspired by javascript but it's not the same...

var UI     = "/addons/aerojudge/ui/";
var ST     = "/addons/aerojudge/settings/";
var ROWS   = 10;
var DIALOG = "aerojudge";
var D2R    = math.pi / 180;
var R2D    = 180 / math.pi;

# ---------------------------------------------------------------- helpers
var fabs  = func(x) { return x < 0 ? -x : x; }
var fmax  = func(a, b) { return a > b ? a : b; }
var fmin  = func(a, b) { return a < b ? a : b; }
var sgn   = func(x) { return x < 0 ? -1 : 1; }
var clamp = func(x, lo, hi) { return x < lo ? lo : (x > hi ? hi : x); }
var acos  = func(x) { x = clamp(x, -1, 1); return math.atan2(math.sqrt(1 - x * x), x); }
var wrap180 = func(a) {
    while (a > 180)  { a -= 360; }
    while (a < -180) { a += 360; }
    return a;
}
var round1 = func(x) { return num(sprintf("%.1f", x)); }

# ---------------------------------------------------------------- config
var cfgRoot     = nil;
var profile     = nil;
var profileName = "defaults";
var PP          = {};          # property paths used for sampling

var cfgVal = func(path) {
    var v = nil;
    if (profile != nil) {
        var n = profile.getNode(path);
        if (n != nil) { v = n.getValue(); }
    }
    if (v == nil) {
        var n2 = cfgRoot.getNode("defaults/" ~ path);
        if (n2 != nil) { v = n2.getValue(); }
    }
    return v;
}
var cfgNum = func(path, dflt) {
    var v = cfgVal(path);
    if (v == nil) { return dflt; }
    v = num(v);
    return v == nil ? dflt : v;
}
var cfgStr = func(path, dflt) {
    var v = cfgVal(path);
    return v == nil ? dflt : (v ~ "");
}
var crit = func(path) {
    return {
        tol: cfgNum(path ~ "/tol", 0),
        per: fmax(cfgNum(path ~ "/per-point", 5), 0.000001),
        cap: cfgNum(path ~ "/cap", 20),
    };
}

var selectProfile = func {
    profile = nil;
    profileName = "defaults";
    var ac = string.lc((getprop("/sim/aircraft") or "") ~ "");
    foreach (var p; cfgRoot.getChildren("profile")) {
        foreach (var m; p.getChildren("match")) {
            var needle = m.getValue();
            if (needle == nil) { continue; }
            needle = string.lc(needle ~ "");
            if (size(needle) and find(needle, ac) >= 0) {
                profile = p;
                var nn = p.getNode("name");
                profileName = nn != nil ? (nn.getValue() ~ "") : needle;
                return;
            }
        }
    }
}

var loadPaths = func {
    PP.time  = "/sim/time/elapsed-sec";
    PP.pitch = cfgStr("properties/pitch",   "/orientation/pitch-deg");
    PP.roll  = cfgStr("properties/roll",    "/orientation/roll-deg");
    PP.hdg   = cfgStr("properties/heading", "/orientation/heading-deg");
    PP.alt   = cfgStr("properties/altitude","/position/altitude-ft");
    PP.agl   = cfgStr("properties/agl",     "/position/altitude-agl-ft");
    PP.ias   = cfgStr("properties/ias",     "/velocities/airspeed-kt");
    PP.g     = cfgStr("properties/g",       "/accelerations/pilot-g");
    # Optional: measured roll rate. Used for the aileron-roll rate criterion when present.
    PP.rr    = cfgStr("properties/roll-rate", "/orientation/roll-rate-degps");
}

var setNum = func(name, dflt) {
    var v = num(getprop(ST ~ name));
    return v == nil ? dflt : v;
}
var settingsProfile = "";
var loadSettings = func {
    settingsProfile = profileName;
    setprop(ST ~ "deck-mode", cfgStr("limits/hard-deck-mode", "Warn"));
    setprop(ST ~ "deck-ft",   cfgNum("limits/hard-deck-agl-ft", 1000));
    setprop(ST ~ "max-g",     cfgNum("limits/max-g", 6));
    setprop(ST ~ "min-g",     cfgNum("limits/min-g", -3));
    setprop(ST ~ "vne-kt",    cfgNum("limits/vne-kt", getprop("/limits/vne") or 200));
}
var reloadProfile = func {
    selectProfile();
    loadPaths();
    loadSettings();
    setStatus("Limits reset from profile: " ~ profileName);
}

var loadConfig = func(path) {
    cfgRoot = props.globals.getNode("/addons/aerojudge/config", 1);
    cfgRoot.removeAllChildren();
    var err = [];
    call(func { io.read_properties(path, cfgRoot); }, nil, nil, nil, err);
    if (size(err)) {
        logprint(LOG_ALERT, "AeroJudge: cannot read ", path, ": ", err[0]);
    }
    selectProfile();
    loadPaths();
}

# ---------------------------------------------------------------- state
var mode   = "idle";    # idle | armed | active | done | failed
var S      = {};        # per-run state
var M      = nil;       # current maneuver definition
var ticker = nil;

var setStatus = func(msg) { setprop(UI ~ "status", msg); }

var sample = func {
    var s = {
        t:     getprop(PP.time),
        pitch: getprop(PP.pitch),
        roll:  getprop(PP.roll),
        hdg:   getprop(PP.hdg),
        alt:   getprop(PP.alt),
        agl:   getprop(PP.agl),
        ias:   getprop(PP.ias),
        g:     getprop(PP.g),
    };
    foreach (var k; keys(s)) {
        if (s[k] == nil) { return nil; }
    }
    s.rr = num(getprop(PP.rr));    # may be nil: not required
    return s;
}

# Derived quantities relative to the entry plane.
# NED convention. Body x = nose, body y = right wing.
var update_kin = func(s) {
    var th = s.pitch * D2R;
    var ps = s.hdg   * D2R;
    var ph = s.roll  * D2R;
    var cth = math.cos(th);  var sth = math.sin(th);
    var cps = math.cos(ps);  var sps = math.sin(ps);
    var cph = math.cos(ph);  var sph = math.sin(ph);
    var nN = cth * cps;
    var nE = cth * sps;
    var nU = sth;
    var wN = sph * sth * cps - cph * sps;
    var wE = sph * sth * sps + cph * cps;
    # loop angle in the entry vertical plane; unambiguous at vertical
    s.alpha   = R2D * math.atan2(nU, nN * S.h0N + nE * S.h0E);
    # angle between right-wing vector and the entry-plane normal (0 = perfect planar loop)
    s.axisErr = R2D * acos(wN * S.r0N + wE * S.r0E);
}

var isLevel = func(s) {
    var l = cfgNum("entry-level-deg", 6);
    return fabs(s.roll) < l and fabs(s.pitch) < l;
}

# Re-baseline entry reference. Runs every tick while armed and still level, so the
# recorded entry heading/alt/speed are the last straight-and-level values before
# the maneuver actually began.
var rebase = func(s) {
    S.entry = { t: s.t, hdg: s.hdg, alt: s.alt, ias: s.ias, roll: s.roll, pitch: s.pitch };
    # Keep ~1 s of level history. The last "level" sample sits at the edge of the
    # level band (e.g. already 5 deg nose-up), so begin() snapshots the entry from
    # before that instead.
    append(S.hist, S.entry);
    var lag = cfgNum("entry-lag-sec", 1.0);
    while (size(S.hist) > 1 and S.hist[1].t <= s.t - lag) { S.hist = subvec(S.hist, 1); }
    var ps = s.hdg * D2R;
    S.h0N = math.cos(ps);  S.h0E = math.sin(ps);
    S.r0N = -S.h0E;        S.r0E = S.h0N;
    update_kin(s);
    S.alphaPrev = s.alpha;  S.alphaU = 0;
    S.rollPrev  = s.roll;   S.rollU  = 0;
    S.hdgPrev   = s.hdg;    S.hdgU   = 0;
}

var accumulate = func(s) {
    S.alphaU += wrap180(s.alpha - S.alphaPrev);  S.alphaPrev = s.alpha;
    S.rollU  += wrap180(s.roll  - S.rollPrev);   S.rollPrev  = s.roll;
    S.hdgU   += wrap180(s.hdg   - S.hdgPrev);    S.hdgPrev   = s.hdg;
}

var newState = func(s) {
    S = { started: 0, t_prev: s.t, t_start: 0, m: {}, hist: [],
          maxg: -99, ming: 99, maxias: 0, minagl: 1e9, timeout: 120,
          levelSince: -1 };
    rebase(s);
}

# Called by a maneuver when its start trigger fires.
var begin = func(s) {
    S.entry = S.hist[0];
    S.started = 1;
    S.levelSince = -1;
    S.t_start = s.t;
    S.maxg = s.g;  S.ming = s.g;  S.maxias = s.ias;  S.minagl = s.agl;
    S.timeout = cfgNum(M.path ~ "/timeout-sec", 120);
    mode = "active";
    setStatus(M.label ~ ": ACTIVE");
}

# ---------------------------------------------------------------- scoring
var ptsFor = func(over, per, cap) { return over <= 0 ? 0 : fmin(cap, over / per); }
var mk     = func(name, text, pts) { return { name: name, value: text, pts: pts }; }
var fmtv   = func(v, unit) {
    return sprintf(fabs(v) >= 100 ? "%.0f %s" : "%.1f %s", v, unit);
}

# one-sided deviation with free tolerance
var addCrit = func(items, name, err, unit, path) {
    var c = crit(path);
    append(items, mk(name, fmtv(err, unit), ptsFor(fabs(err) - c.tol, c.per, c.cap)));
}
# window: <lo> <hi> bounds; points for distance outside
var addWindow = func(items, name, val, unit, path) {
    var lo = cfgNum(path ~ "/lo", -1e9);
    var hi = cfgNum(path ~ "/hi",  1e9);
    var c  = crit(path);
    var over = val < lo ? lo - val : (val > hi ? val - hi : 0);
    append(items, mk(name, fmtv(val, unit), ptsFor(over - c.tol, c.per, c.cap)));
}

var envelopeRows = func(items) {
    var maxG = setNum("max-g", 6);
    var minG = setNum("min-g", -3);
    var vne  = setNum("vne-kt", 200);
    var gc = crit("limits/over-g");
    var vc = crit("limits/over-vne");
    append(items, mk("Peak g", sprintf("%.1f g (limit %.1f)", S.maxg, maxG),
                     ptsFor(S.maxg - maxG, gc.per, gc.cap)));
    if (S.ming < 0) {
        append(items, mk("Min g", sprintf("%.1f g (limit %.1f)", S.ming, minG),
                         ptsFor(minG - S.ming, gc.per, gc.cap)));
    }
    append(items, mk("Max airspeed", sprintf("%.0f kt (Vne %.0f)", S.maxias, vne),
                     ptsFor(S.maxias - vne, vc.per, vc.cap)));
    if (getprop(ST ~ "deck-mode") == "Warn") {
        var hd = setNum("deck-ft", 1000);
        var dc = crit("limits/deck-warn");
        append(items, mk("Lowest height AGL", sprintf("%.0f ft (deck %.0f)", S.minagl, hd),
                         ptsFor(hd - S.minagl, dc.per, dc.cap)));
    }
}

var clearRows = func {
    for (var i = 0; i < ROWS; i += 1) {
        var b = UI ~ "row[" ~ i ~ "]/";
        setprop(b ~ "name", "");  setprop(b ~ "value", "");  setprop(b ~ "pts", "");
    }
    setprop(UI ~ "total", "");
}

var showCard = func(title, items, total) {
    clearRows();
    for (var i = 0; i < ROWS and i < size(items); i += 1) {
        var b = UI ~ "row[" ~ i ~ "]/";
        var it = items[i];
        setprop(b ~ "name",  it.name);
        setprop(b ~ "value", it.value);
        setprop(b ~ "pts",   it.pts > 0.04 ? sprintf("-%.1f", it.pts) : "0");
    }
    setprop(UI ~ "total", sprintf("%s: %.1f / 100", title, total));
}

# ---- session history and CSV log
var history = [];     # last few results, newest last
var HIST_MAX = 4;

var csvQuote = func(t) {
    var out = "";
    for (var i = 0; i < size(t); i += 1) {
        var c = substr(t, i, 1);
        out ~= (c == "\"" ? "\"\"" : c);
    }
    return "\"" ~ out ~ "\"";
}

# Appends one line per scored run to $FG_HOME/Export/AeroJudge.csv (the only place
# FlightGear lets Nasal write). Never lets a file problem disturb scoring.
var logCsv = func(label, items, total) {
    var home = getprop("/sim/fg-home");
    if (home == nil) { return; }
    var path = home ~ "/Export/AeroJudge.csv";
    var err = [];
    call(func {
        var isNew = (io.stat(path) == nil);
        var f = io.open(path, "a");
        if (f == nil) { die("cannot open " ~ path); }
        if (isNew) { io.write(f, "utc,aircraft,maneuver,score,details\n"); }
        var det = "";
        foreach (var it; items) {
            det ~= (size(det) ? "; " : "") ~ sprintf("%s: %s (%s)", it.name, it.value,
                                                     it.pts > 0.04 ? sprintf("-%.1f", it.pts) : "0");
        }
        var ts = sprintf("%04d-%02d-%02dT%02d:%02d:%02dZ",
                         getprop("/sim/time/utc/year") or 0, getprop("/sim/time/utc/month") or 0,
                         getprop("/sim/time/utc/day") or 0,  getprop("/sim/time/utc/hour") or 0,
                         getprop("/sim/time/utc/minute") or 0, getprop("/sim/time/utc/second") or 0);
        io.write(f, sprintf("%s,%s,%s,%.1f,%s\n", ts, csvQuote((getprop("/sim/aircraft") or "") ~ ""),
                            csvQuote(label), total, csvQuote(det)));
        io.close(f);
    }, nil, nil, nil, err);
    if (size(err)) { logprint(LOG_ALERT, "AeroJudge: could not write ", path, ": ", err[0]); }
}

var recordResult = func(label, total) {
    append(history, sprintf("%s %.0f", label, total));
    while (size(history) > HIST_MAX) { history = subvec(history, 1); }
    var txt = "";
    foreach (var h; history) { txt ~= (size(txt) ? "  |  " : "") ~ h; }
    setprop(UI ~ "history", "Recent: " ~ txt);
}

var finish = func(s) {
    ticker.stop();
    mode = "done";
    var items = [];
    M.score(s, items);
    envelopeRows(items);
    var total = 100;
    foreach (var it; items) { total -= it.pts; }
    total = round1(fmax(0, total));
    showCard(M.label, items, total);
    recordResult(M.label, total);
    logCsv(M.label, items, total);
    setprop(UI ~ "live", "");
    setStatus(M.label ~ ": finished");
    gui.popupTip(sprintf("AeroJudge - %s: %.0f / 100", M.label, total), 6);
    logprint(LOG_INFO, "AeroJudge ", M.label, " (", profileName, "): ", total, " / 100");
    foreach (var it; items) {
        logprint(LOG_INFO, sprintf("   %-24s %-22s -%.1f", it.name, it.value, it.pts));
    }
}

var fail = func(msg, zero = 0) {
    ticker.stop();
    mode = "failed";
    clearRows();
    setprop(UI ~ "row[0]/name", "Maneuver not scored");
    setprop(UI ~ "row[0]/value", msg);
    setprop(UI ~ "total", zero ? "FAIL: 0 / 100" : "No score");
    setprop(UI ~ "live", "");
    setStatus(M.label ~ ": " ~ msg);
    gui.popupTip("AeroJudge: " ~ msg, 6);
}

# ---------------------------------------------------------------- tick
var liveLine = func(s) {
    var txt = "";
    if (mode == "armed") {
        txt = sprintf("Armed. Entry hdg %03.0f  alt %.0f ft  IAS %.0f kt",
                      S.entry.hdg, S.entry.alt, S.entry.ias);
    } else {
        txt = sprintf("[%s] t %.0fs g %.1f IAS %.0f dAlt %+.0f dHdg %+.0f loop %+.0f roll %+.0f",
                      S.m.phase, s.t - S.t_start, s.g, s.ias, s.alt - S.entry.alt,
                      wrap180(s.hdg - S.entry.hdg), S.alphaU, S.rollU);
    }
    setprop(UI ~ "live", txt);
}

var tick = func {
    if (mode != "armed" and mode != "active") { return; }
    var s = sample();
    if (s == nil) { return; }
    if (s.t <= S.t_prev) { return; }
    var dt = s.t - S.t_prev;
    S.t_prev = s.t;
    if (S.started and dt > 1.0) { return fail("sim time jumped"); }

    if (!S.started and isLevel(s)) {
        rebase(s);
    } else {
        update_kin(s);
        accumulate(s);
    }

    # Hard deck: only aborts in "Fail" mode. "Warn" just records lowest AGL and deducts.
    if (getprop(ST ~ "deck-mode") == "Fail") {
        var hd = setNum("deck-ft", 1000);
        if (s.agl < hd) {
            if (S.started) { return fail(sprintf("HARD DECK: %.0f ft AGL (min %.0f)", s.agl, hd), 1); }
            disarm("Disarmed: below hard deck");
            return;
        }
    }

    if (S.started) {
        S.maxg   = fmax(S.maxg, s.g);
        S.ming   = fmin(S.ming, s.g);
        S.maxias = fmax(S.maxias, s.ias);
        S.minagl = fmin(S.minagl, s.agl);
    }

    # Abandoned run: wings and nose level for a few seconds while the maneuver is
    # still incomplete. Ends the run now instead of waiting for the timeout.
    # (Settle phases are exempt: finishing a maneuver ends in level flight.)
    if (S.started and S.m.phase != "settle") {
        if (!isLevel(s)) {
            S.levelSince = -1;
        } elsif (S.levelSince < 0) {
            S.levelSince = s.t;
        } elsif (s.t - S.levelSince >= cfgNum("abandon-level-sec", 3)) {
            var why = M.why != nil ? M.why(S.m) : "maneuver incomplete";
            return fail("abandoned: " ~ why);
        }
    }

    var done = M.step(s);
    liveLine(s);
    if (done) { finish(s); return; }
    if (S.started and (s.t - S.t_start) > S.timeout) { fail("timed out"); }
}

# ---------------------------------------------------------------- maneuvers
var MANEUVERS = {};
var ORDER = ["steep", "roll", "fourpoint", "loop", "immelmann", "splits"];

# --- 1. Steep turn --------------------------------------------------------
MANEUVERS["steep"] = {
    label: "Steep turn",
    path:  "steep-turn",
    init: func(s) {
        S.m = { phase: "wait", dir: 1, est: 0, scoring: 1,
                bankEx: 0, altDev: 0, iasDev: 0, tSettle: 0 };
    },
    step: func(s) {
        var m = S.m;
        var P = "steep-turn/";
        if (m.phase == "wait") {
            if (fabs(s.roll - S.entry.roll) > cfgNum(P ~ "start-roll-deg", 20)) {
                begin(s);
                m.dir = sgn(s.roll - S.entry.roll);
                m.phase = "turn";
            }
            return 0;
        }
        var turned  = m.dir * S.hdgU;
        var bmin    = cfgNum(P ~ "bank-min-deg", 45);
        var bmax    = cfgNum(P ~ "bank-max-deg", 60);
        var turnDeg = cfgNum(P ~ "turn-deg", 360);
        var lead    = cfgNum(P ~ "rollout-lead-deg", 25);
        var b = fabs(s.roll);
        if (m.phase == "turn") {
            if (!m.est and b >= bmin) { m.est = 1; }
            if (m.est) {
                if (m.scoring) {
                    var ex = b < bmin ? bmin - b : (b > bmax ? b - bmax : 0);
                    m.bankEx = fmax(m.bankEx, ex);
                    if (turned >= turnDeg - lead or b < bmin - 15) { m.scoring = 0; }
                }
                m.altDev = fmax(m.altDev, fabs(s.alt - S.entry.alt));
                m.iasDev = fmax(m.iasDev, fabs(s.ias - S.entry.ias));
                if (turned >= 270 and b < 12) { m.phase = "settle"; m.tSettle = s.t; }
            }
        } else if (m.phase == "settle") {
            m.altDev = fmax(m.altDev, fabs(s.alt - S.entry.alt));
            m.iasDev = fmax(m.iasDev, fabs(s.ias - S.entry.ias));
            if (s.t - m.tSettle >= cfgNum(P ~ "settle-sec", 1.0)) { return 1; }
        }
        return 0;
    },
    why: func(m) {
        return m.est ? "turn stopped before 270 deg"
                     : sprintf("bank never reached %.0f deg", cfgNum("steep-turn/bank-min-deg", 45));
    },
    score: func(s, items) {
        var m = S.m;
        var P = "steep-turn/";
        addCrit(items, "Bank excursion",       m.bankEx, "deg", P ~ "bank");
        addCrit(items, "Altitude deviation",   m.altDev, "ft",  P ~ "alt-ft");
        addCrit(items, "Airspeed deviation",   m.iasDev, "kt",  P ~ "ias-kt");
        addCrit(items, "Rollout heading error", wrap180(s.hdg - S.entry.hdg), "deg", P ~ "rollout-deg");
    },
};

# --- 2. Aileron roll ------------------------------------------------------
MANEUVERS["roll"] = {
    label: "Aileron roll",
    path:  "aileron-roll",
    init: func(s) {
        S.m = { phase: "wait", dir: 1, ts: [], us: [], rrs: [], maxHdg: 0, tSettle: 0 };
    },
    step: func(s) {
        var m = S.m;
        var P = "aileron-roll/";
        if (m.phase == "wait") {
            if (fabs(S.rollU) > cfgNum(P ~ "start-roll-deg", 15)) {
                begin(s);
                m.dir = sgn(S.rollU);
                m.phase = "roll";
            } else { return 0; }
        }
        var rolled = m.dir * S.rollU;
        append(m.ts, s.t);
        append(m.us, rolled);
        m.maxHdg = fmax(m.maxHdg, fabs(wrap180(s.hdg - S.entry.hdg)));
        var n = size(m.us);
        var rate = 0;
        if (s.rr != nil) {
            rate = m.dir * s.rr;      # measured: signed along the roll direction
        } elsif (n > 4) {
            rate = (m.us[n-1] - m.us[n-5]) / (m.ts[n-1] - m.ts[n-5]);
        }
        # measured rate for the constancy score, excluding start/stop (30..330 deg)
        if (s.rr != nil and rolled >= 30 and rolled <= 330) { append(m.rrs, rate); }
        if (m.phase == "roll") {
            if (rolled >= cfgNum(P ~ "settle-start-deg", 340)) { m.phase = "settle"; m.tSettle = s.t; }
        } else if (m.phase == "settle") {
            if (rolled >= 360 + cfgNum(P ~ "overroll-deg", 60)
                or fabs(rate) < cfgNum(P ~ "end-rate-dps", 10)
                or s.t - m.tSettle > cfgNum(P ~ "settle-timeout-sec", 2.5)) { return 1; }
        }
        return 0;
    },
    score: func(s, items) {
        var m = S.m;
        var P = "aileron-roll/";
        # roll-rate constancy between 30 and 330 deg of rotation (excludes start/stop)
        var rates = [];
        if (size(m.rrs) >= 5) {
            rates = m.rrs;            # measured roll rate
        } else {                      # property missing: differentiate the roll angle
            var n = size(m.us);
            for (var i = 4; i < n; i += 1) {
                var mid = m.us[i-2];
                if (mid >= 30 and mid <= 330) {
                    append(rates, (m.us[i] - m.us[i-4]) / (m.ts[i] - m.ts[i-4]));
                }
            }
        }
        var mean = 0;  var cv = 100;
        if (size(rates) >= 5) {
            foreach (var r; rates) { mean += r; }
            mean /= size(rates);
            var ss = 0;
            foreach (var r2; rates) { ss += (r2 - mean) * (r2 - mean); }
            var sd = math.sqrt(ss / size(rates));
            cv = fabs(mean) > 1 ? 100 * sd / fabs(mean) : 100;
        }
        addCrit(items, "Heading drift (max)", m.maxHdg, "deg", P ~ "heading-deg");
        addCrit(items, "Altitude change", s.alt - S.entry.alt, "ft", P ~ "alt-ft");
        var c = crit(P ~ "rate-cv-pct");
        append(items, mk("Roll-rate variation",
                         sprintf("%.1f %% (avg %.0f dps)", cv, fabs(mean)),
                         ptsFor(cv - c.tol, c.per, c.cap)));
        addCrit(items, "Roll angle at exit", wrap180(s.roll - S.entry.roll), "deg", P ~ "exit-roll-deg");
    },
};

# --- 2b. Four-point roll --------------------------------------------------
# 360 deg of roll in four 90 deg steps. Pauses at 90/180/270 are found from the
# roll rate, the fourth point is the exit at 360.
MANEUVERS["fourpoint"] = {
    label: "Four-point roll",
    path:  "four-point-roll",
    init: func(s) {
        S.m = { phase: "wait", dir: 1, ts: [], us: [], maxHdg: 0, tSettle: 0,
                inStop: 0, sT: 0, sSum: 0, sN: 0, lastAt: 0, stops: [] };
    },
    step: func(s) {
        var m = S.m;
        var P = "four-point-roll/";
        if (m.phase == "wait") {
            if (fabs(S.rollU) > cfgNum(P ~ "start-roll-deg", 15)) {
                begin(s);
                m.dir = sgn(S.rollU);
                m.phase = "roll";
            } else { return 0; }
        }
        var rolled = m.dir * S.rollU;
        append(m.ts, s.t);
        append(m.us, rolled);
        m.maxHdg = fmax(m.maxHdg, fabs(wrap180(s.hdg - S.entry.hdg)));
        var n = size(m.us);
        var rate = 0;
        if (s.rr != nil) {
            rate = m.dir * s.rr;      # measured: signed along the roll direction
        } elsif (n > 4) {
            rate = (m.us[n-1] - m.us[n-5]) / (m.ts[n-1] - m.ts[n-5]);
        }
        var stopRate = cfgNum(P ~ "stop-rate-dps", 25);
        if (m.phase == "roll") {
            if (!m.inStop) {
                # pause starts when the rate drops after moving on from the last point
                if (rolled - m.lastAt >= cfgNum(P ~ "min-travel-deg", 30) and fabs(rate) < stopRate) {
                    m.inStop = 1;  m.sT = s.t;  m.sSum = rolled;  m.sN = 1;
                }
            } else if (fabs(rate) < 2 * stopRate) {
                m.sSum += rolled;  m.sN += 1;
            } else {
                # rolling again: pause only counts if it was held long enough
                m.inStop = 0;
                var hold = s.t - m.sT;
                if (hold >= cfgNum(P ~ "min-hold-sec", 0.25)) {
                    var at = m.sSum / m.sN;
                    var k = int(at / 90 + 0.5);     # nearest point, at is positive
                    k = k < 1 ? 1 : (k > 3 ? 3 : k);
                    append(m.stops, { hold: hold, err: at - 90 * k });
                    m.lastAt = at;
                }
            }
            if (rolled >= cfgNum(P ~ "settle-start-deg", 340)) { m.phase = "settle"; m.tSettle = s.t; }
        } else if (m.phase == "settle") {
            if (rolled >= 360 + cfgNum(P ~ "overroll-deg", 60)
                or fabs(rate) < cfgNum(P ~ "end-rate-dps", 10)
                or s.t - m.tSettle > cfgNum(P ~ "settle-timeout-sec", 2.5)) { return 1; }
        }
        return 0;
    },
    why: func(m) {
        return sprintf("%d of 3 pauses found before the roll stopped", size(m.stops));
    },
    score: func(s, items) {
        var m = S.m;
        var P = "four-point-roll/";
        var n = size(m.stops);
        var missing = fmax(0, 3 - n);
        append(items, mk("Pauses found", sprintf("%d of 3", n),
                         fmin(cfgNum(P ~ "missing-stop-cap", 30), missing * cfgNum(P ~ "missing-stop-pts", 10))));
        var maxErr = 0;  var hmin = 1e9;  var hmax = 0;
        foreach (var st; m.stops) {
            maxErr = fmax(maxErr, fabs(st.err));
            hmin = fmin(hmin, st.hold);  hmax = fmax(hmax, st.hold);
        }
        addCrit(items, "Pause angle error (max)", maxErr, "deg", P ~ "stop-angle-deg");
        if (n >= 2) {
            var c = crit(P ~ "hold-spread-sec");
            append(items, mk("Pause length spread", sprintf("%.2f s (%.2f-%.2f)", hmax - hmin, hmin, hmax),
                             ptsFor((hmax - hmin) - c.tol, c.per, c.cap)));
        } else {
            append(items, mk("Pause length spread", "n/a", 0));
        }
        addCrit(items, "Heading drift (max)", m.maxHdg, "deg", P ~ "heading-deg");
        addCrit(items, "Altitude change", s.alt - S.entry.alt, "ft", P ~ "alt-ft");
        addCrit(items, "Roll angle at exit", wrap180(s.roll - S.entry.roll), "deg", P ~ "exit-roll-deg");
    },
};

# --- 3. Loop --------------------------------------------------------------
MANEUVERS["loop"] = {
    label: "Loop",
    path:  "loop",
    init: func(s) { S.m = { phase: "wait", maxAxis: 0, tExit: 0 }; },
    step: func(s) {
        var m = S.m;
        var P = "loop/";
        if (m.phase == "wait") {
            if (S.alphaU > cfgNum(P ~ "start-deg", 20)) {
                begin(s);
                m.phase = "loop";
            } else { return 0; }
        }
        if (m.phase == "loop") {
            m.maxAxis = fmax(m.maxAxis, s.axisErr);
            if (S.alphaU >= cfgNum(P ~ "exit-deg", 350)) { m.phase = "settle"; m.tExit = s.t; }
        } else if (m.phase == "settle") {
            if (S.alphaU >= cfgNum(P ~ "overrun-deg", 375)
                or s.t - m.tExit >= cfgNum(P ~ "settle-sec", 1.0)) { return 1; }
        }
        return 0;
    },
    score: func(s, items) {
        var m = S.m;
        var P = "loop/";
        addCrit(items, "Exit heading error",  wrap180(s.hdg - S.entry.hdg), "deg", P ~ "heading-deg");
        addCrit(items, "Exit altitude change", s.alt - S.entry.alt, "ft", P ~ "alt-ft");
        addCrit(items, "Exit speed change",    s.ias - S.entry.ias, "kt", P ~ "ias-kt");
        addCrit(items, "Exit pitch",           s.pitch - S.entry.pitch, "deg", P ~ "exit-pitch-deg");
        addCrit(items, "Wing-axis error (max)", m.maxAxis, "deg", P ~ "axis-deg");
    },
};

# --- 4a. Immelmann: half loop up + half roll -------------------------------
MANEUVERS["immelmann"] = {
    label: "Immelmann",
    path:  "immelmann",
    init: func(s) { S.m = { phase: "wait", tEnd: 0 }; },
    step: func(s) {
        var m = S.m;
        var P = "immelmann/";
        if (m.phase == "wait") {
            if (S.alphaU > cfgNum(P ~ "start-deg", 20)) {
                begin(s);
                m.phase = "pull";
            }
            return 0;
        }
        if (m.phase == "pull") {
            if (S.alphaU >= cfgNum(P ~ "pull-done-deg", 150)) { m.phase = "roll"; }
        } else if (m.phase == "roll") {
            if (fabs(s.roll) < cfgNum(P ~ "end-roll-deg", 15)) { m.phase = "settle"; m.tEnd = s.t; }
        } else if (m.phase == "settle") {
            if (s.t - m.tEnd >= cfgNum(P ~ "settle-sec", 1.0)) { return 1; }
        }
        return 0;
    },
    score: func(s, items) {
        var P = "immelmann/";
        addCrit(items, "Heading reversal error", wrap180(s.hdg - S.entry.hdg - 180), "deg", P ~ "heading-deg");
        addWindow(items, "Altitude gained", s.alt - S.entry.alt, "ft", P ~ "alt-gain-ft");
        addCrit(items, "Exit roll (upright)", s.roll, "deg", P ~ "exit-roll-deg");
        addCrit(items, "Exit pitch (level)",  s.pitch - S.entry.pitch, "deg", P ~ "exit-pitch-deg");
        addWindow(items, "Exit speed", s.ias, "kt", P ~ "exit-ias-kt");
    },
};

# --- 4b. Split-S: half roll to inverted + half loop down -------------------
MANEUVERS["splits"] = {
    label: "Split-S",
    path:  "split-s",
    init: func(s) { S.m = { phase: "wait", rollAtPull: -1, tExit: 0 }; },
    step: func(s) {
        var m = S.m;
        var P = "split-s/";
        if (m.phase == "wait") {
            if (fabs(S.rollU) > cfgNum(P ~ "roll-start-deg", 30)) {
                begin(s);
                m.phase = "pull";
            }
            return 0;
        }
        if (m.phase == "pull") {
            if (m.rollAtPull < 0 and S.alphaU < -cfgNum(P ~ "pull-start-deg", 15)) {
                m.rollAtPull = fabs(s.roll);
            }
            if (S.alphaU <= -cfgNum(P ~ "exit-deg", 165)) { m.phase = "settle"; m.tExit = s.t; }
        } else if (m.phase == "settle") {
            if (S.alphaU <= -cfgNum(P ~ "overrun-deg", 200)
                or s.t - m.tExit >= cfgNum(P ~ "settle-sec", 1.0)) { return 1; }
        }
        return 0;
    },
    score: func(s, items) {
        var m = S.m;
        var P = "split-s/";
        var inv = cfgNum(P ~ "inverted-roll-deg", 150);
        var deficit = fmax(0, inv - (m.rollAtPull < 0 ? 0 : m.rollAtPull));
        addCrit(items, "Inverted before pull", deficit, "deg short", P ~ "inverted-deg");
        addCrit(items, "Heading reversal error", wrap180(s.hdg - S.entry.hdg - 180), "deg", P ~ "heading-deg");
        addWindow(items, "Altitude lost", S.entry.alt - s.alt, "ft", P ~ "alt-loss-ft");
        addCrit(items, "Exit roll (upright)", s.roll, "deg", P ~ "exit-roll-deg");
        addCrit(items, "Exit pitch (level)",  s.pitch - S.entry.pitch, "deg", P ~ "exit-pitch-deg");
    },
};

# ---------------------------------------------------------------- control API
var labelToKey = func(label) {
    foreach (var k; ORDER) {
        if (MANEUVERS[k].label == label) { return k; }
    }
    return nil;
}

var arm = func {
    selectProfile();
    loadPaths();
    # A different aircraft means different limits: reload the panel values then,
    # but never overwrite what the user typed for the same aircraft.
    if (profileName != settingsProfile) { loadSettings(); }
    var key = labelToKey(getprop(UI ~ "maneuver"));
    if (key == nil) { setStatus("Pick a maneuver first"); return; }
    var s = sample();
    if (s == nil) { setStatus("Arm refused: flight data not available"); return; }
    var lvl = cfgNum("arm-level-deg", 15);
    if (fabs(s.roll) > lvl or fabs(s.pitch) > lvl) {
        setStatus(sprintf("Arm refused: fly straight and level first (pitch %.0f, roll %.0f)", s.pitch, s.roll));
        return;
    }
    var hd = setNum("deck-ft", 1000);
    if (getprop(ST ~ "deck-mode") == "Fail" and s.agl < hd) {
        setStatus(sprintf("Arm refused: %.0f ft AGL is below the %.0f ft hard deck", s.agl, hd));
        return;
    }
    M = MANEUVERS[key];
    newState(s);
    M.init(s);
    mode = "armed";
    clearRows();
    setStatus(sprintf("%s ARMED (profile: %s). Start when ready.", M.label, profileName));
    ticker.stop();
    ticker.start();
}

# Select a maneuver by key ("steep", "roll", "fourpoint", "loop", "immelmann", "splits") and arm it.
# Used by the menu so no panel is needed.
var armKey = func(key) {
    if (!contains(MANEUVERS, key)) { return; }
    setprop(UI ~ "maneuver", MANEUVERS[key].label);
    arm();
    var st = getprop(UI ~ "status");
    if (st != nil) { gui.popupTip("AeroJudge: " ~ st, 4); }
}

var disarm = func(msg = "Idle") {
    if (ticker != nil) { ticker.stop(); }
    mode = "idle";
    setStatus(msg);
    setprop(UI ~ "live", "");
}

# ---------------------------------------------------------------- dialog
var addw = func(parent, tag, vals = nil) {
    var n = parent.addChild(tag);
    if (vals != nil) { n.setValues(vals); }
    return n;
}

var buildDialog = func {
    var W = "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX";
    var d = props.Node.new({ name: DIALOG, layout: "vbox", "default-padding": 4 });

    var g = addw(d, "group", { layout: "hbox" });
    addw(g, "empty", { stretch: 1 });
    addw(g, "text",  { label: "Aerobatic Judge" });
    addw(g, "empty", { stretch: 1 });
    var x = addw(g, "button", { "pref-width": 20, "pref-height": 20, legend: "", keynum: 27, border: 2 });
    addw(x, "binding", { command: "dialog-close" });

    addw(d, "hr");

    var r = addw(d, "group", { layout: "hbox" });
    addw(r, "text", { label: "Maneuver", halign: "left" });
    var cb = addw(r, "combo", { name: "maneuver", "pref-width": 190,
                                property: UI ~ "maneuver", live: 1 });
    foreach (var k; ORDER) { addw(cb, "value").setValue(MANEUVERS[k].label); }
    addw(cb, "binding", { command: "dialog-apply", "object-name": "maneuver" });

    var br = addw(d, "group", { layout: "hbox" });
    var b1 = addw(br, "button", { legend: "Arm", "pref-width": 100 });
    addw(b1, "binding", { command: "dialog-apply" });
    addw(b1, "binding", { command: "nasal", script: "aerojudge.arm();" });
    var b2 = addw(br, "button", { legend: "Disarm / reset", "pref-width": 120 });
    addw(b2, "binding", { command: "nasal", script: "aerojudge.disarm();" });

    addw(d, "text", { label: W, halign: "left", property: UI ~ "status", live: 1 });
    addw(d, "text", { label: W, halign: "left", property: UI ~ "live",   live: 1 });
    addw(d, "hr");

    # --- limits (what the judge actually uses; initialised from the aircraft profile)
    var lr = addw(d, "group", { layout: "hbox" });
    addw(lr, "text", { label: "Hard deck", halign: "left" });
    var dm = addw(lr, "combo", { name: "deckmode", "pref-width": 70, property: ST ~ "deck-mode", live: 1 });
    foreach (var v; ["Off", "Warn", "Fail"]) { addw(dm, "value").setValue(v); }
    addw(dm, "binding", { command: "dialog-apply", "object-name": "deckmode" });
    var di = addw(lr, "input", { name: "deckft", "pref-width": 60, property: ST ~ "deck-ft", live: 1 });
    addw(di, "binding", { command: "dialog-apply", "object-name": "deckft" });
    addw(lr, "text", { label: "ft AGL", halign: "left" });

    var gr = addw(d, "group", { layout: "hbox" });
    var fields = [["Max g", "max-g"], ["Min g", "min-g"], ["Vne kt", "vne-kt"]];
    foreach (var f; fields) {
        addw(gr, "text", { label: f[0], halign: "left" });
        var inp = addw(gr, "input", { name: f[1], "pref-width": 55, property: ST ~ f[1], live: 1 });
        addw(inp, "binding", { command: "dialog-apply", "object-name": f[1] });
    }
    var rb = addw(gr, "button", { legend: "Reset to profile", "pref-width": 110 });
    addw(rb, "binding", { command: "nasal", script: "aerojudge.reloadProfile();" });
    addw(d, "hr");

    for (var i = 0; i < ROWS; i += 1) {
        var row = addw(d, "group", { layout: "hbox" });
        var b = UI ~ "row[" ~ i ~ "]/";
        addw(row, "text", { label: "XXXXXXXXXXXXXXXXXXXXXXXX", halign: "left",  property: b ~ "name",  live: 1 });
        addw(row, "text", { label: "XXXXXXXXXXXXXXXXXXXXXXXXXXXX", halign: "left",  property: b ~ "value", live: 1 });
        addw(row, "text", { label: "XXXXXX", halign: "right", property: b ~ "pts",   live: 1 });
    }
    addw(d, "hr");
    addw(d, "text", { label: W, halign: "left", property: UI ~ "total", live: 1 });
    addw(d, "text", { label: W, halign: "left", property: UI ~ "history", live: 1 });

    fgcommand("dialog-new", d);
}

var showDialog = func {
    fgcommand("dialog-show", props.Node.new({ "dialog-name": DIALOG }));
}

var init = func(addon) {
    loadConfig(addon.basePath ~ "/addon-config.xml");
    loadSettings();
    setprop(UI ~ "maneuver", MANEUVERS[ORDER[0]].label);
    setStatus("Idle. Fly level, choose a maneuver, press Arm.");
    ticker = maketimer(0.05, tick);
    ticker.simulatedTime = 1;
    buildDialog();
    if (cfgNum("show-on-start", 0)) { settimer(showDialog, 2); }
    logprint(LOG_INFO, "AeroJudge ready (profile: ", profileName, ")");
}

var shutdown = func {
    if (ticker != nil) { ticker.stop(); }
    fgcommand("dialog-close", props.Node.new({ "dialog-name": DIALOG }));
}
