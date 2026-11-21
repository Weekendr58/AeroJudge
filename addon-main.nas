# AeroJudge entry point

var main = func(addon) {
    logprint(LOG_INFO, "AeroJudge: loading from ", addon.basePath);
    # The scoring code lives in the global namespace "aerojudge" so that the
    # dialog buttons and menu can call aerojudge.arm(), aerojudge.showDialog(), ...
    io.load_nasal(addon.basePath ~ "/Nasal/judge.nas", "aerojudge");
    globals["aerojudge"].init(addon);
}

var unload = func(addon) {
    if (contains(globals, "aerojudge")) {
        globals["aerojudge"].shutdown();
    }
}
