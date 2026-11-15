# Recording and scoring provenance

The supplied videos record the same physical scene and demonstration controller as the submitted code. They precede the scoring corrections for commanded initial closure, complete brick clearance, and simultaneous opening and hand separation.

The retained traces contain4 Hz samples and no body orientation or per-run source snapshot. Rechecking these samples with a conservative40 mm bounding sphere gives demo1.0 and Dream-1 0.0. The demo's initial close command is established from its retained controller. The historical model reset was not logged in enough detail to certify its initial-close interval. This is a saved-sample check, not a new20 Hz simulation run.

The model recording contains1800 control steps including32 observation-window warm-up steps,221 requests, and one continuous session. The submitted runners record the initial-close check and scoring measurements at20 Hz for subsequent runs.
