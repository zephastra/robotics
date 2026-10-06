import json

r = json.load(open("reports/p4-h3-w5-01/report.json", encoding="utf-8"))
n = json.load(open("reports/p4-h3-w5-neg-01/report.json", encoding="utf-8"))

for tag, rep in (("POS", r), ("NEG", n)):
    print("=====", tag, "=====")
    rows = rep["h3"]
    print("  tray_on_source_band_at_handoff numbers:",
          json.dumps(rows["tray_on_source_band_at_handoff"]["numbers"], ensure_ascii=False)[:300])
    print("  tray_still numbers:",
          json.dumps(rows["tray_still_between_release_and_the_chain"]["numbers"],
                     ensure_ascii=False)[:300])
    print("  hands_clear numbers:",
          json.dumps(rows["hands_clear_of_tray_at_handoff"]["numbers"], ensure_ascii=False)[:260])
    print("  withdrew numbers:",
          json.dumps(rows["humanoid_withdrew_clear"]["numbers"], ensure_ascii=False)[:320])
    print("  parked numbers:",
          json.dumps(rows["vehicle_parked_while_humanoid_works"]["numbers"],
                     ensure_ascii=False)[:280])
    print("  chain_transactions stage_evidence_keys:",
          {k: v["stage_evidence_keys"] for k, v in rep["chain_transactions"].items()})
    print("  gate:", rep.get("gate_counters"))
    print("  world_s:", round(rep.get("world_s_elapsed") or 0, 2),
          "wall:", round(rep.get("wall_seconds") or 0, 1))
    print("  h stages:", {k: v["verdict"] for k, v in (rep.get("h_stages") or {}).items()})
    print("  feet worst pitch:", rep.get("feet"))
    print("  H2 thresholds:", json.dumps(rep.get("h2_thresholds") or {}, ensure_ascii=False)[:60])
