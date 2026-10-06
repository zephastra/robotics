import json
import sys

path = sys.argv[1] if len(sys.argv) > 1 else "reports/p4-h3-w5-01/report.json"
r = json.load(open(path, encoding="utf-8"))
print("status       :", r.get("status"))
print("error        :", r.get("error"))
print("arm          :", r.get("h3_arm"), "| not_run:", r.get("h3_rows_not_run"))
print("h3_overall   :", r.get("h3_overall"))
print("H total      :", r.get("full_H_acceptance"))
print("world        :", r.get("world"), (r.get("world_sha256") or "")[:16])
print("shared       :", r.get("shared_world", {}).get("same_model"),
      r.get("shared_world", {}).get("same_data"),
      "owns_world", r.get("shared_world", {}).get("plant_owns_world"))
print("init writes  :", r.get("init_qpos_writes"), "plant:", r.get("plant_qpos_writes"),
      "runtime:", r.get("runtime_qpos_writes"))
print("grasp/release:", r.get("grasp_s"), r.get("release_s"))
print("chassis drift:", r.get("chassis_drift_during_humanoid_m"))
print("tray start   :", r.get("tray_start"), "->", r.get("tray_at_chain_end"))
print("support      :", r.get("start_support"), "->", r.get("support_at_handoff"),
      "->", r.get("support_at_chain_end"))
print("abnormal     :", r.get("abnormal_collision_count"))
print("gate         :", r.get("gate_counters"))
print("world_s      :", round(r.get("world_s_elapsed") or 0, 2))
print()
print("=== H3 rows ===")
for k, v in (r.get("h3") or {}).items():
    print("  %-40s %-8s %s" % (k, v["verdict"], (v["detail"] or "")[:170]))
print()
print("=== transactions ===")
for leg, t in (r.get("chain_transactions") or {}).items():
    print("  %s: stage %s custody %s" % (leg, t["final_stage"], t["final_custody"]))
    print("     history        %s" % t["history"])
    print("     custody        %s" % t["custody_history"])
    print("     dest/support   %s / %s" % (t["destination_zone"], t["support_after"]))
print()
print("=== chain rows ===")
for row in r.get("chain_rows") or []:
    print("  %s %-16s acc=%-5s %-9s %-22s t=%7.2f zone %s x %.4f"
          % (row["command_id"], row["skill"], row["accepted"], row["status"],
             row["reason_code"], row["plant_clock_s"], row["tray_zone"], row["tray_x_m"]))
