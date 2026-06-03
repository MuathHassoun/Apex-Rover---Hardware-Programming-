# Optional patch for main_startup.py
# Add these imports near the top of main_startup.py:
#
# import json
# import os
#
# Then add this route near the other Flask routes.
# This makes auto status available from the mode manager port 5050:
#   http://192.168.4.X:5050/auto_status

AUTO_STATUS_FILE = "/tmp/apex_auto_status.json"

@app.route("/auto_status")
def api_auto_status():
    try:
        if not os.path.exists(AUTO_STATUS_FILE):
            return jsonify({
                "ok": False,
                "mode": current_mode,
                "error": "auto status file not created yet",
                "status_file": AUTO_STATUS_FILE,
            }), 404

        with open(AUTO_STATUS_FILE, "r", encoding="utf-8") as f:
            data = json.load(f)

        data["mode_manager_mode"] = current_mode
        data["served_by"] = "main_startup.py:5050"
        return jsonify(data)

    except Exception as e:
        return jsonify({
            "ok": False,
            "mode": current_mode,
            "error": str(e),
            "status_file": AUTO_STATUS_FILE,
        }), 500
