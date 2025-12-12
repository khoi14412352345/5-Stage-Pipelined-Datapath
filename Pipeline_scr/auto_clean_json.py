#!/usr/bin/env python3
import os
import re
import json

TARGET_DIR = "."


# Regex to remove trailing commas before ] or }
TRAILING_COMMA_PATTERN = re.compile(r",\s*([\]}])")

def clean_json_file(path):
    with open(path, "r") as f:
        content = f.read()

    cleaned = TRAILING_COMMA_PATTERN.sub(r"\1", content)

    # Try to parse after cleaning to ensure valid JSON
    try:
        json.loads(cleaned)
    except json.JSONDecodeError as e:
        print(f"❌ Still invalid JSON in: {path}: {e}")
        return

    # Backup original
    backup_path = path + ".bak"
    if not os.path.exists(backup_path):
        with open(backup_path, "w") as b:
            b.write(content)

    # Write cleaned JSON
    with open(path, "w") as f:
        f.write(cleaned)

    print(f"✔ Cleaned: {path}")

def main():
    print(f"Scanning directory: {TARGET_DIR}")
    for filename in os.listdir(TARGET_DIR):
        if filename.endswith(".json"):
            clean_json_file(os.path.join(TARGET_DIR, filename))

    print("Done.")

if __name__ == "__main__":
    main()
