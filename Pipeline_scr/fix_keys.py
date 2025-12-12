import json
import os

for filename in os.listdir('.'):
    if filename.startswith("trace-") and filename.endswith(".json"):
        print("Fixing:", filename)
        with open(filename, 'r') as f:
            data = json.load(f)

        # Fix all elements
        for elem in data:
            if "trace_writeback_insn" in elem:
                elem["trace_writeback_inst"] = elem.pop("trace_writeback_insn")

        with open(filename, 'w') as f:
            json.dump(data, f, indent=2)

print("Done.")
